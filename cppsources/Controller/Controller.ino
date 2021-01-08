#include "Arduino.h"

// Необходимо заккоментировать, если не будут
// использоваться LCD-дисплей с кнопками управления
#define USE_LCD

#define DEFAULT_TOTAL_TIME	5		// в секундах
#define DEFAULT_DETECT_TIME	3		// в секундах
#define DEFAULT_RELAY_TIME	40		// в десятках миллисекунд 40 -> 400 миллисекунд

// Если сенсор при наличии листа выдаёт низкий уровень, то значение 1.
// Если -- высокий, то значение 0.
#define SENSOR_ACTIVE_LOW	1

// Если реле включается низким уровнем, то значение 1.
// Если -- высоким, то значение 0.
#define RELAY_ACTIVE_LOW	1

// Выводы модуля Arduino для подключения сенсора и реле
#define SENSOR	2
#define RELAY	3

// Выводы модуля Arduino для подключения кнопок управлдения
#define BUTTON_UP		A0
#define BUTTON_DOWN		A1
#define BUTTON_LEFT		A2
#define BUTTON_RIGHT	A3


#include <avr/eeprom.h>

#ifdef USE_LCD
#include <LiquidCrystal.h>

LiquidCrystal lcd( 8, 9, 4, 5, 6, 7);
#endif


// Структура, содержащая конфигурацию устройства.
// Значение параметра равное 0 или 0xFF говорит о том,
// что параметр не задан.
struct Config {
	// Обход дефекта, связанного с нулевой ячейкой EEPROM.
	uint8_t reserved;

	// Общее время нахождения листа в зоне действия устройства.
	// Секунды от 1 до 254.
	uint8_t totalTime;

	// Время определения наличия железного листа в зоне действия устройства.
	// Секунды от 1 до 254.
	uint8_t detectTime;

	// Время срабатывания реле. Период включения-выключения будет relayTime * 2.
	// Десятки миллисекунд от 1 до 254.
	uint8_t relayTime;

	struct {
		// Если сенсор при наличии листа выдаёт низкий уровень, то значение 1.
		// Если -- высокий, то значение 0.
		uint8_t sensorActiveLow : 1;

		// Если реле включается низким уровнем, то значение 1.
		// Если -- высоким, то значение 0.
		uint8_t relayActiveLow : 1;

		// Бит статуса.
		// Если 0, то флаги заданы.
		// Если 1, то -- нет.
		uint8_t status : 1;
	} flags;
};

enum class SENSOR_STATE : uint8_t {
	INIT,					// начальное состаяние
	SENSOR_OFF,				// листа нет под сенсором
	SENSOR_ON,				// сенсор определил наличие листа
	SENSOR_ON_DETECTED,		// лист находится по сенсором время равное detectTime
	SENSOR_ON_TOTAL,		// лист находится по сенсором время равное totalTime
	CONFIGURE,				// производится настройка контроллера, сенсор не проверятеся
	NONE
};


// +++++++++++++++++++++ Controller onfiguration +++++++++++++++++++++

// Конфигурация, хранимая в EEPROM.
struct Config eeConfig EEMEM = { 0xFF, 0xFF, 0xFF, 0xFF, { 1, 1, 1 } };

// Копия конфигурации в ОЗУ.
struct Config config;

static inline void loadConfiguration() {
	eeprom_read_block(&config, &eeConfig, sizeof(Config));
}

uint8_t getTotalTime() {
	uint8_t result = config.totalTime;
	if ((result == 0)
			|| (result == 0xFF)) {
		result = DEFAULT_TOTAL_TIME;
	}
	return result;
}

void setTotalTime(uint8_t value) {
	config.totalTime = value;
	eeprom_update_byte(&eeConfig.totalTime, value);
}

uint8_t getDetectTime() {
	uint8_t result = config.detectTime;
	if ((result == 0)
			|| (result == 0xFF)) {
		result = DEFAULT_DETECT_TIME;
	}
	return result;
}

void setDetectTime(uint8_t value) {
	config.detectTime = value;
	eeprom_update_byte(&eeConfig.detectTime, value);
}

uint8_t getRelayTime() {
	uint8_t result = config.relayTime;
	if ((result == 0)
			|| (result == 0xFF)) {
		result = DEFAULT_RELAY_TIME;
	}
	return result;
}

void setRelayTime(uint8_t value) {
	config.relayTime = value;
	eeprom_update_byte(&eeConfig.relayTime, value);
}

uint8_t getSensorActiveLow() {
	if (config.flags.status > 0) {
		return SENSOR_ACTIVE_LOW;
	} else {
		return config.flags.sensorActiveLow;
	}
}

void setSensorActiveLow(uint8_t value) {
	config.flags.status = 1;
	config.flags.sensorActiveLow = value;
	eeprom_update_block(&config.flags, &eeConfig.flags, sizeof(uint8_t));
}

uint8_t getRelayActiveLow() {
	if (config.flags.status > 0) {
		return RELAY_ACTIVE_LOW;
	} else {
		return config.flags.relayActiveLow;
	}
}

void setRelayActiveLow(uint8_t value) {
	config.flags.status = 1;
	config.flags.relayActiveLow = value;
	eeprom_update_block(&config.flags, &eeConfig.flags, sizeof(uint8_t));
}

// --------------------- Controller configuration ---------------------


// +++++++++++++++++++++ StateMachine Functions +++++++++++++++++++

// Новое состояние
volatile SENSOR_STATE sensorState;

// Время переключения состояния (миллисекунды от старта МК)
volatile unsigned long switchStateTime;


void (*stateFunc) ();

void stateInit();
void stateSensorOff();
void stateSensorOn();
void stateSensorOnDetected();
void stateSensorOnTotal();


// Возвращяет время прошедшее с последнего переключения состояния.
// В секундах.
uint8_t timeInterval() {
	unsigned long delta = millis() - switchStateTime;
	return delta / 1000;
}

void stateInit() {
	if (SENSOR_STATE::INIT != sensorState) {
		sensorState = SENSOR_STATE::INIT;
		switchStateTime = millis();
#ifdef USE_LCD
		lcd.clear();
		lcd.setCursor(0, 0);
		lcd.print(F("Controller"));
		lcd.setCursor(0, 1);
		lcd.print(F("v 0.1"));
#endif
		Serial.println(F("Controller"));
		Serial.println(F("v 0.1"));
	} else if (timeInterval() >= getTotalTime()) {
		stateFunc = stateSensorOff;
	}
}

void stateSensorOff() {
	relayDeactivate();
	if (SENSOR_STATE::SENSOR_OFF != sensorState) {
		sensorState = SENSOR_STATE::SENSOR_OFF;
		switchStateTime = millis();
#ifdef USE_LCD
		lcd.clear();
		lcd.setCursor(0, 0);
		lcd.print(F("Sensor OFF"));
#endif
		Serial.println(F("Sensor OFF"));
	}
}

void stateSensorOn() {
	relayDeactivate();
	uint8_t ti = timeInterval();
	if (SENSOR_STATE::SENSOR_ON != sensorState) {
		sensorState = SENSOR_STATE::SENSOR_ON;
		switchStateTime = millis();
#ifdef USE_LCD
		lcd.clear();
		lcd.setCursor(0, 0);
		lcd.print(F("Sensor ON"));
#endif
		Serial.println(F("Sensor ON"));
	} else if (ti >= getDetectTime()) {
		stateFunc = stateSensorOnDetected;
	} else {
#ifdef USE_LCD
		lcd.setCursor(10, 0);
		lcd.print(ti, 10);
		lcd.print(F(" s  "));
#endif
		Serial.print(ti, 10);
		Serial.println(F("sec"));
	}
}

void stateSensorOnDetected() {
	uint8_t ti = timeInterval();
	if (SENSOR_STATE::SENSOR_ON_DETECTED != sensorState) {
		sensorState = SENSOR_STATE::SENSOR_ON_DETECTED;
		switchStateTime = millis();
		relayActivate();
#ifdef USE_LCD
		lcd.clear();
		lcd.setCursor(0, 0);
		lcd.print(F("Sensor ON"));
		lcd.setCursor(0, 1);
		lcd.print(F("Plate detected"));
#endif
		Serial.println(F("Plate detected"));
	} else if (ti >= (getTotalTime() - getDetectTime())) {
		stateFunc = stateSensorOnTotal;
	} else {
#ifdef USE_LCD
		lcd.setCursor(10, 0);
		lcd.print(ti, 10);
		lcd.print(F(" s  "));
#endif
		Serial.print(ti, 10);
		Serial.println(F("sec"));
	}
}

void stateSensorOnTotal() {
	relayDeactivate();
	if (SENSOR_STATE::SENSOR_ON_TOTAL != sensorState) {
		sensorState = SENSOR_STATE::SENSOR_ON_TOTAL;
		switchStateTime = millis();
		lcd.clear();
#ifdef USE_LCD
		lcd.setCursor(0, 0);
		lcd.print(F("Sensor ON"));
		lcd.setCursor(0, 1);
		lcd.print(F("Time elapsed"));
#endif
		Serial.println(F("Time elapsed"));
	}
}

// --------------------- StateMachine Functions -------------------


void setup() {

	sensorState = SENSOR_STATE::NONE;
	stateFunc = stateInit;

	loadConfiguration();

	Serial.begin(9600);

	if (getRelayActiveLow()) {
		digitalWrite(RELAY, HIGH);
	} else {
		digitalWrite(RELAY, LOW);
	}
	pinMode(RELAY, OUTPUT);
	relayDeactivate();

	if (getSensorActiveLow()) {
		pinMode(SENSOR, INPUT_PULLUP);
	} else {
		pinMode(SENSOR, INPUT);
	}

#ifdef USE_LCD
	pinMode(BUTTON_UP, INPUT_PULLUP);
	pinMode(BUTTON_DOWN, INPUT_PULLUP);
	pinMode(BUTTON_LEFT, INPUT_PULLUP);
	pinMode(BUTTON_RIGHT, INPUT_PULLUP);

	lcd.begin(16, 2);
#endif

	attachInterrupt(digitalPinToInterrupt(SENSOR), sensorInterrupt, CHANGE);

	switchStateTime = millis();

}


void loop() {
	stateFunc();
}


void sensorInterrupt() {
	uint8_t pinState = digitalRead(SENSOR) & 1;

	if (getSensorActiveLow()) {
		pinState = (pinState + 1) & 1;
	}

	if (sensorState < SENSOR_STATE::CONFIGURE) {
		if (pinState) {
			stateFunc = stateSensorOn;
		} else {
			stateFunc = stateSensorOff;
		}
	}
}


void relayOn() {
	if (getRelayActiveLow()) {
		digitalWrite(RELAY, LOW);
	} else {
		digitalWrite(RELAY, HIGH);
	}
}

void relayOff() {
	if (getRelayActiveLow()) {
		digitalWrite(RELAY, HIGH);
	} else {
		digitalWrite(RELAY, LOW);
	}
}

void relayActivate() {

	uint16_t period = getRelayTime() * 2 * 10 * 8;

	TCNT1 = 0;

	TCCR1A = (1 << WGM11) | (1 << WGM10);
	TCCR1B = (1 << WGM13) | (1 << WGM12)
				| (1 << CS12) | (1 << CS10);	// Timer1 prescaller = 1024

	OCR1A = period;
	OCR1B = period / 2;

	TIFR1 = (1 << OCF1B) | (1 << OCF1A) | (1 << TOV1);
	TIMSK1 = (1 << OCIE1B) | (1 << TOIE1);

	relayOn();
}

void relayDeactivate() {

	TCCR1B = 0;
	TIFR1 = (1 << OCF1B) | (1 << OCF1A) | (1 << TOV1);
	TIMSK1 = 0;

	relayOff();
}


ISR(TIMER1_COMPB_vect) {
	relayOff();
}

ISR(TIMER1_OVF_vect) {
	relayOn();
}

