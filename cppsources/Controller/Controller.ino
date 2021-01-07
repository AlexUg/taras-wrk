#include "Arduino.h"

#define DEFAULT_TOTALTIME	5
#define DEFAULT_DETECTTIME	3
#define DEFAULT_RELAYTIME	20

#define SENSOR	2
#define RELAY	3

// Если сенсор при наличии листа выдаёт низкий уровень, то значение 1.
// Если -- высокий, то значение 0.
#define SENSOR_ACTIVE_LOW	1

// Если реле включается низким уровнем, то значение 1.
// Если -- высоким, то значение 0.
#define RELAY_ACTIVE_LOW	1

#define USE_LCD

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
// Любое отрицательное значение параметра говорит о том,
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

	// Время срабатывания реле.
	// Десятки миллисекунд от 1 до 254.
	uint8_t relayTime;
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


// +++++++++++++++++++++ EEPROM configuration +++++++++++++++++++++

struct Config eeConfig EEMEM = { 0xFF, 0xFF, 0xFF, 0xFF };

uint8_t getTotalTime() {
	uint8_t result = eeprom_read_byte(&eeConfig.totalTime);
	if ((result == 0)
			|| (result == 0xFF)) {
		result = DEFAULT_TOTALTIME;
	}
	return result;
}

void setTotalTime(uint8_t value) {
	eeprom_update_byte(&eeConfig.totalTime, value);
}

uint8_t getDetectTime() {
	uint8_t result = eeprom_read_byte(&eeConfig.detectTime);
	if ((result == 0)
			|| (result == 0xFF)) {
		result = DEFAULT_DETECTTIME;
	}
	return result;
}

void setDetectTime(uint8_t value) {
	eeprom_update_byte(&eeConfig.detectTime, value);
}

uint8_t getRelayTime() {
	uint8_t result = eeprom_read_byte(&eeConfig.relayTime);
	if ((result == 0)
			|| (result == 0xFF)) {
		result = DEFAULT_RELAYTIME;
	}
	return result;
}

void setRelayTime(uint8_t value) {
	eeprom_update_byte(&eeConfig.relayTime, value);
}

// --------------------- EEPROM configuration ---------------------


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
	if (SENSOR_STATE::SENSOR_ON != sensorState) {
		sensorState = SENSOR_STATE::SENSOR_ON;
#ifdef USE_LCD
		lcd.clear();
		lcd.setCursor(0, 0);
		lcd.print(F("Sensor ON"));
#endif
		Serial.println(F("Sensor ON"));
	} else if (timeInterval() >= getDetectTime()) {
		stateFunc = stateSensorOnDetected;
	}
}

void stateSensorOnDetected() {
	if (SENSOR_STATE::SENSOR_ON_DETECTED != sensorState) {
		sensorState = SENSOR_STATE::SENSOR_ON_DETECTED;
//		relayActivate();
#ifdef USE_LCD
		lcd.clear();
		lcd.setCursor(0, 0);
		lcd.print(F("Sensor ON"));
		lcd.setCursor(0, 1);
		lcd.print(F("Plate detected"));
#endif
		Serial.println(F("Plate detected"));
	} else if (timeInterval() >= getTotalTime()) {
		stateFunc = stateSensorOnTotal;
	}
}

void stateSensorOnTotal() {
	relayDeactivate();
	if (SENSOR_STATE::SENSOR_ON_TOTAL != sensorState) {
		sensorState = SENSOR_STATE::SENSOR_ON_TOTAL;
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

	Serial.begin(9600);

#if RELAY_ACTIVE_LOW == 1
	digitalWrite(RELAY, HIGH);
#else
	digitalWrite(RELAY, LOW);
#endif
	pinMode(RELAY, OUTPUT);
	relayDeactivate();

#if SENSOR_ACTIVE_LOW == 1
	pinMode(SENSOR, INPUT_PULLUP);
#else
	pinMode(SENSOR, INPUT);
#endif

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

#if SENSOR_ACTIVE_LOW == 1
	pinState = (pinState + 1) & 1;
#endif

	if (sensorState < SENSOR_STATE::CONFIGURE) {
		if (pinState) {
			stateFunc = stateSensorOn;
		} else {
			stateFunc = stateSensorOff;
		}
		switchStateTime = millis();
	}
}


void relayOn() {
#if RELAY_ACTIVE_LOW == 1
	digitalWrite(RELAY, LOW);
#else
	digitalWrite(RELAY, HIGH);
#endif
}

void relayOff() {
#if RELAY_ACTIVE_LOW == 1
	digitalWrite(RELAY, HIGH);
#else
	digitalWrite(RELAY, LOW);
#endif
}

void relayActivate() {
	relayOn();

	TCCR1A = (1 << WGM11) | (1 << WGM10);
	TCCR1B = (1 << WGM13) | (1 << WGM12)
				| (1 << CS12);	// Timer1 prescaller = 256

	uint16_t period = (getRelayTime() * 10) / 16;
	OCR1A = period;
	OCR1B = period / 2;

	TIFR1 = (1 << OCF1B) | (1 << OCF1A) | (1 << TOV1);
	TIMSK1 = (1 << OCIE1B) | (1 << TOIE1);
}

void relayDeactivate() {

	TCCR1B = 0;
	TIFR1 = (1 << OCF1B) | (1 << OCF1A) | (1 << TOV1);
	TIMSK1 = 0;

	relayOff();
}


ISR(TIMER1_COMPA_vect) {
	relayOff();
}

ISR(TIMER1_OVF_vect) {
	relayOn();
}

