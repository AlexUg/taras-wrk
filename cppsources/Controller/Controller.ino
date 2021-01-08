#include "Arduino.h"

// Необходимо заккоментировать, если не будут
// использоваться LCD-дисплей с кнопками управления
#define USE_LCD

#define DEFAULT_TOTAL_TIME		5		// в секундах
#define DEFAULT_DETECT_TIME		3		// в секундах
#define DEFAULT_RELAY_TIME		40		// в десятках миллисекунд 40 -> 400 миллисекунд
#define DEFAULT_BUTTONS_TIME	20		// в секундах

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
// ВНИМАНИЕ!!!! Если нужно изменить значения,
//              то необходимо переработать код в секции 'LCD/Keyboard Functions'.
#define BUTTON_RIGHT_PIN	A0
#define BUTTON_DOWN_PIN		A1
#define BUTTON_UP_PIN		A2
#define BUTTON_LEFT_PIN		A3

// Коды кнопок.
// Точнее биты.
// ВНИМАНИЕ!!!! Если нужно изменить значения,
//              то необходимо переработать код в секции 'LCD/Keyboard Functions'.
//              В частности, обработчик прерывания.
#define BUTTON_RIGHT_NUMBER		1
#define BUTTON_DOWN_NUMBER		2
#define BUTTON_UP_NUMBER		4
#define BUTTON_LEFT_NUMBER		8


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

	// Время ожидания нажатия кнопок до выхода из меню конфигурации.
	// Если контроллер находится в режиме конфигурирования
	// и в течении заданного времени не нажимаются кнопки,
	// то контроллер выходит из режима конфигурирования.
	// Секунды от 1 до 254.
	uint8_t buttonsTime;

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


// +++++++++++++++++++++ Controller configuration +++++++++++++++++++++

// Конфигурация, хранимая в EEPROM.
struct Config eeConfig EEMEM = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, { 1, 1, 1 } };

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

uint8_t getButtonsTime() {
	uint8_t result = config.buttonsTime;
	if ((result == 0)
			|| (result == 0xFF)) {
		result = DEFAULT_BUTTONS_TIME;
	}
	return result;
}

void setButtonsTime(uint8_t value) {
	config.buttonsTime = value;
	eeprom_update_byte(&eeConfig.buttonsTime, value);
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


// +++++++++++++++++++++ LCD/Keyboard Functions +++++++++++++++++++

// Задержка для подавления дребезга кнопок.
#define BOUNCING_DELAY	100		// in milliseconds

volatile uint8_t buttonsState;
volatile uint8_t lastChanges;
volatile unsigned long lastButtonsChange;

void (*onButtonPressed) (uint8_t button);

void defaultButtonPressed(uint8_t button);

void initButtons() {

	pinMode(BUTTON_UP_PIN, INPUT_PULLUP);
	pinMode(BUTTON_DOWN_PIN, INPUT_PULLUP);
	pinMode(BUTTON_LEFT_PIN, INPUT_PULLUP);
	pinMode(BUTTON_RIGHT_PIN, INPUT_PULLUP);

	buttonsState = PINC;
	lastChanges = 0;
	lastButtonsChange = 0;

	PCIFR = (1 << PCIF0) | (1 << PCIF1) | (1 << PCIF2);
	PCMSK1 =	(1 << PCINT8)		// Вывод A0 -- BUTTON_UP
				| (1 << PCINT9)		// Вывод A1 -- BUTTON_DOWN
				| (1 << PCINT10)	// Вывод A2 -- BUTTON_LEFT
				| (1 << PCINT11)	// Вывод A3 -- BUTTON_RIGHT
				;
	PCICR = (1 << PCIE1);	// Активация обработчика прерывания PCINT1_vect,
							// который "смотрит" за портом PINC (выводы A0 .. A5)
}

ISR(PCINT1_vect) {
	uint8_t newButtonsState = PINC;
	uint8_t changes = newButtonsState ^ buttonsState;
	if (changes) {
		if ((changes != lastChanges)
				|| ((millis() - lastButtonsChange) > BOUNCING_DELAY)) {

			Serial.print(F("nb:"));
			Serial.println(newButtonsState, 16);
			Serial.print(F("f:"));
			Serial.println((uint16_t) onButtonPressed, 16);

			for (uint8_t bit = 1; bit < 0x10; bit <<= 1) {

				Serial.print(F("i:"));
				Serial.println(bit, 16);

				if ((changes & bit)
						&& (newButtonsState & bit)
						&& onButtonPressed) {

					Serial.println(F("on"));

					onButtonPressed(bit);
				} else {

					Serial.println(F("off"));

				}
			}
		}
		lastButtonsChange = millis();
		lastChanges = changes;
		buttonsState = newButtonsState;
	}
}

// --------------------- LCD/Keyboard Functions -------------------


// +++++++++++++++++++++ StateMachine Functions +++++++++++++++++++


enum class SENSOR_STATE : uint8_t {
	INIT,					// начальное состаяние
	SENSOR_OFF,				// листа нет под сенсором
	SENSOR_ON,				// сенсор определил наличие листа
	SENSOR_ON_DETECTED,		// лист находится по сенсором время равное detectTime
	SENSOR_ON_TOTAL,		// лист находится по сенсором время равное totalTime
	CONFIGURE,				// производится настройка контроллера, сенсор не проверятеся
#ifdef USE_LCD
	CONFIGURE_TOTAL_TIME,	// производится настройка контроллера, сенсор не проверятеся
	CONFIGURE_DETECT_TIME,	// производится настройка контроллера, сенсор не проверятеся
	CONFIGURE_RELAY_TIME,	// производится настройка контроллера, сенсор не проверятеся
	CONFIGURE_BUTTONS_TIME,	// производится настройка контроллера, сенсор не проверятеся
	CONFIGURE_SENSOR_LEVEL,	// производится настройка контроллера, сенсор не проверятеся
	CONFIGURE_RELAY_LEVEL,	// производится настройка контроллера, сенсор не проверятеся
#endif
	NONE
};

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
void stateConfigureDetectTime();
void stateConfigureTotalTime();
void stateConfigureRelayTime();
void stateConfigureButtonsTime();
void stateConfigureSensorLevel();
void stateConfigureRelayLevel();


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
		lcd.noBlink();
		lcd.clear();
		lcd.setCursor(0, 0);
		lcd.print(F("Controller"));
		lcd.setCursor(0, 1);
		lcd.print(F("v0.1"));
#endif
		Serial.println(F("Controller v0.1"));
	} else if (timeInterval() >= getTotalTime()) {
		stateFunc = stateSensorOff;
	}

	onButtonPressed = defaultButtonPressed;
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

#ifdef USE_LCD

void printSeconds(uint8_t value) {
	lcd.setCursor(0, 1);
	lcd.print(value, 10);
	lcd.print(F(" sec    "));
	lcd.setCursor(0, 1);
	lcd.blink();
}

void printMillis(uint8_t value) {
	lcd.setCursor(0, 1);
	lcd.print(value, 10);
	lcd.print(F("0 ms    "));
	lcd.setCursor(0, 1);
	lcd.blink();
}

void printLevel(uint8_t value) {
	lcd.setCursor(0, 1);
	if (value) {
		lcd.print(F("LOW   "));
	} else {
		lcd.print(F("HIGH   "));
	}
	lcd.setCursor(0, 1);
	lcd.blink();
}

volatile uint8_t tempValue;
void (*nextStateFunc) ();
void (*printValueFunc) (uint8_t value);
void (*setTimeFunc) (uint8_t value);
void (*setLevelFunc) (uint8_t value);


volatile unsigned long configTime;

uint8_t checkConfigTimeout() {
	relayDeactivate();
	unsigned long delta = millis() - configTime;
	if ((delta / 1000) > getButtonsTime()) {
		stateFunc = stateInit;
		return 0;
	} else {
		return 1;
	}
}

void configureTimePressed(uint8_t button) {
	configTime = millis();
	if (button & BUTTON_UP_NUMBER) {			// увеличение значения
		tempValue++;
		if (tempValue == 0xFF) {
			tempValue = 1;
		}
		printValueFunc(tempValue);
	} else if (button & BUTTON_DOWN_NUMBER) {	// уменьшение значения
		tempValue--;
		if (tempValue == 0) {
			tempValue = 0xFE;
		}
		printValueFunc(tempValue);
	} else if (button & BUTTON_LEFT_NUMBER) {	// сохранение значения
		if (setTimeFunc) {
			setTimeFunc(tempValue);
		}
		stateFunc = nextStateFunc;
	} else if (button & BUTTON_RIGHT_NUMBER) {	// отмена изменения
		stateFunc = nextStateFunc;
	}
}

void configureLevelPressed(uint8_t button) {
	configTime = millis();
	if (button & BUTTON_UP_NUMBER) {			// увеличение значения
		tempValue = (tempValue + 1) & 1;
		printValueFunc(tempValue);
	} else if (button & BUTTON_DOWN_NUMBER) {	// уменьшение значения
		tempValue = (tempValue - 1) & 1;
		printValueFunc(tempValue);
	} else if (button & BUTTON_LEFT_NUMBER) {	// сохранение значения
		if (setTimeFunc) {
			setTimeFunc(tempValue);
		}
		stateFunc = nextStateFunc;
	} else if (button & BUTTON_RIGHT_NUMBER) {	// отмена изменения
		stateFunc = nextStateFunc;
	}
}

void stateConfigureTotalTime() {
	if (checkConfigTimeout()) {
		if (SENSOR_STATE::CONFIGURE_TOTAL_TIME != sensorState) {
			sensorState = SENSOR_STATE::CONFIGURE_TOTAL_TIME;
			tempValue = getTotalTime();
			lcd.clear();
			lcd.setCursor(0, 0);
			lcd.print(F("Total time:"));

			setTimeFunc = setTotalTime;
			printValueFunc = printSeconds;
			onButtonPressed = configureTimePressed;
			nextStateFunc = stateConfigureDetectTime;

			printValueFunc(tempValue);
		}
	}
}

void stateConfigureDetectTime() {
	if (checkConfigTimeout()) {
		if (SENSOR_STATE::CONFIGURE_DETECT_TIME != sensorState) {
			sensorState = SENSOR_STATE::CONFIGURE_DETECT_TIME;
			tempValue = getDetectTime();
			lcd.clear();
			lcd.setCursor(0, 0);
			lcd.print(F("Detect time:"));

			setTimeFunc = setDetectTime;
			printValueFunc = printSeconds;
			onButtonPressed = configureTimePressed;
			nextStateFunc = stateConfigureRelayTime;

			printValueFunc(tempValue);
		}
	}
}

void stateConfigureRelayTime() {
	if (checkConfigTimeout()) {
		if (SENSOR_STATE::CONFIGURE_RELAY_TIME != sensorState) {
			sensorState = SENSOR_STATE::CONFIGURE_RELAY_TIME;
			tempValue = getRelayTime();
			lcd.clear();
			lcd.setCursor(0, 0);
			lcd.print(F("Relay time:"));

			setTimeFunc = setRelayTime;
			printValueFunc = printMillis;
			onButtonPressed = configureTimePressed;
			nextStateFunc = stateConfigureButtonsTime;

			printValueFunc(tempValue);
		}
	}
}

void stateConfigureButtonsTime() {
	if (checkConfigTimeout()) {
		if (SENSOR_STATE::CONFIGURE_BUTTONS_TIME != sensorState) {
			sensorState = SENSOR_STATE::CONFIGURE_BUTTONS_TIME;
			tempValue = getButtonsTime();
			lcd.clear();
			lcd.setCursor(0, 0);
			lcd.print(F("Buttons time:"));

			setTimeFunc = setButtonsTime;
			printValueFunc = printSeconds;
			onButtonPressed = configureTimePressed;
			nextStateFunc = stateConfigureSensorLevel;

			printValueFunc(tempValue);
		}
	}
}

void stateConfigureSensorLevel() {
	if (checkConfigTimeout()) {
		if (SENSOR_STATE::CONFIGURE_SENSOR_LEVEL != sensorState) {
			sensorState = SENSOR_STATE::CONFIGURE_SENSOR_LEVEL;
			tempValue = getSensorActiveLow();
			lcd.clear();
			lcd.setCursor(0, 0);
			lcd.print(F("Sensor level:"));

			setLevelFunc = setSensorActiveLow;
			printValueFunc = printLevel;
			onButtonPressed = configureLevelPressed;
			nextStateFunc = stateConfigureRelayLevel;

			printValueFunc(tempValue);
		}
	}
}

void stateConfigureRelayLevel() {
	if (checkConfigTimeout()) {
		if (SENSOR_STATE::CONFIGURE_RELAY_LEVEL != sensorState) {
			sensorState = SENSOR_STATE::CONFIGURE_RELAY_LEVEL;
			tempValue = getRelayActiveLow();
			lcd.clear();
			lcd.setCursor(0, 0);
			lcd.print(F("Relay level:"));

			setLevelFunc = setRelayActiveLow;
			printValueFunc = printLevel;
			onButtonPressed = configureLevelPressed;
			nextStateFunc = stateInit;

			printValueFunc(tempValue);
		}
	}
}


void defaultButtonPressed(uint8_t button) {
	if (button) {
		stateFunc = stateConfigureTotalTime;
		configTime = millis();
	}
}

#endif

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
	lcd.begin(16, 2);

	initButtons();
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

