#include "Arduino.h"

#define DEFAULT_TOTALTIME	5
#define DEFAULT_DETECTTIME	3
#define DEFAULT_COUNT		3

#define SENSOR	2
#define RELAY	3

// Если реле включается низким уровнем, то значение 1.
// Если высоким, то значение 0.
#define RELAY_ACTIVE_LOW	1

#define USE_LCD

#define BUTTON_UP		A0
#define BUTTON_DOWN		A1
#define BUTTON_LEFT		A2
#define BUTTON_RIGHT	A3


#include <avr/eeprom.h>

#ifdef USE_LCD
#include <LiquidCrystal.h>
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

enum SENSOR_STATE {
	SENSOR_OFF,				// листа нет под сенсором
	SENSOR_ON,				// сенсор определил наличие листа
	SENSOR_ON_DETECTED,		// лист находится по сенсором время равное detectTime
	SENSOR_ON_TOTAL,		// лист находится по сенсором время равное totalTime
	CONFIGURE				// производится настройка контроллера, сенсор не проверятеся
};

struct Config eeConfig EEMEM = { -1, -1, -1, -1 };

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
		result = DEFAULT_DETECTTIME;
	}
	return result;
}

void setRelayTime(uint8_t value) {
	eeprom_update_byte(&eeConfig.relayTime, value);
}

struct Config config;

#ifdef USE_LCD
// display pins:   RS E D4 D5 D6 D7
LiquidCrystal lcd( 9, 8, 7, 6, 5, 4);
#endif

volatile SENSOR_STATE sensorState;
volatile unsigned long timeInState;

void sensorInterrupt() {
	if (sensorState < CONFIGURE) {
		if (digitalRead(SENSOR)) {
			sensorState = SENSOR_OFF;
		} else {
			sensorState = SENSOR_ON;
		}
		timeInState = millis();
	}
}

int8_t timeInterval() {
	unsigned long delta = millis() - timeInState;
	int8_t result = delta / 1000;
	if (result < 0) {
		result = 127;
	}
	return result;
}

void nextSensorState() {
	if (sensorState < SENSOR_ON_TOTAL) {
		timeInState = millis();
		sensorState++;
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

	TIMER1A = (1 << WGM11) | (1 << WGM10);
	TIMER1B = (1 << WGM13) | (1 << WGM12)
				| (1 << CS12);	// Timer1 prescaller = 256

	uint16_t period = (getRelayTime() * 10) / 16;
	OCR1A = period;
	OCR1B = period / 2;

	TIFR1 = (1 << OCF1B) | (1 << OCF1A) | (1 << TOV1);
	TIMSK1 = (1 << OCIE1B) | (1 << TOIE1);
}

void relayDeactivate() {

	TIMER1B = 0;
	TIFR1 = (1 << OCF1B) | (1 << OCF1A) | (1 << TOV1);
	TIMSK1 = 0;

	relayOff();
}

void setup() {

	sensorState = SENSOR_OFF;
	config.totalTime = getTotalTime();
	config.detectTime = getDetectTime();
	config.relayTime = getRelayTime();

	Serial.begin(9600);

#if RELAY_ACTIVE_LOW == 1
	digitalWrite(RELAY, HIGH);
#else
	digitalWrite(RELAY, LOW);
#endif
	pinMode(RELAY, OUTPUT);
	relayDeactivate();

	pinMode(SENSOR, INPUT);

#ifdef USE_LCD
	pinMode(BUTTON_UP, INPUT_PULLUP);
	pinMode(BUTTON_DOWN, INPUT_PULLUP);
	pinMode(BUTTON_LEFT, INPUT_PULLUP);
	pinMode(BUTTON_RIGHT, INPUT_PULLUP);

	lcd.begin(16, 2);
	lcd.setCursor(0, 0);
	lcd.print(F("Controller"));
	lcd.setCursor(0, 1);
	lcd.print(F("v 0.1"));
#endif

	attachInterrupt(digitalPinToInterrupt(SENSOR), sensorInterrupt, CHANGE);
}


void loop() {

static SENSOR_STATE lastSensorState = sensorState;

	switch (sensorState) {
	case SENSOR_OFF:
		relayDeactivate();
		if (lastSensorState != sensorState) {
			lcd.clear();
			lcd.setCursor(0, 0);
			lcd.print(F("Sensor OFF"));
			Serial.print(F("Sensor OFF"));
		}
		break;
	case SENSOR_ON:
		relayDeactivate();
		if (timeInterval() >= getDetectTime()) {
			nextSensorState();
		} else if (lastSensorState != sensorState) {
			lcd.clear();
			lcd.setCursor(0, 0);
			lcd.print(F("Sensor ON"));
			Serial.print(F("Sensor ON"));
		}
		break;
	case SENSOR_ON_DETECTED:
		if (timeInterval() >= getTotalTime()) {
			nextSensorState();
		} else if (lastSensorState != sensorState) {
			relayActivate();
			lcd.clear();
			lcd.setCursor(0, 0);
			lcd.print(F("Sensor ON"));
			lcd.setCursor(0, 1);
			lcd.print(F("Plate detected"));
			Serial.print(F("Plate detected"));
		}
		break;
	case SENSOR_ON_TOTAL:
		relayDeactivate();
		if (lastSensorState != sensorState) {
			lcd.clear();
			lcd.setCursor(0, 0);
			lcd.print(F("Sensor ON"));
			lcd.setCursor(0, 1);
			lcd.print(F("Time elapsed"));
			Serial.print(F("Time elapsed"));
		}
		break;
	default:
		relayDeactivate();
		break;
	}
	lastSensorState = sensorState;
}

ISR(TIMER1_COMPA_vect) {
	relayOff();
}

ISR(TIMER1_OVF_vect) {
	relayOn();
}

