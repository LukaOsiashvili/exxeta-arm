// Pin map of the exxeta-arm controller. MUST match docs/hardware/pinmap.md.
//
// Board: ESP32 DevKit, 38 pins, ESP32-WROOM-32 (classic ESP32).
// To use another board variant, change only this file (and the docs table).
//
// Rules behind the choices (PROJECT_SPEC 5.1):
//  * analog inputs only on ADC1 (GPIO 32-39): ADC2 does not work while Wi-Fi is on
//  * no strapping pins (0, 2, 5, 12, 15), no flash pins (6-11)
//  * GPIO 34-39 are input-only: fine for analog inputs, useless for servos
//  * GPIO 21/22 stay free for I2C (optional ADS1115 ADC)
#pragma once
#include <stdint.h>

// Servo signal outputs, J1..J6 (3.3 V logic, 50 Hz PWM)
static const uint8_t PIN_SERVO[6] = {13, 16, 17, 18, 19, 23};

// Optional joint pots J1..J4 (wiper to pin, ends to 3V3 and GND)
static const uint8_t PIN_POT[4] = {36, 39, 34, 35};

static const uint8_t PIN_BATTERY = 32;     // pack voltage through the 100k / 33k divider
static const uint8_t PIN_ADC_SPARE = 33;   // free ADC1 pin (ADR-0002: J5 pot or battery mid-point later)
static const uint8_t PIN_RGB_LED = 27;     // WS2812 / SK6812 data
