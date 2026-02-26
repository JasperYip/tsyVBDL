#pragma once
#include <cstdint>
#include <cmath>

// Minimal Arduino API surface used by MotorDriver
#define OUTPUT 1
#define INPUT 0
#define INPUT_PULLUP 2

#define HIGH 1
#define LOW  0

void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t value);
int  digitalRead(uint8_t pin);

void analogWrite(uint8_t pin, int value);
int  analogRead(uint8_t pin);

void analogReadResolution(int bits);

// Test helper hooks (not part of Arduino)
namespace ArduinoMock {
    void reset();

    uint8_t getPinMode(uint8_t pin);
    int getDigitalWrite(uint8_t pin);
    int getAnalogWrite(uint8_t pin);

    void setDigitalRead(uint8_t pin, int value);
    void setAnalogRead(uint8_t pin, int value);

    int getAnalogReadResolution();
}