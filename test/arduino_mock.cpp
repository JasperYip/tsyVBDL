#include "Arduino.h"
#include <array>
#include <algorithm>

namespace {
    constexpr int MAX_PINS = 256;

    std::array<uint8_t, MAX_PINS> g_pinMode{};
    std::array<int,     MAX_PINS> g_digitalWrite{};
    std::array<int,     MAX_PINS> g_digitalRead{};
    std::array<int,     MAX_PINS> g_analogWrite{};
    std::array<int,     MAX_PINS> g_analogRead{};
    int g_adcBits = 10;
}

void pinMode(uint8_t pin, uint8_t mode) {
    g_pinMode[pin] = mode;
}

void digitalWrite(uint8_t pin, uint8_t value) {
    g_digitalWrite[pin] = value;
}

int digitalRead(uint8_t pin) {
    return g_digitalRead[pin];
}

void analogWrite(uint8_t pin, int value) {
    g_analogWrite[pin] = value;
}

int analogRead(uint8_t pin) {
    return g_analogRead[pin];
}

void analogReadResolution(int bits) {
    g_adcBits = bits;
}

namespace ArduinoMock {

void reset() {
    g_pinMode.fill(0);
    g_digitalWrite.fill(0);
    g_digitalRead.fill(0);
    g_analogWrite.fill(0);
    g_analogRead.fill(0);
    g_adcBits = 10;
}

uint8_t getPinMode(uint8_t pin) { return g_pinMode[pin]; }
int getDigitalWrite(uint8_t pin) { return g_digitalWrite[pin]; }
int getAnalogWrite(uint8_t pin) { return g_analogWrite[pin]; }

void setDigitalRead(uint8_t pin, int value) { g_digitalRead[pin] = value; }
void setAnalogRead(uint8_t pin, int value) { g_analogRead[pin] = value; }

int getAnalogReadResolution() { return g_adcBits; }

} // namespace ArduinoMock