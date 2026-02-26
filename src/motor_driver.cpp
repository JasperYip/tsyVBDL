#include "motor_driver.h"
#include <Arduino.h>

void MotorDriver::begin(const Pins& pins) {
    pins_ = pins;
    pins_set_ = true;

    pinMode(pins_.pin_pwm, OUTPUT);
    pinMode(pins_.pin_dir, OUTPUT);
    pinMode(pins_.pin_slp, OUTPUT);
    pinMode(pins_.pin_flt, INPUT_PULLUP); // safe default if active-low
    pinMode(pins_.pin_cs, INPUT);

    // Start safe
    analogWrite(pins_.pin_pwm, 0);
    digitalWrite(pins_.pin_dir, LOW);
    digitalWrite(pins_.pin_slp, LOW); // disabled initially

    enabled_ = false;
    last_dir_extend_ = true;
    last_pwm_ = 0;
}

void MotorDriver::setAdcResolutionBits(uint8_t bits) {
    adc_bits_ = bits;
#if defined(TEENSYDUINO)
    analogReadResolution(bits);
#else
    (void)bits;
#endif
}

void MotorDriver::setSleep(bool enable_driver) {
    if (!pins_set_) return;

    if (!enable_driver) {
        // Force PWM off before sleeping
        analogWrite(pins_.pin_pwm, 0);
        last_pwm_ = 0;
        digitalWrite(pins_.pin_slp, LOW);
        enabled_ = false;
        return;
    }

    digitalWrite(pins_.pin_slp, HIGH);
    enabled_ = true;
}

void MotorDriver::setCommand(bool dir_extend, uint8_t pwm) {
    if (!pins_set_) return;

    last_dir_extend_ = dir_extend;

    // Apply direction
    digitalWrite(pins_.pin_dir, dir_extend ? HIGH : LOW);

    if (!enabled_) {
        // If asleep, don't drive PWM
        analogWrite(pins_.pin_pwm, 0);
        last_pwm_ = 0;
        return;
    }

    last_pwm_ = pwm;
    analogWrite(pins_.pin_pwm, pwm);
}

void MotorDriver::emergencyStop() {
    if (!pins_set_) return;
    analogWrite(pins_.pin_pwm, 0);
    last_pwm_ = 0;
    digitalWrite(pins_.pin_slp, LOW);
    enabled_ = false;
}

bool MotorDriver::faultAsserted() const {
    if (!pins_set_) return false;
    int v = digitalRead(pins_.pin_flt);

    // If active-low, LOW means fault.
    if (flt_active_low_) return (v == LOW);
    return (v == HIGH);
}

int MotorDriver::currentSenseRaw() const {
    if (!pins_set_) return 0;
    return analogRead(pins_.pin_cs);
}

int MotorDriver::currentMilliAmps() const {
    // TODO: implement using Pololu CS scaling and your calibration.
    // For now, return 0 to compile and allow later integration.
    return 0;
}