#pragma once
#include <stdint.h>

// Minimal Pololu G2 motor driver wrapper (PWM, DIR, SLP, FLT, CS).
// - PWM: 0..255
// - DIR: true = extend, false = retract (you define meaning)
// - SLP: true = enabled/awake, false = disabled/sleep
//
// Current sense scaling is project-specific (requires calibration).
// This class provides raw ADC and a placeholder conversion hook.

class MotorDriver {
public:
    struct Pins {
        uint8_t pin_pwm;   // PWM output
        uint8_t pin_dir;   // digital output
        uint8_t pin_slp;   // digital output
        uint8_t pin_flt;   // digital input
        uint8_t pin_cs;    // analog input
    };

    MotorDriver() = default;

    void begin(const Pins& pins);

    // Enable/disable driver via SLP.
    // When disabling, PWM is forced to 0.
    void setSleep(bool enable_driver);

    // Set direction + PWM. PWM will be forced to 0 if driver is asleep.
    void setCommand(bool dir_extend, uint8_t pwm);

    // Immediate safe shutdown: PWM=0 and SLP disabled.
    void emergencyStop();

    // Reads fault pin (true = fault asserted) depending on Pololu logic.
    // NOTE: Some boards use active-low FLT. We'll treat "fault asserted" as true.
    bool faultAsserted() const;

    // Raw current-sense ADC (0..1023 or 0..4095 depending on ADC resolution).
    int currentSenseRaw() const;

    // Optional: set ADC resolution (Teensy supports analogReadResolution)
    void setAdcResolutionBits(uint8_t bits);

    // Placeholder: convert raw CS to milliamps after calibration.
    // For now returns 0 until you implement scaling.
    int currentMilliAmps() const;

    bool isEnabled() const { return enabled_; }
    bool lastDirExtend() const { return last_dir_extend_; }
    uint8_t lastPwm() const { return last_pwm_; }

private:
    Pins pins_{};
    bool pins_set_ = false;

    bool enabled_ = false;
    bool last_dir_extend_ = true;
    uint8_t last_pwm_ = 0;

    uint8_t adc_bits_ = 10; // Arduino default

    bool flt_active_low_ = true; // Pololu FLT is commonly active-low; confirm on your board
};