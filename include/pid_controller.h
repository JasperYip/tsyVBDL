#pragma once
#include <stdint.h>

// Simple PID controller for position control.
// - Uses float units (we will use mm for error/measurement).
// - Fixed dt (seconds) provided on each update.
// - Derivative is computed on measurement (recommended).
// - Includes deadband, output clamp, integrator clamp (anti-windup).

class PIDController {
public:
    struct Config {
        float kp = 0.0f;
        float ki = 0.0f;   // per second
        float kd = 0.0f;   // seconds

        float dt_s = 0.001f;          // default 1 kHz
        float deadband_mm = 0.0f;     // set >0 to reduce dithering

        float out_min = -1.0f;        // normalized effort by default
        float out_max =  1.0f;

        // Integrator clamp in output-units (same units as output).
        // This is a simple, robust anti-windup method.
        float i_min = -0.5f;
        float i_max =  0.5f;

        // Optional: clamp derivative term to avoid noise spikes (0 = disabled)
        float d_max_abs = 0.0f;
    };

    PIDController() = default;

    void setConfig(const Config& cfg);
    const Config& getConfig() const { return cfg_; }

    // Reset internal state (integrator, previous measurement).
    void reset(float measurement_mm = 0.0f);

    // Update using setpoint and measurement in mm.
    // Returns output (normalized effort by default).
    float update(float setpoint_mm, float measurement_mm);

    // Useful for logging
    float lastError() const { return last_error_; }
    float lastOutput() const { return last_output_; }
    float integrator() const { return i_term_; }
    float derivativeMmPerSec() const { return d_meas_mm_s_; }

private:
    Config cfg_{};

    bool has_prev_meas_ = false;
    float prev_meas_mm_ = 0.0f;

    float last_error_ = 0.0f;
    float last_output_ = 0.0f;

    float i_term_ = 0.0f;         // integrator term in output-units
    float d_meas_mm_s_ = 0.0f;    // measurement derivative (mm/s)

    static float clamp(float v, float lo, float hi);
};