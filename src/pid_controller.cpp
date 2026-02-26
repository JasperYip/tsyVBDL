#include "pid_controller.h"
#include <math.h>

float PIDController::clamp(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void PIDController::setConfig(const Config& cfg) {
    cfg_ = cfg;

    // Basic sanity: avoid zero/negative dt
    if (cfg_.dt_s <= 0.0f) cfg_.dt_s = 0.001f;

    // Ensure bounds sane
    if (cfg_.out_min > cfg_.out_max) {
        float tmp = cfg_.out_min;
        cfg_.out_min = cfg_.out_max;
        cfg_.out_max = tmp;
    }
    if (cfg_.i_min > cfg_.i_max) {
        float tmp = cfg_.i_min;
        cfg_.i_min = cfg_.i_max;
        cfg_.i_max = tmp;
    }
}

void PIDController::reset(float measurement_mm) {
    has_prev_meas_ = true;
    prev_meas_mm_ = measurement_mm;

    last_error_ = 0.0f;
    last_output_ = 0.0f;

    i_term_ = 0.0f;
    d_meas_mm_s_ = 0.0f;
}

float PIDController::update(float setpoint_mm, float measurement_mm) {
    const float dt = cfg_.dt_s;

    // Error (mm)
    float error = setpoint_mm - measurement_mm;

    // Deadband: eliminate tiny corrections that cause dithering
    if (cfg_.deadband_mm > 0.0f && fabsf(error) < cfg_.deadband_mm) {
        error = 0.0f;
    }

    // Derivative on measurement (mm/s), avoids derivative kick on setpoint jumps
    if (!has_prev_meas_) {
        prev_meas_mm_ = measurement_mm;
        has_prev_meas_ = true;
    }
    d_meas_mm_s_ = (measurement_mm - prev_meas_mm_) / dt;
    prev_meas_mm_ = measurement_mm;

    // P term (output-units)
    const float p = cfg_.kp * error;

    // I term update (output-units)
    // Simple integrator with clamp as anti-windup.
    // Integrating error -> output domain via Ki (per second).
    i_term_ += cfg_.ki * error * dt;
    i_term_ = clamp(i_term_, cfg_.i_min, cfg_.i_max);

    // D term (output-units) based on measurement derivative
    // Negative sign: if measurement increasing, derivative should reduce effort.
    float d = -cfg_.kd * d_meas_mm_s_;

    // Optional clamp on D to reject spikes
    if (cfg_.d_max_abs > 0.0f) {
        d = clamp(d, -cfg_.d_max_abs, cfg_.d_max_abs);
    }

    float out = p + i_term_ + d;

    // Output clamp (saturation)
    out = clamp(out, cfg_.out_min, cfg_.out_max);

    last_error_ = error;
    last_output_ = out;
    return out;
}