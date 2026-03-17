#pragma once

#include "controllers/pid_controller.hpp"
#include "controllers/estimator.hpp"

class PistonController {
public:

    enum class State {
        UNHOMED,
        HOMING,
        RUN,
        HOLD,
        FAULT
    };

    struct Inputs {
        float pos_est_mm;
        float vel_est_mm_s;

        bool prox_triggered;

        bool hard_fault;
        bool soft_fault;

        bool cmd_enable;
        bool cmd_home;

        float target_percent;
    };

    struct Output {
        bool dir_extend;
        float duty;
        State state;
        float target_mm;
        bool homing_failed;
    };

    void configurePID(const PIDController::Config& cfg);

    void reset();

    Output update(const Inputs& in, float dt);

private:

    float percentToMm(float pct) const;
    float applySoftZone(float duty, float pos_mm) const;

    PIDController pid_;

    State state_ = State::UNHOMED;

    float target_mm_ = 10.0f;

    bool homing_backoff_ = false;

    float homing_timer_s_ = 0.0f;
};