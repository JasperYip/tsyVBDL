#include "controllers/piston_controller.hpp"
#include "config/constants.hpp"
#include <algorithm>
#include <cmath>

void PistonController::configurePID(const PIDController::Config& cfg)
{
    pid_.configure(cfg);
}

void PistonController::reset()
{
    state_ = State::UNHOMED;
    pid_.reset();
    homing_timer_s_ = 0.0f;
}

float PistonController::percentToMm(float pct) const
{
    pct = std::clamp(pct, 0.0f, 100.0f);

    return config::STROKE_HARD_MIN_MM +
           (pct / 100.0f) * config::STROKE_SPAN_MM;
}

float PistonController::applySoftZone(float duty, float pos_mm) const
{
    float limit = 1.0f;

    if (pos_mm < config::STROKE_SOFT_END_MM)
        limit = config::SOFT_ZONE_MAX_DUTY;

    if (pos_mm > config::STROKE_SOFT_START_MM)
        limit = config::SOFT_ZONE_MAX_DUTY;

    return std::clamp(duty, -limit, limit);
}

PistonController::Output
PistonController::update(const Inputs& in, float dt)
{
    Output out{};
    out.state = state_;
    out.target_mm = target_mm_;
    out.homing_failed = false;

    if (in.hard_fault) {
        state_ = State::FAULT;
    }

    switch (state_) {

    case State::UNHOMED:
        out.duty = 0.0f;

        if (in.cmd_home)
        {
            state_ = State::HOMING;
            homing_backoff_ = false;
            homing_timer_s_ = 0.0f;
        }
        break;

    case State::HOMING:

        homing_timer_s_ += dt;

        if (homing_timer_s_ > config::HOMING_TIMEOUT_S)
        {
            out.homing_failed = true;
            state_ = State::FAULT;
            out.duty = 0.0f;
            break;
        }

        if (!homing_backoff_)
        {
            if (!in.prox_triggered)
            {
                out.dir_extend = false;
                out.duty = config::HOMING_PWM;
            }
            else
            {
                homing_backoff_ = true;
            }
        }
        else
        {
            if (in.prox_triggered)
            {
                out.dir_extend = true;
                out.duty = config::HOMING_BACKOFF_DUTY;
            }
            else
            {
                pid_.reset();
                target_mm_ = config::STROKE_HARD_MIN_MM;
                state_ = State::RUN;
                homing_timer_s_ = 0.0f;
            }
        }

        break;

    case State::RUN:

        if (!in.cmd_enable)
        {
            state_ = State::HOLD;
            break;
        }

        target_mm_ = percentToMm(in.target_percent);

        {
            float error = target_mm_ - in.pos_est_mm;

            if (std::fabs(error) < config::POS_TOL_MM)
            {
                state_ = State::HOLD;
                break;
            }

            float effort =
                pid_.update(target_mm_,
                            in.pos_est_mm,
                            in.vel_est_mm_s,
                            dt);

            effort = applySoftZone(effort, in.pos_est_mm);

            out.dir_extend = effort > 0.0f;
            out.duty = std::fabs(effort);
        }

        break;

    case State::HOLD:

        out.duty = 0.0f;

        if (in.cmd_enable)
        {
            float error = target_mm_ - in.pos_est_mm;

            if (std::fabs(error) > config::DRIFT_RESTART_MM)
            {
                state_ = State::RUN;
            }
        }

        break;

    case State::FAULT:

        out.duty = 0.0f;
        break;
    }

    out.state = state_;
    out.target_mm = target_mm_;

    return out;
}