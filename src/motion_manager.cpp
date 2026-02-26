#include "motion_manager.h"
#include <math.h>

MotionManager::MotionManager() {
    PIDController::Config pcfg;
    pcfg.kp = PID_KP;
    pcfg.ki = PID_KI;
    pcfg.kd = PID_KD;

    pcfg.dt_s = CONTROL_DT_S;
    pcfg.deadband_mm = PID_DEADBAND_MM;

    pcfg.out_min = PID_OUT_MIN;
    pcfg.out_max = PID_OUT_MAX;

    pcfg.i_min = PID_I_MIN;
    pcfg.i_max = PID_I_MAX;

    pcfg.d_max_abs = PID_D_MAX_ABS;

    pid_.setConfig(pcfg);
}

void MotionManager::setPidConfig(const PIDController::Config& pid_cfg) {
    pid_.setConfig(pid_cfg);
}

void MotionManager::setCommand(const CmdSetpoint& cmd) {
    cmd_ = cmd;
}

float MotionManager::clampf(float v, float lo, float hi) const {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

float MotionManager::pctToMm(uint8_t pct) const {
    if (pct > 100) pct = 100;
    const float span = STROKE_HARD_MAX_MM - STROKE_HARD_MIN_MM;
    return STROKE_HARD_MIN_MM + (span * ((float)pct / 100.0f));
}

float MotionManager::applySoftZones(float pos_mm, float effort) const {
    // Only scale effort if it drives further INTO the soft boundary.
    // Min boundary (near 10mm)
    if (effort < 0.0f && pos_mm < STROKE_SOFT_MIN_END_MM) {
        float scale = (pos_mm - STROKE_HARD_MIN_MM) /
                      (STROKE_SOFT_MIN_END_MM - STROKE_HARD_MIN_MM);
        scale = clampf(scale, 0.0f, 1.0f);
        effort *= scale;
    }

    // Max boundary (near 140mm)
    if (effort > 0.0f && pos_mm > STROKE_SOFT_MAX_START_MM) {
        float scale = (STROKE_HARD_MAX_MM - pos_mm) /
                      (STROKE_HARD_MAX_MM - STROKE_SOFT_MAX_START_MM);
        scale = clampf(scale, 0.0f, 1.0f);
        effort *= scale;
    }

    return effort;
}

float MotionManager::applyVelocityLimit(float vel_mm_s, float effort) const {
    const float vmax = V_MAX_MM_S;
    if (vmax <= 0.0f) return effort;

    if (vel_mm_s > vmax && effort > 0.0f) return 0.0f;
    if (vel_mm_s < -vmax && effort < 0.0f) return 0.0f;
    return effort;
}

void MotionManager::setHardFault(HardFault hf, FirstHardFaultIndex idx) {
    if ((st_fault_.hard_faults & (uint16_t)hf) == 0) {
        // latch "first hard fault" if not set yet
        if (st_fault_.first_hard == FirstHardFaultIndex::NONE) {
            st_fault_.first_hard = idx;
        }
    }
    st_fault_.hard_faults |= (uint16_t)hf;
}

void MotionManager::enterFault() {
    state_ = VbdState::FAULT;
    motor_cmd_.pwm = 0;
    motor_cmd_.slp_enable = false; // disable driver
    relaxed_hold_ = false;
}

void MotionManager::updateStatus(const MotionInputs& in, float effort_final) {
    // Build STATUS_CONTROL (0.1mm units)
    st_control_.pos_0p1mm = (uint16_t)clampf(in.pos_est_mm * 10.0f, 0.0f, 65535.0f);
    st_control_.tof_0p1mm = 0; // ToF value will be provided by tof layer; keep 0 for now.
    st_control_.flags = 0;
    if (in.prox_active) st_control_.flags |= StatusControlFlags::PROX;
    if (in.leak)        st_control_.flags |= StatusControlFlags::LEAK;
    if (in.driver_flt)  st_control_.flags |= StatusControlFlags::DRIVER_FLT;
    if (homed_)         st_control_.flags |= StatusControlFlags::HOMED;

    const bool moving = (fabsf(in.vel_est_mm_s) > 0.5f) && (motor_cmd_.pwm > 0);
    if (moving)         st_control_.flags |= StatusControlFlags::MOVING;

    if (motor_cmd_.slp_enable) st_control_.flags |= StatusControlFlags::MOTOR_EN;
    if (motor_cmd_.dir_extend) st_control_.flags |= StatusControlFlags::DIR;
    if (in.tof_valid)          st_control_.flags |= StatusControlFlags::TOF_VALID;

    st_control_.pwm = motor_cmd_.pwm;
    st_control_.seq = seq_tx_++;

    // STATUS_FAULT
    st_fault_.state = state_;
    st_fault_.seq = st_control_.seq;
}

void MotionManager::tick(const MotionInputs& in) {
    // Update hard-fault sources (always checked)
    if (in.leak)       setHardFault(HF_LEAK, FirstHardFaultIndex::LEAK);
    if (in.driver_flt) setHardFault(HF_DRIVER, FirstHardFaultIndex::DRIVER);

    // Hard limit check (position beyond usable envelope)
    if (in.pos_est_mm < STROKE_HARD_MIN_MM || in.pos_est_mm > STROKE_HARD_MAX_MM) {
        setHardFault(HF_POS_LIM, FirstHardFaultIndex::POS_LIM);
    }

    if (hasHardFault()) {
        enterFault();
        updateStatus(in, 0.0f);
        return;
    }

    // Handle motor enable from command
    if (!cmd_.motor_enable) {
        // Driver disabled by command: safe idle state (no fault)
        motor_cmd_.slp_enable = false;
        motor_cmd_.pwm = 0;
        state_ = homed_ ? VbdState::RUN : VbdState::BOOT; // simple; can refine later
        updateStatus(in, 0.0f);
        return;
    }

    motor_cmd_.slp_enable = true;

    // ENFORCE: motor cannot be enabled unless homed or actively homing
    if (!homed_ && !cmd_.request_homing) {
        setHardFault(HF_NOT_HOMED, FirstHardFaultIndex::NOT_HOMED);
        enterFault();
        updateStatus(in, 0.0f);
        return;
    }

    // State machine
    if (cmd_.request_homing && !homed_) {
        state_ = VbdState::HOMING;
    } else if (state_ == VbdState::BOOT) {
        // If enabled and not homed, you can choose to auto-home later.
        // For now: stay in BOOT until homed is requested.
        state_ = homed_ ? VbdState::RUN : VbdState::BOOT;
    }

    float effort = 0.0f;

    if (state_ == VbdState::HOMING) {
        homing_time_s_ += CONTROL_DT_S;

        // Drive gently toward min boundary until prox triggers
        effort = HOMING_EFFORT; // negative retract assumed

        // If prox triggers, declare homed and reset PID
        if (in.prox_active) {
            homed_ = true;
            state_ = VbdState::RUN;
            homing_time_s_ = 0.0f;
            pid_.reset(in.pos_est_mm);
        } else if (homing_time_s_ > HOMING_TIMEOUT_S) {
            setHardFault(HF_NOT_HOMED, FirstHardFaultIndex::NOT_HOMED);
            enterFault();
            updateStatus(in, 0.0f);
            return;
        }

        // Shape and limit even during homing
        effort = applySoftZones(in.pos_est_mm, effort);
        effort = applyVelocityLimit(in.vel_est_mm_s, effort);
    }
    else if (state_ == VbdState::RUN) {
        if (!homed_) {
            // Motor enabled without successful homing is a hard fault
            setHardFault(HF_NOT_HOMED, FirstHardFaultIndex::NOT_HOMED);
            enterFault();
            updateStatus(in, 0.0f);
            return;
        } else {
            const float target_mm = clampf(pctToMm(cmd_.target_pct), STROKE_HARD_MIN_MM, STROKE_HARD_MAX_MM);
            const float err_mm = target_mm - in.pos_est_mm;

            // Relaxed hold logic
            if (!relaxed_hold_) {
                if (fabsf(err_mm) < HOLD_ENTER_ERR_MM) {
                    in_band_time_s_ += CONTROL_DT_S;
                    if (in_band_time_s_ >= HOLD_ENTER_TIME_S) {
                        relaxed_hold_ = true;
                    }
                } else {
                    in_band_time_s_ = 0.0f;
                }
            } else {
                // In relaxed hold, only re-engage if drift is large
                if (fabsf(err_mm) > HOLD_EXIT_DRIFT_MM) {
                    relaxed_hold_ = false;
                    in_band_time_s_ = 0.0f;
                    pid_.reset(in.pos_est_mm); // avoid derivative spike
                }
            }

            if (relaxed_hold_) {
                effort = 0.0f;
            } else {
                effort = pid_.update(target_mm, in.pos_est_mm);
            }

            // Apply shaping
            effort = applySoftZones(in.pos_est_mm, effort);
            effort = applyVelocityLimit(in.vel_est_mm_s, effort);
        }
    }

    // Convert effort -> motor command
    // effort sign sets direction, magnitude sets PWM
    if (effort >= 0.0f) {
        motor_cmd_.dir_extend = true;
    } else {
        motor_cmd_.dir_extend = false;
        effort = -effort;
    }

    effort = clampf(effort, 0.0f, 1.0f);
    motor_cmd_.pwm = (uint8_t)clampf(effort * 255.0f, 0.0f, 255.0f);

    updateStatus(in, effort);
}