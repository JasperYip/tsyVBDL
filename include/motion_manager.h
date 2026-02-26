#pragma once
#include <stdint.h>
#include "types.h"
#include "pid_controller.h"

struct MotionConfig {
    // Physical / usable limits (mm)
    float hard_min_mm = 10.0f;
    float hard_max_mm = 140.0f;

    // Soft zones (mm)
    float soft_min_start_mm = 10.0f;
    float soft_min_end_mm   = 20.0f;
    float soft_max_start_mm = 130.0f;
    float soft_max_end_mm   = 140.0f;

    // Control tick
    float dt_s = 0.001f; // 1 kHz

    // Velocity cap (mm/s)
    float v_max_mm_s = 12.0f;

    // Relaxed hold behavior
    float hold_enter_err_mm = 0.5f;   // within this error, start counting
    float hold_enter_time_s = 1.0f;   // time in band before relaxing
    float hold_exit_drift_mm = 2.0f;  // drift threshold to re-engage

    // Homing behavior (no hardware yet, but logic)
    float homing_effort = -0.15f;     // gentle retract toward prox
    float homing_timeout_s = 20.0f;

    // Percent->mm mapping
    float stroke_min_mm = 10.0f;       // 0% maps to 0mm (conceptually)
    float stroke_max_mm = 140.0f;     // 100% maps to 150mm
};

struct MotorCommand {
    bool slp_enable = false;   // true = driver awake
    bool dir_extend = true;    // true = extend/increase position
    uint8_t pwm = 0;           // 0..255
};

// Motion manager is pure logic: it needs sensor inputs each tick.
struct MotionInputs {
    float pos_est_mm = 0.0f;        // encoder-based fused position estimate
    float vel_est_mm_s = 0.0f;      // derived from encoder delta
    bool prox_active = false;
    bool leak = false;
    bool driver_flt = false;

    bool tof_valid = false;         // for status flags only here
};

class MotionManager {
public:
    MotionManager();

    void setConfig(const MotionConfig& cfg);
    const MotionConfig& getConfig() const { return cfg_; }

    void setPidConfig(const PIDController::Config& pid_cfg);

    // Provide latest command from CAN (or sim)
    void setCommand(const CmdSetpoint& cmd);

    // Call at 1 kHz
    void tick(const MotionInputs& in);

    // Outputs/state
    MotorCommand motorCommand() const { return motor_cmd_; }
    StatusControl statusControl() const { return st_control_; }
    StatusFault statusFault() const { return st_fault_; }

    VbdState state() const { return state_; }

private:
    MotionConfig cfg_{};
    PIDController pid_{};

    CmdSetpoint cmd_{};

    VbdState state_ = VbdState::BOOT;

    MotorCommand motor_cmd_{};
    StatusControl st_control_{};
    StatusFault st_fault_{};

    // internal timers
    float in_band_time_s_ = 0.0f;
    bool relaxed_hold_ = false;

    float homing_time_s_ = 0.0f;
    bool homed_ = false;

    uint8_t seq_tx_ = 0;

    // helpers
    float pctToMm(uint8_t pct) const;
    float clampf(float v, float lo, float hi) const;
    float applySoftZones(float pos_mm, float effort) const;
    float applyVelocityLimit(float vel_mm_s, float effort) const;

    void setHardFault(HardFault hf, FirstHardFaultIndex idx);
    bool hasHardFault() const { return st_fault_.hard_faults != 0; }

    void enterFault();
    void updateStatus(const MotionInputs& in, float effort_final);
};