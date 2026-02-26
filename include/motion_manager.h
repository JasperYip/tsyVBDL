#pragma once
#include <stdint.h>
#include "types.h"
#include "pid_controller.h"
#include "config.h"

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
    bool homed() const { return homed_; }

private:
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