#include <Arduino.h>

#include "config/pin_map.hpp"
#include "config/constants.hpp"

#include "drivers/motor_driver.hpp"
#include "drivers/encoder.hpp"
#include "drivers/tof_sensor.hpp"
#include "drivers/proximity.hpp"

#include "controllers/estimator.hpp"
#include "controllers/piston_controller.hpp"
#include "controllers/pid_controller.hpp"

using namespace config;

// ---------------- Hardware ----------------
MotorDriver::Config motor_cfg{
    PIN_MOTOR_PWM,
    PIN_MOTOR_DIR,
    PIN_MOTOR_SLP,
    PIN_MOTOR_FLT,
    PIN_MOTOR_CS
};
MotorDriver motor(motor_cfg);

EncoderDriver::Config enc_cfg{
    PIN_ENC_A,
    PIN_ENC_B,
    PIN_ENC_Z,
    1,
    COUNTS_PER_SCREW_REV,
    LEAD_SCREW_MM_PER_REV
};
EncoderDriver encoder(enc_cfg);
ToFSensor tof;
ProximitySensor prox(PIN_PROX);

// ---------------- Controllers ----------------
Estimator estimator;
PistonController piston;

// ---------------- Test command ----------------
bool cmd_enable = false;
bool cmd_home = false;
float cmd_target_percent = 60.0f;

// ---------------- Timing ----------------
elapsedMicros loop_timer_us;
uint32_t last_log_ms = 0;

// ---------------- Homing bookkeeping ----------------
bool homing_position_set = false;

static const char* stateToStr(PistonController::State s)
{
    switch (s) {
        case PistonController::State::UNHOMED: return "UNHOMED";
        case PistonController::State::HOMING:  return "HOMING";
        case PistonController::State::RUN:     return "RUN";
        case PistonController::State::HOLD:    return "HOLD";
        case PistonController::State::FAULT:   return "FAULT";
        default: return "UNKNOWN";
    }
}

void setup()
{
    Serial.begin(115200);
    delay(1500);

    Serial.println("\n=== VBD HARDWARE TEST: HOMING -> RUN -> 60% -> HOLD ===");

    // ---- Drivers ----
    motor.begin();
    encoder.begin();
    tof.begin();
    prox.begin();

    // ---- Estimator ----
    Estimator::Config est_cfg{};
    est_cfg.mm_per_count    = MM_PER_COUNT;
    est_cfg.tof_gain        = TOF_FUSION_GAIN;
    est_cfg.recovery_gain   = SLIP_RECOVERY_GAIN;
    est_cfg.slip_detect_mm  = SLIP_DETECT_MM;
    est_cfg.tof_min_valid_mm = TOF_MIN_VALID_MM;
    estimator.configure(est_cfg);
    estimator.reset();

    // ---- PID ----
    PIDController::Config pid_cfg{};
    pid_cfg.kp        = PID_KP;
    pid_cfg.ki        = PID_KI;
    pid_cfg.kd        = PID_KD;
    pid_cfg.i_limit   = PID_I_LIMIT;
    pid_cfg.out_limit = PID_OUT_LIMIT;

    piston.configurePID(pid_cfg);
    piston.reset();

    // Start in UNHOMED. You manually trigger homing by setting cmd_home=true below.
    cmd_enable = false;
    cmd_home = false;
    cmd_target_percent = 60.0f;

    Serial.println("Send 'h' in serial monitor to start homing.");
}

void loop()
{
    // ---------------- Manual serial control ----------------
    if (Serial.available()) {
        char c = Serial.read();

        if (c == 'h') {
            cmd_home = true;
            cmd_enable = false;
            homing_position_set = false;
            Serial.println("[CMD] home_request = 1");
        }
        else if (c == 'e') {
            cmd_enable = true;
            Serial.println("[CMD] enable = 1");
        }
        else if (c == 'd') {
            cmd_enable = false;
            Serial.println("[CMD] enable = 0");
        }
        else if (c == '6') {
            cmd_target_percent = 60.0f;
            Serial.println("[CMD] target = 60%");
        }
    }

    // ---------------- 1 kHz loop ----------------
    if (loop_timer_us < 1000) {
        return;
    }
    loop_timer_us = 0;

    const float dt = CONTROL_DT_S;

    // ---- Read hardware ----
    const int32_t enc_count = encoder.readCounts();

    bool tof_valid_hw = false;
    uint16_t tof_mm = 0xFFFF;

    {
        // Adapt to your ToF driver API
        auto reading = tof.read();
        tof_valid_hw = reading.valid;
        tof_mm = reading.mm;
    }

    const bool prox_triggered = prox.isTriggered();

    // ---- Update estimator ----
    auto est = estimator.update(enc_count, tof_valid_hw, tof_mm, dt);

    // ---- Build piston controller inputs ----
    PistonController::Inputs in{};
    in.pos_est_mm     = est.pos_est_mm;
    in.vel_est_mm_s   = est.vel_est_mm_s;
    in.prox_triggered = prox_triggered;

    in.hard_fault     = false;   // safety manager not added yet
    in.soft_fault     = false;

    in.cmd_enable     = cmd_enable;
    in.cmd_home       = cmd_home;
    in.target_percent = cmd_target_percent;

    // ---- Update piston controller ----
    auto out = piston.update(in, dt);

    // ---- Homing reference set on release after backoff ----
    // We only do this once when controller has finished homing and entered RUN.
    if (!homing_position_set &&
        out.state == PistonController::State::RUN &&
        cmd_home == true)
    {
        estimator.setPositionMm(STROKE_HARD_MIN_MM);   // 10 mm
        homing_position_set = true;
        cmd_home = false;
        cmd_enable = true;   // immediately enable closed-loop move to 60%
        Serial.println("[HOME] estimator referenced to 10.0 mm, enabling RUN to 60%");
    }

    // ---- Drive motor ----
    if (out.state == PistonController::State::FAULT || out.duty <= 0.0f) {
        motor.setDuty(0.0f);
    } else {
        motor.setDirection(
            out.dir_extend ?
            MotorDriver::Direction::EXTEND :
            MotorDriver::Direction::RETRACT
        );
        motor.setDuty(out.duty);
    }

    // ---- Logging ----
    if (millis() - last_log_ms >= 100) {
        last_log_ms = millis();

        const float error_mm = out.target_mm - est.pos_est_mm;

        Serial.print("state=");
        Serial.print(stateToStr(out.state));

        Serial.print(" pos=");
        Serial.print(est.pos_est_mm, 2);

        Serial.print(" target=");
        Serial.print(out.target_mm, 2);

        Serial.print(" err=");
        Serial.print(error_mm, 2);

        Serial.print(" duty=");
        Serial.print(out.duty, 3);

        Serial.print(" prox=");
        Serial.print(prox_triggered ? 1 : 0);

        Serial.print(" tof=");
        if (est.tof_valid) Serial.print(est.tof_mm);
        else Serial.print("INV");

        Serial.println();
    }
}