#pragma once
#include <stdint.h>

// =============================
// Mechanical Envelope (mm) [Physical usable limits]
// =============================
constexpr float STROKE_HARD_MIN_MM = 10.0f;
constexpr float STROKE_HARD_MAX_MM = 140.0f;

// Soft approach zones
constexpr float STROKE_SOFT_MIN_END_MM   = 20.0f;   // 10–20 mm
constexpr float STROKE_SOFT_MAX_START_MM = 130.0f;  // 130–140 mm

// =============================
// Encoder / Mechanics
// =============================
constexpr float LEAD_SCREW_PITCH_MM = 4.0f;  // 4mm per revolution
constexpr uint16_t ENCODER_COUNTS_PER_REV_SPEC = 1024;
constexpr uint8_t ENCODER_QUAD_MULT = 4;     // change if library differs

// Which QuadEncoder hardware channel to use (1..4)
constexpr uint8_t ENCODER_HW_CHANNEL = 2;  // pins 2/3 example in docs

// =============================
// Control Loop Timing
// =============================
constexpr float CONTROL_DT_S = 0.001f; // 1kHz

// =============================
// Motion Limits
// =============================
constexpr float V_MAX_MM_S = 12.0f;

// =============================
// Hold Behavior
// =============================
constexpr float HOLD_ENTER_ERR_MM   = 0.5f; // within this error, start counting
constexpr float HOLD_ENTER_TIME_S   = 1.0f; // time in band before relaxing
constexpr float HOLD_EXIT_DRIFT_MM  = 2.0f; // drift threshold to re-engage

// =============================
// Homing
// =============================
constexpr float HOMING_EFFORT = -0.15f; // gentle retract toward prox
constexpr float HOMING_TIMEOUT_S = 20.0f;

// =============================
// PID Default Parameters
// =============================
constexpr float PID_KP = 0.08f;
constexpr float PID_KI = 0.0f;
constexpr float PID_KD = 0.01f;

constexpr float PID_DEADBAND_MM = 0.2f;

constexpr float PID_OUT_MIN = -1.0f;
constexpr float PID_OUT_MAX =  1.0f;

// Integrator clamp (output-units)
constexpr float PID_I_MIN = -0.3f;
constexpr float PID_I_MAX =  0.3f;

// Optional D clamp (0 disables)
constexpr float PID_D_MAX_ABS = 0.0f;

// =============================
// CAN Rates (Hz)
// =============================
constexpr uint16_t CAN_CMD_RATE_HZ = 20;
constexpr uint16_t CAN_STATUS_RATE_HZ = 50;
constexpr uint16_t CAN_BMS_RATE_HZ = 1;

// =============================
// Pin Map (Teensy 4.0)
// =============================

// CAN (Teensy CAN1)
constexpr uint8_t PIN_CAN1_RX = 0; // CAN RX1 -> SN65HVD230 R
constexpr uint8_t PIN_CAN1_TX = 1; // CAN TX1 -> SN65HVD230 D

// Encoder
constexpr uint8_t PIN_ENC_A   = 2;
constexpr uint8_t PIN_ENC_B   = 3;
constexpr uint8_t PIN_ENC_Z   = 4; // Index

// Proximity sensor
constexpr uint8_t PIN_PROX    = 5;

// BMS UART (Serial2)
constexpr uint8_t PIN_BMS_RX2 = 7; // Teensy RX2 <- BMS TX (confirm wiring)
constexpr uint8_t PIN_BMS_TX2 = 8; // Teensy TX2 -> BMS RX (confirm wiring)

// I2C (Wire1 on Teensy 4.0)
constexpr uint8_t PIN_I2C1_SCL = 16;
constexpr uint8_t PIN_I2C1_SDA = 17;

// Leak sensor
constexpr uint8_t PIN_LEAK    = 18;

// Motor driver (Pololu G2 24v13)
constexpr uint8_t PIN_MOTOR_CS   = 19; // Analog
constexpr uint8_t PIN_MOTOR_FLT  = 20; // Digital input
constexpr uint8_t PIN_MOTOR_SLP  = 21; // Digital output
constexpr uint8_t PIN_MOTOR_PWM  = 22; // PWM output
constexpr uint8_t PIN_MOTOR_DIR  = 23; // Digital output