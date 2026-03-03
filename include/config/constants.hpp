#pragma once
#include <stdint.h>

// =============================
// Core timing (Phase 1 defaults)
// =============================
constexpr float CONTROL_DT_S = 0.001f;      // 1 kHz control loop (used later)
constexpr float PWM_FREQ_HZ  = 25000.0f;    // 25 kHz motor PWM

// =============================
// Piston mechanical limits (mm)
// Physical usable range is 0–150 but we *crop* to 10–140
// =============================
constexpr float STROKE_MIN_MM = 10.0f;
constexpr float STROKE_MAX_MM = 140.0f;

// Soft zones (slow down near ends)
// 10–20 mm and 130–140 mm
constexpr float SOFT_MIN_END_MM   = 20.0f;
constexpr float SOFT_MAX_START_MM = 130.0f;

// Home reference
// Prox triggers when piston is at ~10 mm
constexpr float HOME_MM = 10.0f;

// =============================
// Stroke percentage mapping
// 0–100% → 10–140 mm
// =============================
constexpr float STROKE_SPAN_MM = (STROKE_MAX_MM - STROKE_MIN_MM);

// =============================
// Encoder / mechanics
// =============================
constexpr int32_t ENCODER_COUNTS_PER_REV = 1024;  // your current assumption (verify x1 vs x4)
constexpr float   LEAD_SCREW_PITCH_MM    = 4.0f;  // 4 mm per revolution

// =============================
// BMS
// =============================
constexpr uint32_t BMS_BAUDRATE = 115200;

// =============================
// Fault beacon/log rates (used later)
// =============================
constexpr float FAULT_BEACON_HZ = 2.0f;
constexpr float STATUS_CONTROL_HZ = 50.0f;
constexpr float STATUS_BMS_HZ = 1.0f;
constexpr float CMD_SETPOINT_HZ = 20.0f;