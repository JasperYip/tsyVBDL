#pragma once
#include <stdint.h>

// -------------------- Node IDs --------------------
enum class NodeId : uint8_t {
    LEFT  = 1,
    RIGHT = 2
};

// -------------------- High-level states --------------------
enum class VbdState : uint8_t {
    BOOT   = 0,
    HOMING = 1,
    RUN    = 2,   // includes moving/hold modes
    FAULT  = 3
};

// -------------------- Fault bitmasks --------------------
// Hard faults: 16-bit field (Byte0 = bits 0..7, Byte1 = bits 8..15)
enum HardFault : uint16_t {
    HF_LEAK         = (1u << 0),
    HF_DRIVER       = (1u << 1),
    HF_OVERCURRENT  = (1u << 2),
    HF_STALL        = (1u << 3),
    HF_CMD_TO_L     = (1u << 4),   // 120s no command
    HF_NOT_HOMED    = (1u << 5),
    HF_POS_LIM      = (1u << 6),
    HF_BMS_TEMP     = (1u << 7),
    HF_BMS_INV_L    = (1u << 8),   // 120s no UART from BMS
    HF_BMS_OC       = (1u << 9),
    HF_BMS_SWITCH   = (1u << 10),
    // bits 11..15 reserved
};

// Soft faults: 8-bit field (Byte2)
enum SoftFault : uint8_t {
    SF_CMD_TO    = (1u << 0),  // 300ms command timeout
    SF_TOF_INV    = (1u << 1),
    SF_TOF_OOR    = (1u << 2),
    SF_SLIP       = (1u << 3),
    SF_ENC_INV    = (1u << 4),
    SF_BMS_INV    = (1u << 5),
    SF_BMS_LOW    = (1u << 6),
    SF_BMS_HIGH   = (1u << 7),
};

// First hard fault index (Byte4) uses this ordering (0..10). 255 = none.
enum class FirstHardFaultIndex : uint8_t {
    NONE           = 255,
    LEAK           = 0,
    DRIVER         = 1,
    OVERCURRENT    = 2,
    STALL          = 3,
    CMD_TO_L       = 4,
    NOT_HOMED      = 5,
    POS_LIM        = 6,
    BMS_TEMP       = 7,
    BMS_INV_L      = 8,
    BMS_OC         = 9,
    BMS_SWITCH     = 10,
};

// -------------------- Command & Status structs --------------------
struct CmdSetpoint {
    uint8_t target_pct = 0;   // 0..100
    bool motor_enable = false; // Byte3 bit0
    bool request_homing = false; // Byte3 bit1
    uint8_t seq = 0;          // Byte6
};

struct StatusControl {
    uint16_t pos_0p1mm = 0;   // Byte0-1
    uint16_t tof_0p1mm = 0;   // Byte2-3
    uint8_t flags = 0;        // Byte4
    uint8_t pwm = 0;          // Byte5
    uint8_t seq = 0;          // Byte6
};

struct StatusFault {
    uint16_t hard_faults = 0; // Byte0-1
    uint8_t soft_faults = 0;  // Byte2
    FirstHardFaultIndex first_hard = FirstHardFaultIndex::NONE; // Byte4
    VbdState state = VbdState::BOOT; // Byte5
    uint8_t seq = 0;          // Byte6
};

struct StatusBms {
    uint16_t pack_mV = 0;     // Byte0-1
    int16_t pack_temp_0p1C = 0; // Byte2-3 signed
    uint8_t bms_flags = 0;    // Byte4
    uint8_t seq = 0;          // Byte6
};

// -------------------- STATUS_CONTROL flags (Byte4) --------------------
// Define exact bit positions so Pi parsing is deterministic.
namespace StatusControlFlags {
    static constexpr uint8_t PROX       = (1u << 0);
    static constexpr uint8_t LEAK       = (1u << 1);
    static constexpr uint8_t DRIVER_FLT = (1u << 2);
    static constexpr uint8_t HOMED      = (1u << 3);
    static constexpr uint8_t MOVING     = (1u << 4);
    static constexpr uint8_t MOTOR_EN   = (1u << 5); // driver enabled (SLP high)
    static constexpr uint8_t DIR        = (1u << 6); // 1 = extend, 0 = retract
    static constexpr uint8_t TOF_VALID  = (1u << 7);
}

// -------------------- STATUS_BMS flags (Byte4) --------------------
// Byte4 is a bitmask produced by bms_interface.* from BMS UART fault IDs.
// Mapping (BMS fault ID -> bit position):
// 0x02 -> bit0  Under-Voltage Cutoff
// 0x03 -> bit1  Over-Voltage Cutoff
// 0x04 -> bit2  Over-Temperature Cutoff
// 0x05 -> bit3  Discharging Over-Current Cutoff
// 0x0A -> bit4  Low Temperature Cutoff
// 0x0C -> bit5  Load Switch Error
// 0x0D -> bit6  Single Port Switch Error
// bit7 reserved
namespace StatusBmsFlags {
    static constexpr uint8_t UNDER_VOLTAGE_CUTOFF   = (1u << 0); // 0x02
    static constexpr uint8_t OVER_VOLTAGE_CUTOFF    = (1u << 1); // 0x03
    static constexpr uint8_t OVER_TEMPERATURE_CUTOFF= (1u << 2); // 0x04
    static constexpr uint8_t DISCHARGE_OVERCURRENT  = (1u << 3); // 0x05
    static constexpr uint8_t LOW_TEMPERATURE_CUTOFF = (1u << 4); // 0x0A
    static constexpr uint8_t LOAD_SWITCH_ERROR      = (1u << 5); // 0x0C
    static constexpr uint8_t SINGLE_PORT_SWITCH_ERR = (1u << 6); // 0x0D
    // bit7 reserved
}