#pragma once
#include <stdint.h>
#include "types.h"

// This header defines:
// - CAN IDs per node
// - Packing/unpacking helpers to/from 8-byte payloads
//
// It intentionally does NOT depend on any specific CAN library
// (FlexCAN_T4 / ACAN / etc). That keeps it portable and testable.

namespace CanProtocol {

static constexpr uint16_t ID_STATUS_FAULT_BASE   = 0x050;
static constexpr uint16_t ID_CMD_SETPOINT_BASE   = 0x100;
static constexpr uint16_t ID_STATUS_CONTROL_BASE = 0x200;
static constexpr uint16_t ID_STATUS_BMS_BASE     = 0x220;

// Helpers to compute 11-bit IDs
inline uint16_t idStatusFault(NodeId node)   { return (uint16_t)(ID_STATUS_FAULT_BASE   + (uint8_t)node); }
inline uint16_t idCmdSetpoint(NodeId node)   { return (uint16_t)(ID_CMD_SETPOINT_BASE   + (uint8_t)node); }
inline uint16_t idStatusControl(NodeId node) { return (uint16_t)(ID_STATUS_CONTROL_BASE + (uint8_t)node); }
inline uint16_t idStatusBms(NodeId node)     { return (uint16_t)(ID_STATUS_BMS_BASE     + (uint8_t)node); }

// Pack/unpack return true on success (e.g., payload length is correct).
// Payload is always 8 bytes.

void packCmdSetpoint(const CmdSetpoint& cmd, uint8_t out[8]);
bool unpackCmdSetpoint(const uint8_t in[8], CmdSetpoint& cmd_out);

void packStatusFault(const StatusFault& st, uint8_t out[8]);
bool unpackStatusFault(const uint8_t in[8], StatusFault& st_out);

void packStatusControl(const StatusControl& st, uint8_t out[8]);
bool unpackStatusControl(const uint8_t in[8], StatusControl& st_out);

void packStatusBms(const StatusBms& st, uint8_t out[8]);
bool unpackStatusBms(const uint8_t in[8], StatusBms& st_out);

} // namespace CanProtocol