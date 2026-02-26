#include "can_protocol.h"

namespace {

// Little-endian helpers (LSB first) — consistent with most embedded CAN conventions.
// Byte0 = least significant byte.
inline void put_u16_le(uint8_t* p, uint16_t v) {
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

inline uint16_t get_u16_le(const uint8_t* p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

inline void put_i16_le(uint8_t* p, int16_t v) {
    uint16_t u = (uint16_t)v;
    put_u16_le(p, u);
}

inline int16_t get_i16_le(const uint8_t* p) {
    uint16_t u = get_u16_le(p);
    return (int16_t)u;
}

} // namespace

namespace CanProtocol {

void packCmdSetpoint(const CmdSetpoint& cmd, uint8_t out[8]) {
    for (int i = 0; i < 8; i++) out[i] = 0;

    out[0] = cmd.target_pct;

    // Byte3 flags: bit0 motor enable, bit1 homing
    uint8_t flags = 0;
    if (cmd.motor_enable)   flags |= (1u << 0);
    if (cmd.request_homing) flags |= (1u << 1);
    out[3] = flags;

    out[6] = cmd.seq;
}

bool unpackCmdSetpoint(const uint8_t in[8], CmdSetpoint& cmd_out) {
    cmd_out.target_pct = in[0];
    cmd_out.motor_enable   = (in[3] & (1u << 0)) != 0;
    cmd_out.request_homing = (in[3] & (1u << 1)) != 0;
    cmd_out.seq = in[6];
    return true;
}

void packStatusFault(const StatusFault& st, uint8_t out[8]) {
    for (int i = 0; i < 8; i++) out[i] = 0;

    // Byte0-1 hard faults (LSB/MSB)
    put_u16_le(&out[0], st.hard_faults);

    // Byte2 soft faults
    out[2] = st.soft_faults;

    // Byte4 first hard fault index
    out[4] = (uint8_t)st.first_hard;

    // Byte5 state
    out[5] = (uint8_t)st.state;

    // Byte6 sequence
    out[6] = st.seq;
}

bool unpackStatusFault(const uint8_t in[8], StatusFault& st_out) {
    st_out.hard_faults = get_u16_le(&in[0]);
    st_out.soft_faults = in[2];
    st_out.first_hard  = (FirstHardFaultIndex)in[4];
    st_out.state       = (VbdState)in[5];
    st_out.seq         = in[6];
    return true;
}

void packStatusControl(const StatusControl& st, uint8_t out[8]) {
    for (int i = 0; i < 8; i++) out[i] = 0;

    put_u16_le(&out[0], st.pos_0p1mm);
    put_u16_le(&out[2], st.tof_0p1mm);

    out[4] = st.flags;
    out[5] = st.pwm;
    out[6] = st.seq;
    // out[7] reserved
}

bool unpackStatusControl(const uint8_t in[8], StatusControl& st_out) {
    st_out.pos_0p1mm = get_u16_le(&in[0]);
    st_out.tof_0p1mm = get_u16_le(&in[2]);
    st_out.flags     = in[4];
    st_out.pwm       = in[5];
    st_out.seq       = in[6];
    return true;
}

void packStatusBms(const StatusBms& st, uint8_t out[8]) {
    for (int i = 0; i < 8; i++) out[i] = 0;

    put_u16_le(&out[0], st.pack_mV);
    put_i16_le(&out[2], st.pack_temp_0p1C);

    out[4] = st.bms_flags;
    out[6] = st.seq;
}

bool unpackStatusBms(const uint8_t in[8], StatusBms& st_out) {
    st_out.pack_mV         = get_u16_le(&in[0]);
    st_out.pack_temp_0p1C  = get_i16_le(&in[2]);
    st_out.bms_flags       = in[4];
    st_out.seq             = in[6];
    return true;
}

} // namespace CanProtocol