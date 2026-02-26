// g++ -std=c++17 -O0 -g test/test_can_protocol.cpp src/can_protocol.cpp -Iinclude -o test_can.exe
// ./test_can.exe
// "All CAN protocol tests passed."

#include <cassert>
#include <cstdint>
#include <cstdio>

#include "types.h"
#include "can_protocol.h"

using namespace CanProtocol;

static void test_cmd_setpoint()
{
    CmdSetpoint tx{};
    tx.target_pct = 55;
    tx.motor_enable = true;
    tx.request_homing = true;
    tx.seq = 12;

    uint8_t frame[8]{};
    packCmdSetpoint(tx, frame);

    // Byte checks
    assert(frame[0] == 55);
    assert((frame[3] & (1u << 0)) != 0); // motor_enable
    assert((frame[3] & (1u << 1)) != 0); // request_homing
    assert(frame[6] == 12);

    CmdSetpoint rx{};
    assert(unpackCmdSetpoint(frame, rx));

    assert(rx.target_pct == tx.target_pct);
    assert(rx.motor_enable == tx.motor_enable);
    assert(rx.request_homing == tx.request_homing);
    assert(rx.seq == tx.seq);
}

static void test_status_fault()
{
    StatusFault tx{};
    tx.hard_faults = (uint16_t)(HF_LEAK | HF_POS_LIM | HF_BMS_SWITCH);
    tx.soft_faults = (uint8_t)(SF_CMD_TO | SF_TOF_OOR);
    tx.first_hard  = FirstHardFaultIndex::LEAK;
    tx.state       = VbdState::FAULT;
    tx.seq         = 77;

    uint8_t frame[8]{};
    packStatusFault(tx, frame);

    // Byte placement checks
    assert(frame[2] == tx.soft_faults);
    assert(frame[4] == (uint8_t)tx.first_hard);
    assert(frame[5] == (uint8_t)tx.state);
    assert(frame[6] == tx.seq);

    StatusFault rx{};
    assert(unpackStatusFault(frame, rx));

    assert(rx.hard_faults == tx.hard_faults);
    assert(rx.soft_faults == tx.soft_faults);
    assert((uint8_t)rx.first_hard == (uint8_t)tx.first_hard);
    assert((uint8_t)rx.state == (uint8_t)tx.state);
    assert(rx.seq == tx.seq);
}

static void test_status_control()
{
    StatusControl tx{};
    tx.pos_0p1mm = 1500;   // 150.0 mm
    tx.tof_0p1mm = 1750;   // allowed to exceed limit
    tx.flags = (uint8_t)(
        StatusControlFlags::HOMED |
        StatusControlFlags::MOTOR_EN |
        StatusControlFlags::DIR |
        StatusControlFlags::TOF_VALID
    );
    tx.pwm = 128;
    tx.seq = 3;

    uint8_t frame[8]{};
    packStatusControl(tx, frame);

    assert(frame[4] == tx.flags);
    assert(frame[5] == tx.pwm);
    assert(frame[6] == tx.seq);

    StatusControl rx{};
    assert(unpackStatusControl(frame, rx));

    assert(rx.pos_0p1mm == tx.pos_0p1mm);
    assert(rx.tof_0p1mm == tx.tof_0p1mm);
    assert(rx.flags == tx.flags);
    assert(rx.pwm == tx.pwm);
    assert(rx.seq == tx.seq);
}

static void test_status_bms()
{
    StatusBms tx{};
    tx.pack_mV = 24000;
    tx.pack_temp_0p1C = -123;  // -12.3 C
    tx.bms_flags = (uint8_t)(
        StatusBmsFlags::UNDER_VOLTAGE_CUTOFF |
        StatusBmsFlags::LOAD_SWITCH_ERROR
    );
    tx.seq = 99;

    uint8_t frame[8]{};
    packStatusBms(tx, frame);

    // Byte placement checks
    assert(frame[4] == tx.bms_flags);
    assert(frame[6] == tx.seq);

    StatusBms rx{};
    assert(unpackStatusBms(frame, rx));

    assert(rx.pack_mV == tx.pack_mV);
    assert(rx.pack_temp_0p1C == tx.pack_temp_0p1C);
    assert(rx.bms_flags == tx.bms_flags);
    assert(rx.seq == tx.seq);
}

static void test_status_bms_signed_limits()
{
    StatusBms tx{};
    tx.pack_mV = 1;
    tx.bms_flags = 0x7F;
    tx.seq = 1;

    uint8_t frame[8]{};
    StatusBms rx{};

    tx.pack_temp_0p1C = 32767;
    packStatusBms(tx, frame);
    unpackStatusBms(frame, rx);
    assert(rx.pack_temp_0p1C == 32767);

    tx.pack_temp_0p1C = (int16_t)-32768;
    packStatusBms(tx, frame);
    unpackStatusBms(frame, rx);
    assert(rx.pack_temp_0p1C == (int16_t)-32768);
}

int main()
{
    test_cmd_setpoint();
    test_status_fault();
    test_status_control();
    test_status_bms();
    test_status_bms_signed_limits();

    std::puts("All CAN protocol tests passed.");
    return 0;
}