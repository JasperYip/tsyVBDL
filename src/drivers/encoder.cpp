#include "drivers/encoder.hpp"

EncoderDriver::EncoderDriver(const Config& cfg)
: cfg_(cfg) {}

void EncoderDriver::begin() {
    pinMode(cfg_.pin_a, INPUT_PULLUP);
    pinMode(cfg_.pin_b, INPUT_PULLUP);

    if (cfg_.pin_z != 255) {
        pinMode(cfg_.pin_z, INPUT_PULLUP);
    }

    last_state_ = readState_();
    count_ = 0;
}

inline uint8_t EncoderDriver::readState_() const {
    return (digitalReadFast(cfg_.pin_a) << 1) |
            digitalReadFast(cfg_.pin_b);
}

void EncoderDriver::update() {
    uint8_t s = readState_();

    if (s == last_state_) return;

    // Based on your observed sequence:
    // 3 → 2 → 0 → 1 → 3 (reverse direction)

    if (
        (last_state_ == 3 && s == 2) ||
        (last_state_ == 2 && s == 0) ||
        (last_state_ == 0 && s == 1) ||
        (last_state_ == 1 && s == 3)
    ) {
        count_++;
    }
    else if (
        (last_state_ == 3 && s == 1) ||
        (last_state_ == 1 && s == 0) ||
        (last_state_ == 0 && s == 2) ||
        (last_state_ == 2 && s == 3)
    ) {
        count_--;
    }

    last_state_ = s;
}

int32_t EncoderDriver::readCounts() const {
    return count_;
}

void EncoderDriver::writeCounts(int32_t value) {
    noInterrupts();
    count_ = value;
    interrupts();
}

float EncoderDriver::countsToMm_(int32_t counts) const {
    if (cfg_.counts_per_rev == 0) return 0.0f;

    const float revs = (float)counts / (float)cfg_.counts_per_rev;
    return revs * cfg_.lead_screw_pitch_mm;
}

float EncoderDriver::readPositionMm() const {
    return countsToMm_(readCounts());
}