#include "encoder_interface.h"

#include <Arduino.h>
#include <QuadEncoder.h>

#include "config.h"

static inline float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

bool EncoderInterface::begin() {
    // Effective counts per revolution depends on how the library counts.
    // We keep it configurable via ENCODER_QUAD_MULT.
    const float counts_per_rev = (float)ENCODER_COUNTS_PER_REV_SPEC * (float)ENCODER_QUAD_MULT;
    if (counts_per_rev <= 0.0f) return false;

    mm_per_count_ = LEAD_SCREW_PITCH_MM / counts_per_rev;

    // Allocate encoder object using your pins:
    // Full constructor supports index/home/trigger pins:
    // QuadEncoder(uint8_t ch, uint8_t A, uint8_t B, uint8_t pullups, uint8_t index, uint8_t home, uint8_t trigger)
    // :contentReference[oaicite:2]{index=2}
    if (enc_ == nullptr) {
        enc_ = new QuadEncoder(ENCODER_HW_CHANNEL, PIN_ENC_A, PIN_ENC_B, 0, PIN_ENC_Z);
    }

    // Default init as per examples: setInitConfig() then init()
    // :contentReference[oaicite:3]{index=3}
    enc_->setInitConfig();
    enc_->init();

    // Start at zero counts (we can re-home later)
    enc_->write(0);

    reset();
    return true;
}

void EncoderInterface::reset() {
    counts_ = 0;
    prev_counts_ = 0;
    count_zero_ = 0;

    pos_mm_ = 0.0f;
    vel_mm_s_ = 0.0f;

    homed_ = false;
}

void EncoderInterface::setHomeAtMm(float home_mm) {
    // Make current position become home_mm by adjusting count_zero_.
    // pos_mm = (counts - count_zero_) * mm_per_count_
    // => count_zero_ = counts - home_mm/mm_per_count_
    if (mm_per_count_ <= 0.0f) return;

    const float target_counts_f = home_mm / mm_per_count_;
    const int32_t target_counts = (int32_t)lroundf(target_counts_f);

    // Keep the underlying encoder counts as-is, adjust our zero reference.
    count_zero_ = counts_ - target_counts;

    homed_ = true;

    // Refresh estimates immediately to avoid a 1-tick discontinuity
    pos_mm_ = (float)(counts_ - count_zero_) * mm_per_count_;
}

void EncoderInterface::update(float dt_s) {
    if (!enc_ || dt_s <= 0.0f) return;

    prev_counts_ = counts_;
    counts_ = enc_->read();

    // Position
    pos_mm_ = (float)(counts_ - count_zero_) * mm_per_count_;

    // Velocity from count delta
    const int32_t dc = counts_ - prev_counts_;
    const float vel_raw = ((float)dc * mm_per_count_) / dt_s;

    // Light filtering is often helpful; for now keep it minimal and stable.
    // You can tune alpha later or move it into config.
    const float alpha = 0.2f; // 0=no update, 1=no filtering
    vel_mm_s_ = vel_mm_s_ + alpha * (vel_raw - vel_mm_s_);
}