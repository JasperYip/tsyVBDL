#include "fake_quadencoder.h"
#include "encoder_interface.h"

#include <cmath>
#include <cstdint>

void EncoderInterface::reset() {
    counts_ = 0;
    prev_counts_ = 0;
    count_zero_ = 0;
    pos_mm_ = 0.0f;
    vel_mm_s_ = 0.0f;
    homed_ = false;
}

void EncoderInterface::setHomeAtMm(float home_mm) {
    if (mm_per_count_ <= 0.0f) return;

    const float target_counts_f = home_mm / mm_per_count_;
    const int32_t target_counts = (int32_t)lroundf(target_counts_f);

    count_zero_ = counts_ - target_counts;
    homed_ = true;

    pos_mm_ = (float)(counts_ - count_zero_) * mm_per_count_;
}

void EncoderInterface::update(float dt_s) {
    if (!enc_ || dt_s <= 0.0f) return;

    prev_counts_ = counts_;
    counts_ = enc_->read();

    pos_mm_ = (float)(counts_ - count_zero_) * mm_per_count_;

    const int32_t dc = counts_ - prev_counts_;
    const float vel_raw = ((float)dc * mm_per_count_) / dt_s;

    const float alpha = 0.2f;
    vel_mm_s_ = vel_mm_s_ + alpha * (vel_raw - vel_mm_s_);
}