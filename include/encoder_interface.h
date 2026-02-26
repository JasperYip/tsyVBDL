#pragma once
#include <stdint.h>

// Forward declare to avoid including heavy headers in .h
class QuadEncoder;

class EncoderInterface {
public:
    EncoderInterface() = default;

    // Initializes the hardware encoder.
    // Returns true if initialized successfully.
    bool begin();

    // Call at 1kHz (CONTROL_DT_S).
    void update(float dt_s);

    // Position estimate in mm (float).
    float posMm() const { return pos_mm_; }

    // Velocity estimate in mm/s (float).
    float velMmPerSec() const { return vel_mm_s_; }

    // Raw encoder counts (as reported by library, signed).
    int32_t rawCounts() const { return counts_; }

    // True after homing reference has been set at least once.
    bool isHomed() const { return homed_; }

    // Re-reference the encoder so that the CURRENT position becomes home_mm.
    // Example: setHomeAtMm(10.0f) when proximity triggers.
    void setHomeAtMm(float home_mm);

    // Optional: reset all internal state (does not re-init hardware).
    void reset();

#ifdef UNIT_TEST
    // Inject a fake encoder (test-only).
    void _setEncoderForTest(QuadEncoder* enc) { enc_ = enc; }
    // Optionally set conversion directly (test-only) if you don’t want config.h.
    void _setMmPerCountForTest(float v) { mm_per_count_ = v; }
#endif

private:
    QuadEncoder* enc_ = nullptr;

    // Cached counts from last update
    int32_t counts_ = 0;
    int32_t prev_counts_ = 0;

    // Conversion
    float mm_per_count_ = 0.0f;

    // Reference offset (counts)
    // Position mm = (counts - count_zero_) * mm_per_count_
    int32_t count_zero_ = 0;

    // State
    bool homed_ = false;

    // Estimates
    float pos_mm_ = 0.0f;
    float vel_mm_s_ = 0.0f;
};