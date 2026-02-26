#pragma once
#include <stdint.h>

// Minimal “drop-in” API used by EncoderInterface.
class QuadEncoder {
public:
    QuadEncoder() = default;

    void setInitConfig() {}
    void init() {}

    void write(int32_t v) { counts_ = v; }
    int32_t read() const { return counts_; }

    // Test helper
    void setCounts(int32_t v) { counts_ = v; }

private:
    int32_t counts_ = 0;
};