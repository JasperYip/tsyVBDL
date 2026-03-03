#include "drivers/tof_sensor.hpp"
#include <Adafruit_VL6180X.h>

static Adafruit_VL6180X g_vl;

bool ToFSensor::begin(TwoWire& wire) {
    wire.begin();
    wire.setClock(400000);

    if (!g_vl.begin(&wire)) {
        inited_ = false;
        return false;
    }
    inited_ = true;
    return true;
}

ToFSensor::Reading ToFSensor::read() {
    Reading out {};
    if (!inited_) {
        out.fresh = false;
        out.valid = false;
        out.status = 0xFF; // no init
        out.mm = last_valid_mm_;
        return out;
    }

    const int MAX_RETRIES = 3;
    uint8_t status;
    uint8_t range;

    for (int i = 0; i < MAX_RETRIES; i++) {
        range  = g_vl.readRange();
        status = g_vl.readRangeStatus();

        // Accept only status==0
        if (status == 0) {
            out.valid = true;
            out.fresh = true;
            out.mm    = static_cast<uint16_t>(range);
            last_valid_mm_ = out.mm;
            out.status = status;
            return out;
        }
    }

    // All retries failed → return last good if exists
    out.valid = false;
    out.fresh = false;
    out.status = status;
    out.mm = last_valid_mm_;
    return out;
}