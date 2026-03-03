#pragma once
#include <stdint.h>
#include <Wire.h>

// We built a dedicated ToFSensor driver that runs on I2C1 (Wire1), 
// initializes the VL6180X, and provides a clean read() interface 
// returning a structured result (distance in mm, validity flag, 
// and status code). The driver performs a small number of internal 
// retries when a reading is invalid (status ≠ 0), and if all retries 
// fail it returns the last valid distance instead of garbage, along 
// with flags indicating the data is not fresh. This keeps the hardware 
// layer clean, isolates sensor quirks, and ensures higher-level 
// controllers receive stable, well-defined measurements.

class ToFSensor {
public:
    struct Reading {
        bool      fresh;       // true if new good reading
        bool      valid;       // true if status code == 0
        uint16_t  mm;          // measured range (only when valid)
        uint8_t   status;      // raw sensor status
    };

    bool begin(TwoWire& wire = Wire1);
    Reading read();           // single sample with internal retries

private:
    bool inited_ = false;
    uint16_t last_valid_mm_ = 0;  // stash last good distance
};