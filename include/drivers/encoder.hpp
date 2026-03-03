#pragma once
#include <Arduino.h>
#include <stdint.h>
#include <QuadEncoder.h>

// We implemented a clean EncoderDriver wrapper around the Teensy 
// QuadEncoder hardware peripheral, constructing the QuadEncoder 
// object safely in the initializer list to avoid startup crashes. 
// The driver exposes simple methods to read raw counts, zero the 
// counter, and convert counts to millimeters using configurable 
// scaling parameters, while keeping index (Z) support optional 
// via configuration rather than hardcoding it in the driver.

class EncoderDriver {
public:
  struct Config {
    uint8_t qdec_channel = 1;
    uint8_t pin_a;
    uint8_t pin_b;
    uint8_t pin_z = 255;   // 255 = unused

    int32_t counts_per_rev = 4096;  // placeholder
    float lead_screw_pitch_mm = 4.0f;
  };

  explicit EncoderDriver(const Config& cfg);

  void begin();
  int32_t readCounts() const;
  void writeCounts(int32_t counts);

  float readPositionMm() const;

private:
  Config cfg_;
  QuadEncoder enc_;  // <-- constructed once, safely

  float countsToMm_(int32_t counts) const;
};