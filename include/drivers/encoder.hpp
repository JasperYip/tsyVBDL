#pragma once
#include <stdint.h>

// We created a clean EncoderDriver wrapper around the Teensy 
// QuadEncoder hardware library, isolating all quadrature 
// decoding inside a simple interface that provides raw counts 
// and a basic counts-to-millimeter conversion. The smoke test 
// initializes the hardware decoder, zeros the counter, and 
// continuously reads counts while the motor is disabled, 
// verifying correct direction, stability, and wiring without 
// any control logic involved.

class EncoderDriver {
public:
  struct Config {
    // Hardware quadrature channel: 1..4 (Teensy has 4 QDEC modules)
    uint8_t qdec_channel = 1;

    // Pins (must be supported by the library)
    uint8_t pin_a;
    uint8_t pin_b;
    uint8_t pin_z;     // optional index (can be 255 if unused)

    // Scaling (can tune later in Phase 2 constants)
    // counts_per_rev should match what QuadEncoder::read() returns per motor shaft rev.
    // (Often this is 4x the encoder "PPR". We’ll calibrate later.)
    int32_t counts_per_rev = 1;

    // Lead screw pitch in mm per revolution (motor shaft rev if direct drive)
    float lead_screw_pitch_mm = 1.0f;
  };

  explicit EncoderDriver(const Config& cfg);

  void begin();
  int32_t readCounts() const;
  void writeCounts(int32_t counts);

  // Convenience conversions
  float countsToMm(int32_t counts) const;
  float readPositionMm() const;

private:
  Config cfg_;
};