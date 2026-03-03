#include "drivers/encoder.hpp"

EncoderDriver::EncoderDriver(const Config& cfg)
: cfg_(cfg),
  // Construct QuadEncoder directly (no assignment!)
  enc_(cfg_.qdec_channel, cfg_.pin_a, cfg_.pin_b, 0, cfg_.pin_z) {}

void EncoderDriver::begin() {
  enc_.init();
}

int32_t EncoderDriver::readCounts() const {
  return enc_.read();
}

void EncoderDriver::writeCounts(int32_t counts) {
  enc_.write((uint32_t)counts);
}

float EncoderDriver::countsToMm_(int32_t counts) const {
  if (cfg_.counts_per_rev == 0) return 0.0f;
  const float revs = (float)counts / (float)cfg_.counts_per_rev;
  return revs * cfg_.lead_screw_pitch_mm;
}

float EncoderDriver::readPositionMm() const {
  return countsToMm_(readCounts());
}