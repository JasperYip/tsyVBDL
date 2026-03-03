#include "drivers/encoder.hpp"
#include <QuadEncoder.h>

static QuadEncoder g_enc;  // single encoder instance for now

EncoderDriver::EncoderDriver(const Config& cfg) : cfg_(cfg) {}

void EncoderDriver::begin() {
  // QuadEncoder constructor signature:
  // QuadEncoder(encoder_ch, PhaseA_pin, PhaseB_pin, pin_pus=0, index_pin=255, home_pin=255, trigger_pin=255)
  // :contentReference[oaicite:1]{index=1}
  g_enc = QuadEncoder(cfg_.qdec_channel, cfg_.pin_a, cfg_.pin_b, 0, cfg_.pin_z);

  // Library init sets up the hardware decoder
  g_enc.init();
}

int32_t EncoderDriver::readCounts() const {
  return g_enc.read();
}

void EncoderDriver::writeCounts(int32_t counts) {
  g_enc.write(static_cast<uint32_t>(counts));
}

float EncoderDriver::countsToMm(int32_t counts) const {
  if (cfg_.counts_per_rev == 0) return 0.0f;
  const float revs = static_cast<float>(counts) / static_cast<float>(cfg_.counts_per_rev);
  return revs * cfg_.lead_screw_pitch_mm;
}

float EncoderDriver::readPositionMm() const {
  return countsToMm(readCounts());
}