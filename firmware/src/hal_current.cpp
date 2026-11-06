#include "hal_current.h"

#include <Arduino.h>

#include "config.h"

namespace hal {
namespace {

// Nominal zero: the ACS712 idles at 2.5 V, which the 10k/20k divider presents
// as 2500 * 20/30 mV. Used when the boot measurement is not believable (D42).
constexpr uint16_t kNominalZeroMv = 1667;

// Millivolts at the ADC pin per amp through the sensor.
constexpr float kMvPerAmp = 1000.0f * cfg::ACS_SENS_V_PER_A;

} // namespace

CurrentSense::CurrentSense()
    : zeroMv_(kNominalZeroMv),
      measuredZeroMv_(0),
      zeroSubstituted_(false),
      lastMv_(kNominalZeroMv),
      amps_(0.0f) {}

uint16_t CurrentSense::readAveragedMv() const {
  uint32_t sum = 0;
  for (uint16_t i = 0; i < cfg::ACS_SAMPLES; ++i) {
    sum += analogReadMilliVolts(cfg::ACS712);
  }
  return static_cast<uint16_t>(sum / cfg::ACS_SAMPLES);
}

void CurrentSense::begin() {
  // ADC1 channel, input only. 11 dB attenuation covers the full 0-3.3 V span.
  analogSetPinAttenuation(cfg::ACS712, ADC_11db);

  measuredZeroMv_ = readAveragedMv();
  if (measuredZeroMv_ < cfg::ACS_ZERO_MIN_MV ||
      measuredZeroMv_ > cfg::ACS_ZERO_MAX_MV) {
    zeroMv_ = kNominalZeroMv;
    zeroSubstituted_ = true;
  } else {
    zeroMv_ = measuredZeroMv_;
    zeroSubstituted_ = false;
  }
  lastMv_ = zeroMv_;
  amps_ = 0.0f;
}

void CurrentSense::sample() {
  lastMv_ = readAveragedMv();
  amps_ = (static_cast<float>(lastMv_) - static_cast<float>(zeroMv_)) /
          kMvPerAmp;
}

} // namespace hal
