#include "hal_current.h"

#include <Arduino.h>

namespace hal {
namespace {

// Nominal zero: the ACS712 idles at 2.5 V, which the 10k/20k divider presents
// as 2500 * 20/30 mV. Used when the boot measurement is not believable (D42).
constexpr uint16_t kNominalZeroMv = 1667;

// Millivolts at the ADC pin per amp through the sensor.
constexpr float kMvPerAmp = 1000.0f * cfg::ACS_SENS_V_PER_A;

// 12-bit ADC against the 3.3 V rail, for the uncalibrated conversion.
constexpr uint16_t kAdcFullScale = 4095;
constexpr uint32_t kAdcRefMv = 3300;

} // namespace

uint16_t CurrentSense::rawReading() {
  return static_cast<uint16_t>(analogRead(cfg::ACS712));
}

uint16_t CurrentSense::calibratedMv() {
  return static_cast<uint16_t>(analogReadMilliVolts(cfg::ACS712));
}

uint16_t CurrentSense::scaledMv(uint16_t raw) {
  return static_cast<uint16_t>((static_cast<uint32_t>(raw) * kAdcRefMv) /
                               kAdcFullScale);
}

uint16_t CurrentSense::readMv() {
#ifdef DG_WOKWI
  // Wokwi has no eFuse ADC calibration data, so analogReadMilliVolts() returns
  // the raw-zero value (~142 mV) no matter what the pin sees. analogRead()
  // tracks the input correctly, so scale that instead (D59). Real hardware
  // keeps the calibrated path, which corrects the ADC's known non-linearity.
  return scaledMv(rawReading());
#else
  return calibratedMv();
#endif
}

CurrentSense::CurrentSense()
    : window_{},
      head_(0),
      filled_(0),
      sum_(0),
      zeroMv_(kNominalZeroMv),
      measuredZeroMv_(0),
      zeroSubstituted_(false),
      lastMv_(kNominalZeroMv),
      amps_(0.0f) {}

void CurrentSense::pushSample(uint16_t mv) {
  // Unwritten slots hold zero, so subtracting before overwriting keeps sum_
  // consistent whether or not the window has filled yet.
  sum_ -= window_[head_];
  window_[head_] = mv;
  sum_ += mv;
  head_ = static_cast<uint16_t>((head_ + 1) % kWindow);
  if (filled_ < kWindow) {
    ++filled_;
  }
}

void CurrentSense::recompute() {
  if (filled_ == 0) {
    return;
  }
  lastMv_ = static_cast<uint16_t>(sum_ / filled_);
  amps_ = (static_cast<float>(lastMv_) - static_cast<float>(zeroMv_)) /
          kMvPerAmp;
}

void CurrentSense::begin() {
  // ADC1 channel, input only. 11 dB attenuation covers the full 0-3.3 V span,
  // and 12-bit resolution is what the uncalibrated scaling assumes.
  analogReadResolution(12);
  analogSetPinAttenuation(cfg::ACS712, ADC_11db);

  head_ = 0;
  filled_ = 0;
  sum_ = 0;
  for (uint16_t i = 0; i < kWindow; ++i) {
    window_[i] = 0;
  }

  // One full burst, so the zero is trustworthy and the window starts primed.
  for (uint16_t i = 0; i < kWindow; ++i) {
    pushSample(readMv());
  }

  measuredZeroMv_ = static_cast<uint16_t>(sum_ / kWindow);
  if (measuredZeroMv_ < cfg::ACS_ZERO_MIN_MV ||
      measuredZeroMv_ > cfg::ACS_ZERO_MAX_MV) {
    zeroMv_ = kNominalZeroMv;
    zeroSubstituted_ = true;
  } else {
    zeroMv_ = measuredZeroMv_;
    zeroSubstituted_ = false;
  }
  recompute();
}

void CurrentSense::sample() {
  for (uint16_t i = 0; i < kPerTick; ++i) {
    pushSample(readMv());
  }
  recompute();
}

} // namespace hal
