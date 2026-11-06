// ACS712 motor-current sensor on an ADC1 input, read through the 10k/20k
// divider that keeps its 2.5 V-centred output inside the ESP32's range.
//
// The ACS_SAMPLES average is gathered a few readings at a time into a rolling
// window rather than as one burst per tick (D57), so no single safety tick pays
// for 64 ADC conversions. The window turns over in about 80 ms, far inside the
// 200 ms a jam must persist to be believed.
//
// The zero point is measured at boot with the motor stopped (D42). If it lands
// outside the expected window the nominal 1667 mV is used instead and the
// caller is told so it can print a "#WARN" line - no fault code is raised,
// because the travel timeouts still protect the motor on their own.

#ifndef DG_HAL_CURRENT_H
#define DG_HAL_CURRENT_H

#include <stdint.h>

#include "config.h"

namespace hal {

class CurrentSense {
public:
  // Full width of the rolling average.
  static constexpr uint16_t kWindow = cfg::ACS_SAMPLES;
  // Conversions per 10 ms safety tick: kWindow / kPerTick = 8 ticks to refresh.
  static constexpr uint16_t kPerTick = 8;

  CurrentSense();

  // Must be called with the motor stopped. Takes a full kWindow burst to prime
  // the rolling window and establish the zero; blocking here is fine because
  // setup() is not the control loop.
  void begin();

  // Takes kPerTick fresh readings into the rolling window. Call on the 10 ms
  // safety tick.
  void sample();

  float amps() const { return amps_; }
  uint16_t lastMv() const { return lastMv_; }
  uint16_t zeroMv() const { return zeroMv_; }

  // True when the boot measurement was outside ACS_ZERO_MIN/MAX_MV and the
  // nominal zero was substituted.
  bool zeroSubstituted() const { return zeroSubstituted_; }
  uint16_t measuredZeroMv() const { return measuredZeroMv_; }

  // Both ADC paths for one pin, for the boot diagnostic line. Wokwi does not
  // supply the eFuse calibration analogReadMilliVolts() needs, so the two
  // disagree there and agree on real silicon (D59).
  static uint16_t rawReading();
  static uint16_t calibratedMv();
  static uint16_t scaledMv(uint16_t raw);

private:
  // The millivolt source this build uses.
  static uint16_t readMv();

  void pushSample(uint16_t mv);
  void recompute();

  uint16_t window_[kWindow];
  uint16_t head_;
  uint16_t filled_;
  uint32_t sum_;

  uint16_t zeroMv_;
  uint16_t measuredZeroMv_;
  bool zeroSubstituted_;
  uint16_t lastMv_;
  float amps_;
};

} // namespace hal

#endif // DG_HAL_CURRENT_H
