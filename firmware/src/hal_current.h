// ACS712 motor-current sensor on an ADC1 input, read through the 10k/20k
// divider that keeps its 2.5 V-centred output inside the ESP32's range.
//
// The zero point is measured at boot with the motor stopped (D42). If it lands
// outside the expected window the nominal 1667 mV is used instead and the
// caller is told so it can print a "#WARN" line - no fault code is raised,
// because the travel timeouts still protect the motor on their own.

#ifndef DG_HAL_CURRENT_H
#define DG_HAL_CURRENT_H

#include <stdint.h>

namespace hal {

class CurrentSense {
public:
  CurrentSense();

  // Must be called with the motor stopped.
  void begin();

  // Takes a fresh ACS_SAMPLES average. Call on the 10 ms safety tick.
  void sample();

  float amps() const { return amps_; }
  uint16_t lastMv() const { return lastMv_; }
  uint16_t zeroMv() const { return zeroMv_; }

  // True when the boot measurement was outside ACS_ZERO_MIN/MAX_MV and the
  // nominal zero was substituted.
  bool zeroSubstituted() const { return zeroSubstituted_; }
  uint16_t measuredZeroMv() const { return measuredZeroMv_; }

private:
  uint16_t readAveragedMv() const;

  uint16_t zeroMv_;
  uint16_t measuredZeroMv_;
  bool zeroSubstituted_;
  uint16_t lastMv_;
  float amps_;
};

} // namespace hal

#endif // DG_HAL_CURRENT_H
