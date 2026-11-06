// Two HC-SR04 ultrasonic rangefinders.
//
// Echo pulses are timed by a CHANGE interrupt plus micros(), never by a
// blocking pulse-width read (D4). Each sensor is pinged every
// SENSOR_PING_PERIOD_MS with the pair offset
// by SENSOR_STAGGER_MS so they cannot hear each other's bursts (D21).
//
// Thin adapter: range rejection, the median and the moving average all live in
// dg::DistanceFilter. This file only moves bytes between pins and that filter.

#ifndef DG_HAL_SENSORS_H
#define DG_HAL_SENSORS_H

#include <stdint.h>

#include "dg_filters.h"

namespace hal {

class Sensors {
public:
  Sensors();

  // Attaches the echo interrupts and schedules the first two pings. Only one
  // Sensors instance may exist: the ISR state lives at file scope.
  void begin(uint32_t nowMs);

  // Call on every pass of loop(): completes finished echoes, retires timed-out
  // ones as invalid samples, and fires the next ping when it comes due.
  void loop(uint32_t nowMs);

  // Water level above the channel bed, in cm. Carries the filter's frozen last
  // value while a sensor is invalid (D41), so check valid() alongside it.
  float upstreamLevelCm() const;
  float downstreamLevelCm() const;

  bool upstreamValid() const;
  bool downstreamValid() const;

  // Filtered distances, for diagnostics.
  float upstreamDistanceCm() const;
  float downstreamDistanceCm() const;

  uint32_t timeouts() const { return timeouts_; }

private:
  static constexpr uint8_t kUp = 0;
  static constexpr uint8_t kDown = 1;

  struct Chan {
    uint8_t trigPin;
    uint8_t echoPin;
    float mountCm;
    uint32_t nextPingMs;
    uint32_t pingUs;
    bool waiting;
  };

  void trigger(uint8_t i);

  Chan ch_[2];
  dg::DistanceFilter filt_[2];
  uint32_t timeouts_;
};

} // namespace hal

#endif // DG_HAL_SENSORS_H
