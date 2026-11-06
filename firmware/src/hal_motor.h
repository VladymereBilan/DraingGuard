// L298N direction driver: turns a dg::MotorCmd into two pin levels.
//
// Two rules are enforced here, not trusted to callers:
//   - the pin being turned off is always written LOW before the other goes
//     HIGH, so there is no instant where both are driven;
//   - a runtime guard re-reads both pins and forces them LOW if it ever finds
//     them both HIGH, which would be a shoot-through on the H-bridge.
//
// Direction convention is IN1 HIGH = lower, IN2 HIGH = raise (D14).

#ifndef DG_HAL_MOTOR_H
#define DG_HAL_MOTOR_H

#include <stdint.h>

#include "dg_motion.h"

namespace hal {

class MotorDriver {
public:
  MotorDriver();

  // Drives both pins LOW. Safe to call more than once.
  void begin();

  void apply(dg::MotorCmd cmd);

  dg::MotorCmd lastCmd() const { return last_; }
  // Number of times the guard had to intervene. Must stay zero.
  uint32_t guardTrips() const { return guardTrips_; }

private:
  dg::MotorCmd last_;
  uint32_t guardTrips_;
};

} // namespace hal

#endif // DG_HAL_MOTOR_H
