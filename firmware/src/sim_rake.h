// Simulated rake, compiled only into the Wokwi build (D44).
//
// Wokwi has no winch and no limit switches that move on their own, so this
// integrates the motor command into a rake position and reports when that
// position reaches either end. main.cpp ORs these with the real HOME/END
// buttons, so a human can still force either switch by hand.
//
// Not compiled for the esp32 environment: on hardware the switches are real.

#ifndef DG_SIM_RAKE_H
#define DG_SIM_RAKE_H

#ifdef DG_WOKWI

#include <stdint.h>

#include "dg_motion.h"

namespace sim {

class Rake {
public:
  // A full stroke takes 3 s, and the rake starts halfway down so the boot
  // homing sweep is visible instead of finishing instantly.
  static constexpr uint32_t kFullStrokeMs = 3000;

  Rake();

  // Advances the position by dtMs under the command the motor is being given.
  void update(dg::MotorCmd cmd, uint32_t dtMs);

  bool homePressed() const;
  bool endPressed() const;
  float posFrac() const { return posFrac_; } // 0 = home, 1 = end of travel

private:
  float posFrac_;
};

} // namespace sim

#endif // DG_WOKWI

#endif // DG_SIM_RAKE_H
