// Test-only plant model: a simulated rake, motor current and water column.
//
// Modelled on the v0.2 Wokwi sketch (D18). Advanced in 10 ms steps so the tests
// drive MotionSupervisor at exactly the cadence the firmware will.
//
// This file is test code - it never ships and nothing in lib/ or src/ sees it.

#ifndef DG_TEST_PLANT_H
#define DG_TEST_PLANT_H

#include <stdint.h>

#include "dg_motion.h"

namespace dgtest {

struct PlantParams {
  uint32_t travelMsDown; // full stroke, home -> end
  uint32_t travelMsUp;   // full stroke, end -> home
  float iIdleA;          // motor stopped
  float iLowerA;         // lowering: the cord goes slack, light load
  float iRaiseA;         // raising, free running
  float iJamA;           // raising into an obstruction
  uint32_t inrushMs;     // how long the start-up spike lasts
  float iInrushA;        // start-up spike level
};

class Plant {
public:
  explicit Plant(const PlantParams& params);

  // Advance the simulation 10 ms under this motor command.
  void step10ms(dg::MotorCmd cmd);

  uint32_t nowMs() const { return nowMs_; }
  bool homePressed() const;
  bool endPressed() const;
  float currentA() const;
  float posFrac() const { return posFrac_; } // 0 = home, 1 = end
  uint16_t sweepsCompleted() const { return sweeps_; }

  // Placement
  void placeAtHome() { posFrac_ = 0.0f; }
  void placeAtEnd() { posFrac_ = 1.0f; }
  void placeAt(float frac) { posFrac_ = frac; }

  // Fault injection
  void injectJamOnRaise(bool on) { jamOnRaise_ = on; }  // frozen + jam current
  void injectStallOnRaise(bool on) { stallOnRaise_ = on; } // frozen, normal current
  void injectStallOnLower(bool on) { stallOnLower_ = on; }
  void forceBothSwitches(bool on) { forceBoth_ = on; }  // wiring fault (D40)
  void setInrush(bool on) { inrush_ = on; }

  // Water column - unused by the motion tests, consumed by the S2b controller
  // tests. The differential starts blocked and drops once enough sweeps land.
  void configureWater(uint8_t sweepsToClear, float blockedDiffCm,
                      float clearDiffCm, float downstreamLevelCm);
  float diffCm() const;
  float downstreamCm() const { return downstreamLevelCm_; }
  float upstreamCm() const { return downstreamLevelCm_ + diffCm(); }
  void setDiffOverride(float diff);
  void clearDiffOverride() { diffOverride_ = false; }

private:
  PlantParams params_;

  uint32_t nowMs_;
  float posFrac_;

  dg::MotorCmd lastCmd_;
  uint32_t startMs_; // when the motor last started turning
  bool inrush_;

  bool jamOnRaise_;
  bool stallOnRaise_;
  bool stallOnLower_;
  bool forceBoth_;

  bool reachedEnd_;
  bool loweredSinceHome_;
  uint16_t sweeps_;

  uint8_t sweepsToClear_;
  float blockedDiffCm_;
  float clearDiffCm_;
  float downstreamLevelCm_;
  bool cleared_;
  bool diffOverride_;
  float diffOverrideCm_;
};

} // namespace dgtest

#endif // DG_TEST_PLANT_H
