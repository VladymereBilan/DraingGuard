// Test harnesses: drive the dg_core logic against the Plant in 10 ms steps and
// assert the motor safety invariant on every single tick.
//
// The invariant has two halves. "Never two directions at once" is enforced by
// the type system - dg::MotorCmd is one enum, so it cannot express both - which
// leaves the timing half to check here: at least deadtimeMs of commanded STOP
// must separate any two opposite direction commands.
//
//   MotionHarness   supervisor only, caller drives jobs directly (S2a)
//   ControlHarness  controller + supervisor + button, 200 ms / 10 ms (S2b)

#ifndef DG_TEST_HARNESS_H
#define DG_TEST_HARNESS_H

#include <stdint.h>

#include "dg_controller.h"
#include "dg_motion.h"
#include "plant.h"

namespace dgtest {

// Watches a stream of motor commands and asserts the dead-time rule.
class DeadtimeInvariant {
public:
  explicit DeadtimeInvariant(uint32_t deadtimeMs);

  void observe(dg::MotorCmd cmd); // asserts on violation

  uint32_t reversals() const { return reversals_; }
  uint32_t minReversalStopMs() const { return minReversalStopMs_; }
  bool sawLower() const { return sawLower_; }
  bool sawRaise() const { return sawRaise_; }
  void clear();

private:
  uint32_t deadtimeMs_;
  dg::MotorCmd lastDir_;
  uint32_t stopRunMs_;
  uint32_t reversals_;
  uint32_t minReversalStopMs_;
  bool sawLower_;
  bool sawRaise_;
};

struct MotionObs {
  dg::MotionStatus status;
  uint8_t retriesUsed;
  uint32_t elapsedMs;
  bool terminal;
};

class MotionHarness {
public:
  MotionHarness(dg::MotionSupervisor& sup, Plant& plant, uint32_t deadtimeMs);

  dg::MotionOutput step();

  void request(dg::Job job);
  MotionObs runJob(dg::Job job, uint32_t budgetMs);
  MotionObs runUntilTerminal(uint32_t budgetMs);
  MotionObs runFor(uint32_t ms);

  const dg::MotionOutput& last() const { return last_; }
  bool sawLower() const { return inv_.sawLower(); }
  bool sawRaise() const { return inv_.sawRaise(); }
  uint32_t reversals() const { return inv_.reversals(); }
  uint32_t minReversalStopMs() const { return inv_.minReversalStopMs(); }
  uint32_t ticks() const { return ticks_; }
  void clearObservations();

private:
  MotionObs observe(uint32_t startMs) const;

  dg::MotionSupervisor& sup_;
  Plant& plant_;
  DeadtimeInvariant inv_;
  uint32_t ticks_;
  dg::MotionOutput last_;
};

// Full control loop: Controller at 200 ms on top of MotionSupervisor at 10 ms.
class ControlHarness {
public:
  static constexpr uint32_t kStepsPerControllerTick = 20; // 200 ms / 10 ms
  static constexpr uint16_t kRecordCapacity = 64;

  ControlHarness(const dg::MotionParams& mp, const dg::ControllerParams& cp,
                 const PlantParams& pp, uint32_t deadtimeMs);

  void begin(bool savedBaselineValid, float savedBaselineCm,
             uint16_t savedCyclesSinceEmptied);

  void step();           // one 10 ms step
  void controllerTick(); // advance to and through the next controller tick
  void runMs(uint32_t ms);
  bool runUntilState(dg::State s, uint32_t budgetMs);
  bool runUntilRecord(uint32_t budgetMs);        // stops on the emitting step
  bool runUntilEvent(uint8_t event, uint32_t budgetMs);
  bool runUntilSince(uint32_t ms, uint32_t budgetMs); // msSinceLastRecord >= ms

  // Inputs under test control
  void setSensorsValid(bool u, bool d);
  void pressShort() { shortLatch_ = true; } // latched edge for the next tick
  void pressHold() { holdLatch_ = true; }

  Plant& plant() { return plant_; }
  dg::Controller& ctrl() { return ctrl_; }
  dg::MotionSupervisor& sup() { return sup_; }

  // Observations
  const dg::ControllerOutput& out() const { return lastOut_; }
  bool justRecorded() const { return justRecorded_; }
  bool controllerTicked() const { return controllerTicked_; }
  // The nowMs the controller saw on its most recent tick.
  uint32_t lastControllerMs() const { return lastControllerMs_; }
  const dg::Record& lastRecord() const { return lastRecord_; }
  uint16_t recordCount() const { return recordCount_; }
  const dg::Record& recordAt(uint16_t i) const { return records_[i]; }
  uint16_t recordsStored() const { return recordsStored_; }
  void clearRecords();
  uint32_t msSinceLastRecord() const;
  dg::MotorCmd lastCmd() const { return lastMotion_.cmd; }
  uint32_t nowMs() const { return plant_.nowMs(); }

private:
  dg::MotionSupervisor sup_;
  dg::Controller ctrl_;
  Plant plant_;
  DeadtimeInvariant inv_;

  uint32_t stepIndex_;
  bool uValid_;
  bool dValid_;
  bool shortLatch_;
  bool holdLatch_;

  dg::MotionOutput lastMotion_;
  dg::ControllerOutput lastOut_;
  bool controllerTicked_;
  uint32_t lastControllerMs_;
  bool justRecorded_;
  dg::Record lastRecord_;
  uint32_t lastRecordAtMs_;
  uint16_t recordCount_;

  dg::Record records_[kRecordCapacity];
  uint16_t recordsStored_;
};

} // namespace dgtest

#endif // DG_TEST_HARNESS_H
