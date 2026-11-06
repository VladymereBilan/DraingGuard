// Test harness: drives MotionSupervisor against the Plant in 10 ms steps and
// asserts the motor safety invariant on every single tick.
//
// The invariant has two halves. "Never two directions at once" is enforced by
// the type system - dg::MotorCmd is one enum, so it cannot express both - which
// leaves the timing half to check here: at least deadtimeMs of commanded STOP
// must separate any two opposite direction commands.

#ifndef DG_TEST_HARNESS_H
#define DG_TEST_HARNESS_H

#include <stdint.h>

#include "dg_motion.h"
#include "plant.h"

namespace dgtest {

struct MotionObs {
  dg::MotionStatus status;
  uint8_t retriesUsed;
  uint32_t elapsedMs;
  bool terminal;
};

class MotionHarness {
public:
  MotionHarness(dg::MotionSupervisor& sup, Plant& plant, uint32_t deadtimeMs);

  // One 10 ms step: read the plant, tick the supervisor, check the invariant,
  // then apply the command to the plant for 10 ms.
  dg::MotionOutput step();

  void request(dg::Job job);
  MotionObs runJob(dg::Job job, uint32_t budgetMs);
  MotionObs runUntilTerminal(uint32_t budgetMs);
  MotionObs runFor(uint32_t ms); // returns early if the job finishes

  const dg::MotionOutput& last() const { return last_; }
  bool sawLower() const { return sawLower_; }
  bool sawRaise() const { return sawRaise_; }
  uint32_t reversals() const { return reversals_; }
  uint32_t minReversalStopMs() const { return minReversalStopMs_; }
  uint32_t ticks() const { return ticks_; }
  void clearObservations();

private:
  void checkInvariant(dg::MotorCmd cmd);
  MotionObs observe(uint32_t startMs) const;

  dg::MotionSupervisor& sup_;
  Plant& plant_;
  uint32_t deadtimeMs_;

  dg::MotorCmd lastDir_;      // last non-STOP command seen
  uint32_t stopRunMs_;        // length of the current run of STOP commands
  uint32_t reversals_;
  uint32_t minReversalStopMs_;
  bool sawLower_;
  bool sawRaise_;
  uint32_t ticks_;
  dg::MotionOutput last_;
};

} // namespace dgtest

#endif // DG_TEST_HARNESS_H
