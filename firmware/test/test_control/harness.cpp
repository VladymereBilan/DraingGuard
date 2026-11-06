#include "harness.h"

#include <unity.h>

namespace dgtest {

MotionHarness::MotionHarness(dg::MotionSupervisor& sup, Plant& plant,
                             uint32_t deadtimeMs)
    : sup_(sup),
      plant_(plant),
      deadtimeMs_(deadtimeMs),
      lastDir_(dg::MotorCmd::STOP),
      stopRunMs_(0),
      reversals_(0),
      minReversalStopMs_(UINT32_MAX),
      sawLower_(false),
      sawRaise_(false),
      ticks_(0),
      last_{dg::MotorCmd::STOP, dg::MotionStatus::IDLE, 0} {}

void MotionHarness::checkInvariant(dg::MotorCmd cmd) {
  if (cmd == dg::MotorCmd::STOP) {
    stopRunMs_ += 10;
    return;
  }

  if (lastDir_ != dg::MotorCmd::STOP && cmd != lastDir_) {
    // A reversal. The STOP run that just ended must have covered the dead time.
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(deadtimeMs_, stopRunMs_);
    ++reversals_;
    if (stopRunMs_ < minReversalStopMs_) {
      minReversalStopMs_ = stopRunMs_;
    }
  }

  lastDir_ = cmd;
  stopRunMs_ = 0;
  if (cmd == dg::MotorCmd::LOWER) {
    sawLower_ = true;
  } else {
    sawRaise_ = true;
  }
}

dg::MotionOutput MotionHarness::step() {
  const dg::MotionInput in{plant_.nowMs(), plant_.homePressed(),
                           plant_.endPressed(), plant_.currentA()};
  last_ = sup_.tick(in);
  checkInvariant(last_.cmd);
  plant_.step10ms(last_.cmd);
  ++ticks_;
  return last_;
}

MotionObs MotionHarness::observe(uint32_t startMs) const {
  return MotionObs{last_.status, last_.retriesUsed, plant_.nowMs() - startMs,
                   dg::isTerminal(last_.status)};
}

void MotionHarness::request(dg::Job job) {
  sup_.requestJob(job, plant_.nowMs());
}

MotionObs MotionHarness::runJob(dg::Job job, uint32_t budgetMs) {
  request(job);
  return runUntilTerminal(budgetMs);
}

MotionObs MotionHarness::runUntilTerminal(uint32_t budgetMs) {
  const uint32_t start = plant_.nowMs();
  for (;;) {
    step();
    if (dg::isTerminal(last_.status)) {
      break;
    }
    if ((plant_.nowMs() - start) >= budgetMs) {
      break;
    }
  }
  return observe(start);
}

MotionObs MotionHarness::runFor(uint32_t ms) {
  const uint32_t start = plant_.nowMs();
  while ((plant_.nowMs() - start) < ms) {
    step();
    if (dg::isTerminal(last_.status)) {
      break;
    }
  }
  return observe(start);
}

void MotionHarness::clearObservations() {
  reversals_ = 0;
  minReversalStopMs_ = UINT32_MAX;
  sawLower_ = false;
  sawRaise_ = false;
  ticks_ = 0;
}

} // namespace dgtest
