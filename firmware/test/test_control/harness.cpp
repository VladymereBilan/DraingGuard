#include "harness.h"

#include <unity.h>

namespace dgtest {

// --------------------------------------------------------------------------
// DeadtimeInvariant
// --------------------------------------------------------------------------

DeadtimeInvariant::DeadtimeInvariant(uint32_t deadtimeMs)
    : deadtimeMs_(deadtimeMs),
      lastDir_(dg::MotorCmd::STOP),
      stopRunMs_(0),
      reversals_(0),
      minReversalStopMs_(UINT32_MAX),
      sawLower_(false),
      sawRaise_(false) {}

void DeadtimeInvariant::observe(dg::MotorCmd cmd) {
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

void DeadtimeInvariant::clear() {
  reversals_ = 0;
  minReversalStopMs_ = UINT32_MAX;
  sawLower_ = false;
  sawRaise_ = false;
}

// --------------------------------------------------------------------------
// MotionHarness
// --------------------------------------------------------------------------

MotionHarness::MotionHarness(dg::MotionSupervisor& sup, Plant& plant,
                             uint32_t deadtimeMs)
    : sup_(sup),
      plant_(plant),
      inv_(deadtimeMs),
      ticks_(0),
      last_{dg::MotorCmd::STOP, dg::MotionStatus::IDLE, 0} {}

dg::MotionOutput MotionHarness::step() {
  const dg::MotionInput in{plant_.nowMs(), plant_.homePressed(),
                           plant_.endPressed(), plant_.currentA()};
  last_ = sup_.tick(in);
  inv_.observe(last_.cmd);
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
  inv_.clear();
  ticks_ = 0;
}

// --------------------------------------------------------------------------
// ControlHarness
// --------------------------------------------------------------------------

ControlHarness::ControlHarness(const dg::MotionParams& mp,
                               const dg::ControllerParams& cp,
                               const PlantParams& pp, uint32_t deadtimeMs)
    : sup_(mp),
      ctrl_(cp),
      plant_(pp),
      inv_(deadtimeMs),
      stepIndex_(0),
      uValid_(true),
      dValid_(true),
      shortLatch_(false),
      holdLatch_(false),
      lastMotion_{dg::MotorCmd::STOP, dg::MotionStatus::IDLE, 0},
      lastOut_{},
      controllerTicked_(false),
      lastControllerMs_(0),
      justRecorded_(false),
      lastRecord_{},
      lastRecordAtMs_(0),
      recordCount_(0),
      records_{},
      recordsStored_(0) {}

void ControlHarness::begin(bool savedBaselineValid, float savedBaselineCm,
                           uint16_t savedCyclesSinceEmptied) {
  ctrl_.begin(plant_.nowMs(), savedBaselineValid, savedBaselineCm,
              savedCyclesSinceEmptied);
  lastRecordAtMs_ = plant_.nowMs();
}

void ControlHarness::step() {
  justRecorded_ = false;
  controllerTicked_ = false;

  // The controller runs on every 20th fast tick. Its motion job is handed to
  // the supervisor before the supervisor ticks, so a STOP takes effect on this
  // very step rather than the next one.
  if ((stepIndex_ % kStepsPerControllerTick) == 0) {
    dg::ControllerInput cin{};
    cin.nowMs = plant_.nowMs();
    cin.upstreamCm = plant_.upstreamCm();
    cin.downstreamCm = plant_.downstreamCm();
    cin.upstreamValid = uValid_;
    cin.downstreamValid = dValid_;
    cin.buttonShortPress = shortLatch_;
    cin.buttonHeldClear = holdLatch_;
    cin.motionStatus = lastMotion_.status;
    cin.motionRetriesUsed = lastMotion_.retriesUsed;
    shortLatch_ = false;
    holdLatch_ = false;

    lastOut_ = ctrl_.tick(cin);
    controllerTicked_ = true;
    lastControllerMs_ = cin.nowMs;

    if (lastOut_.hasRecord) {
      justRecorded_ = true;
      lastRecord_ = lastOut_.record;
      lastRecordAtMs_ = plant_.nowMs();
      ++recordCount_;
      if (recordsStored_ < kRecordCapacity) {
        records_[recordsStored_++] = lastOut_.record;
      }
    }
    if (lastOut_.motionJob != dg::Job::NONE) {
      sup_.requestJob(lastOut_.motionJob, plant_.nowMs());
    }
  }

  const dg::MotionInput min{plant_.nowMs(), plant_.homePressed(),
                            plant_.endPressed(), plant_.currentA()};
  lastMotion_ = sup_.tick(min);
  inv_.observe(lastMotion_.cmd);
  plant_.step10ms(lastMotion_.cmd);
  ++stepIndex_;
}

void ControlHarness::controllerTick() {
  // Steps until the controller has actually run, whatever the current phase of
  // the 20-step cycle, and stops on that step.
  for (uint32_t i = 0; i < kStepsPerControllerTick; ++i) {
    step();
    if (controllerTicked_) {
      return;
    }
  }
}

void ControlHarness::runMs(uint32_t ms) {
  const uint32_t start = plant_.nowMs();
  while ((plant_.nowMs() - start) < ms) {
    step();
  }
}

bool ControlHarness::runUntilState(dg::State s, uint32_t budgetMs) {
  const uint32_t start = plant_.nowMs();
  while ((plant_.nowMs() - start) < budgetMs) {
    if (ctrl_.state() == s) {
      return true;
    }
    step();
  }
  return ctrl_.state() == s;
}

bool ControlHarness::runUntilRecord(uint32_t budgetMs) {
  const uint32_t start = plant_.nowMs();
  while ((plant_.nowMs() - start) < budgetMs) {
    step();
    if (justRecorded_) {
      return true;
    }
  }
  return false;
}

bool ControlHarness::runUntilEvent(uint8_t event, uint32_t budgetMs) {
  const uint32_t start = plant_.nowMs();
  while ((plant_.nowMs() - start) < budgetMs) {
    step();
    if (justRecorded_ && lastRecord_.event == event) {
      return true;
    }
  }
  return false;
}

bool ControlHarness::runUntilSince(uint32_t ms, uint32_t budgetMs) {
  const uint32_t start = plant_.nowMs();
  while ((plant_.nowMs() - start) < budgetMs) {
    if (msSinceLastRecord() >= ms) {
      return true;
    }
    step();
  }
  return msSinceLastRecord() >= ms;
}

void ControlHarness::setSensorsValid(bool u, bool d) {
  uValid_ = u;
  dValid_ = d;
}

void ControlHarness::clearRecords() {
  recordsStored_ = 0;
  recordCount_ = 0;
}

uint32_t ControlHarness::msSinceLastRecord() const {
  return plant_.nowMs() - lastRecordAtMs_;
}

} // namespace dgtest
