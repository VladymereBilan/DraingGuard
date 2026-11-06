#include "dg_controller.h"

namespace dg {
namespace {

float absf(float x) { return x < 0.0f ? -x : x; }

} // namespace

Controller::Controller(const ControllerParams& params)
    : params_(params),
      state_(State::HOMING),
      stateEnteredMs_(0),
      faultCode_(kFaultNone),
      blockageCycles_(0),
      recoveringFromFault_(false),
      baselineCm_(0.0f),
      haveBaseline_(false),
      savedBaselineCm_(0.0f),
      savedBaselineValid_(false),
      baselineSum_(0.0f),
      baselineCount_(0),
      cyclesSinceEmptied_(0),
      sensorBad_(false),
      sensorBadSinceMs_(0),
      atBaselineTimer_(false),
      atBaselineSinceMs_(0),
      lastRecordMs_(0),
      queue_{},
      queueHead_(0),
      queueCount_(0),
      eventOverflow_(0),
      jobRequest_(Job::NONE),
      saveBaseline_(false),
      saveCycles_(false) {}

void Controller::begin(uint32_t nowMs, bool savedBaselineValid,
                       float savedBaselineCm, uint16_t savedCyclesSinceEmptied) {
  savedBaselineValid_ = savedBaselineValid;
  savedBaselineCm_ = savedBaselineCm;
  baselineCm_ = savedBaselineValid ? savedBaselineCm : 0.0f;
  haveBaseline_ = savedBaselineValid;
  cyclesSinceEmptied_ = savedCyclesSinceEmptied;

  faultCode_ = kFaultNone;
  blockageCycles_ = 0;
  recoveringFromFault_ = false;
  baselineSum_ = 0.0f;
  baselineCount_ = 0;
  sensorBad_ = false;
  sensorBadSinceMs_ = 0;
  atBaselineTimer_ = false;
  atBaselineSinceMs_ = 0;
  lastRecordMs_ = nowMs;
  queueHead_ = 0;
  queueCount_ = 0;
  eventOverflow_ = 0;
  saveBaseline_ = false;
  saveCycles_ = false;

  // Every boot homes first, so the rake position is known before anything else.
  state_ = State::HOMING;
  stateEnteredMs_ = nowMs;
  jobRequest_ = Job::HOME;
}

bool Controller::sensorsOk(const ControllerInput& in) const {
  return in.upstreamValid && in.downstreamValid;
}

bool Controller::sensorFaultDue(const ControllerInput& in) const {
  return sensorBad_ && (in.nowMs - sensorBadSinceMs_) >= params_.sensorFaultMs;
}

float Controller::diffOf(const ControllerInput& in) const {
  return in.upstreamCm - in.downstreamCm;
}

bool Controller::aboveThreshold(const ControllerInput& in) const {
  return diffOf(in) > baselineCm_ + params_.thresholdCm;
}

bool Controller::atBaseline(const ControllerInput& in) const {
  return diffOf(in) <= baselineCm_ + params_.verifyMarginCm;
}

Record Controller::buildRecord(uint8_t event, const ControllerInput& in) const {
  Record r{};
  r.ms = in.nowMs;
  r.uCm = in.upstreamCm;
  r.dCm = in.downstreamCm;
  r.diffCm = diffOf(in);
  r.state = static_cast<uint8_t>(state_);
  r.cycles = cyclesSinceEmptied_;
  r.fault = faultCode_;
  r.event = event;
  return r;
}

void Controller::queueEvent(uint8_t event, const ControllerInput& in) {
  if (queueCount_ == kEventQueueCapacity) {
    // Full: drop the oldest so the newest survives, and count the loss (D39).
    queueHead_ = static_cast<uint8_t>((queueHead_ + 1) % kEventQueueCapacity);
    --queueCount_;
    ++eventOverflow_;
  }
  const uint8_t slot =
      static_cast<uint8_t>((queueHead_ + queueCount_) % kEventQueueCapacity);
  queue_[slot] = buildRecord(event, in);
  ++queueCount_;
}

Record Controller::popEvent() {
  const Record r = queue_[queueHead_];
  queueHead_ = static_cast<uint8_t>((queueHead_ + 1) % kEventQueueCapacity);
  --queueCount_;
  return r;
}

void Controller::enterState(State s, uint32_t nowMs) {
  state_ = s;
  stateEnteredMs_ = nowMs;
  if (s == State::BASELINE) {
    baselineSum_ = 0.0f;
    baselineCount_ = 0;
  }
}

void Controller::enterFault(uint8_t code, const ControllerInput& in) {
  faultCode_ = code; // set first so the event record carries it
  enterState(State::FAULT, in.nowMs);
  jobRequest_ = Job::STOP; // entering FAULT always stops the motor
  atBaselineTimer_ = false;
  queueEvent(kEventFault, in);
}

ControllerOutput Controller::tick(const ControllerInput& in) {
  ControllerOutput out{};
  out.motionJob = Job::NONE;

  // Sensor validity is tracked controller-wide, not per state, so the 60 s
  // window survives a CONFIRMING -> MONITORING abort (D32 with D41).
  if (sensorsOk(in)) {
    sensorBad_ = false;
    sensorBadSinceMs_ = in.nowMs;
  } else if (!sensorBad_) {
    sensorBad_ = true;
    sensorBadSinceMs_ = in.nowMs;
  }

  // The bin button is an operator action and is honoured in every state (D37),
  // handled before the state machine so a press is never queued behind a
  // state event.
  if (in.buttonShortPress) {
    cyclesSinceEmptied_ = 0;
    saveCycles_ = true;
    queueEvent(kEventBinEmptied, in);
  }

  switch (state_) {
    case State::HOMING:
      // No sensor-fault check: homing does not read the sensors.
      if (in.motionStatus == MotionStatus::DONE ||
          in.motionStatus == MotionStatus::DONE_AFTER_RETRY) {
        if (recoveringFromFault_) {
          recoveringFromFault_ = false;
          faultCode_ = kFaultNone;
          blockageCycles_ = 0;
          queueEvent(kEventFaultRecovered, in);
          // D34: keep the baseline we already trust, otherwise learn one.
          enterState(haveBaseline_ ? State::MONITORING : State::BASELINE,
                     in.nowMs);
        } else {
          enterState(State::BASELINE, in.nowMs);
        }
      } else if (isTerminal(in.motionStatus)) {
        enterFault(kFaultHomingFailed, in);
      }
      break;

    case State::BASELINE:
      if (sensorFaultDue(in)) {
        enterFault(kFaultSensor, in);
        break;
      }
      if (sensorsOk(in)) {
        baselineSum_ += diffOf(in);
        ++baselineCount_;
      }
      if ((in.nowMs - stateEnteredMs_) >= params_.baselineMs) {
        if (baselineCount_ > 0) {
          const float mean =
              baselineSum_ / static_cast<float>(baselineCount_);
          if (savedBaselineValid_ &&
              absf(mean - savedBaselineCm_) > params_.baselineRejectCm) {
            // Too far from the stored value to trust: keep the saved one and
            // say so, rather than silently adopting a bad reading (D15/D31).
            baselineCm_ = savedBaselineCm_;
            haveBaseline_ = true;
            queueEvent(kEventBaselineRejected, in);
          } else {
            baselineCm_ = mean;
            savedBaselineCm_ = mean;
            savedBaselineValid_ = true;
            haveBaseline_ = true;
            saveBaseline_ = true;
          }
          enterState(State::MONITORING, in.nowMs);
        } else {
          // Nothing valid landed in the window. Start another one; the
          // sensor-fault timer keeps running and ends this at 60 s (D32).
          stateEnteredMs_ = in.nowMs;
          baselineSum_ = 0.0f;
          baselineCount_ = 0;
        }
      }
      break;

    case State::MONITORING:
      if (sensorFaultDue(in)) {
        enterFault(kFaultSensor, in);
        break;
      }
      // Never start clearing on data we do not trust (D25).
      if (sensorsOk(in) && aboveThreshold(in)) {
        enterState(State::CONFIRMING, in.nowMs);
      }
      break;

    case State::CONFIRMING:
      if (sensorFaultDue(in)) {
        enterFault(kFaultSensor, in);
        break;
      }
      if (!sensorsOk(in)) {
        enterState(State::MONITORING, in.nowMs); // abort, do not freeze (D41)
        break;
      }
      if (!aboveThreshold(in)) {
        enterState(State::MONITORING, in.nowMs); // false alarm, no event
        break;
      }
      if ((in.nowMs - stateEnteredMs_) >= params_.confirmMs) {
        blockageCycles_ = 1;
        enterState(State::CLEARING, in.nowMs);
        jobRequest_ = Job::SWEEP;
      }
      break;

    case State::CLEARING:
      // No sensor-fault check: the stroke is mechanically committed and no
      // sensor decision is pending until SETTLING.
      if (in.motionStatus == MotionStatus::DONE ||
          in.motionStatus == MotionStatus::DONE_AFTER_RETRY) {
        ++cyclesSinceEmptied_; // only completed sweeps count (D35)
        saveCycles_ = true;
        if (in.motionStatus == MotionStatus::DONE_AFTER_RETRY) {
          // Informational, not a latched fault (D36). Queued while still in
          // CLEARING so the record says where the jam happened.
          faultCode_ = kFaultJamClearedByRetry;
          queueEvent(kEventJamRetryCleared, in);
        }
        enterState(State::SETTLING, in.nowMs);
      } else if (isTerminal(in.motionStatus)) {
        enterFault(kFaultJamNotCleared, in);
      }
      break;

    case State::SETTLING:
      if (sensorFaultDue(in)) {
        enterFault(kFaultSensor, in);
        break;
      }
      if ((in.nowMs - stateEnteredMs_) < params_.settleMs) {
        break;
      }
      if (!sensorsOk(in)) {
        // The verdict moment, with nothing trustworthy to judge. Hold here;
        // the sensor-fault timer bounds the wait (D32).
        break;
      }
      if (atBaseline(in)) {
        faultCode_ = kFaultNone; // clears an informational fault 1 (D36)
        blockageCycles_ = 0;
        queueEvent(kEventClearedVerified, in);
        enterState(State::MONITORING, in.nowMs);
      } else if (blockageCycles_ >= params_.maxCycles) {
        enterFault(kFaultStillBlocked, in);
      } else {
        ++blockageCycles_;
        enterState(State::CLEARING, in.nowMs);
        jobRequest_ = Job::SWEEP;
      }
      break;

    case State::FAULT:
      if (in.buttonHeldClear) {
        recoveringFromFault_ = true;
        enterState(State::HOMING, in.nowMs);
        jobRequest_ = Job::HOME;
        break;
      }
      // D33: only "still blocked" and "sensor" faults clear themselves, and
      // only when there is a baseline to compare against.
      if ((faultCode_ == kFaultStillBlocked ||
           faultCode_ == kFaultSensor) &&
          haveBaseline_ && sensorsOk(in) && atBaseline(in)) {
        if (!atBaselineTimer_) {
          atBaselineTimer_ = true;
          atBaselineSinceMs_ = in.nowMs;
        } else if ((in.nowMs - atBaselineSinceMs_) >=
                   params_.faultAutoRecoverMs) {
          recoveringFromFault_ = true;
          enterState(State::HOMING, in.nowMs);
          jobRequest_ = Job::HOME;
        }
      } else {
        atBaselineTimer_ = false;
      }
      break;
  }

  // One record per tick, events ahead of snapshots (D39).
  if (queueCount_ > 0) {
    out.hasRecord = true;
    out.record = popEvent();
    lastRecordMs_ = in.nowMs;
  } else if ((in.nowMs - lastRecordMs_) >= params_.snapshotMs) {
    out.hasRecord = true;
    out.record = buildRecord(kEventSnapshot, in);
    lastRecordMs_ = in.nowMs;
  }

  // The green lamp says "alive"; the 10 ms layer turns that into a heartbeat.
  out.ledStatus = state_ != State::FAULT;
  out.ledFault = state_ == State::FAULT;
  out.ledClearing = state_ == State::HOMING || state_ == State::CLEARING;

  out.motionJob = jobRequest_;
  jobRequest_ = Job::NONE;
  out.saveBaseline = saveBaseline_;
  saveBaseline_ = false;
  out.baselineCm = baselineCm_;
  out.saveCyclesSinceEmptied = saveCycles_;
  saveCycles_ = false;
  out.cyclesSinceEmptied = cyclesSinceEmptied_;
  return out;
}

} // namespace dg
