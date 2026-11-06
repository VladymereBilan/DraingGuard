#include "dg_motion.h"

namespace dg {

bool isTerminal(MotionStatus s) {
  return s == MotionStatus::DONE || s == MotionStatus::DONE_AFTER_RETRY ||
         s == MotionStatus::FAILED_JAM || s == MotionStatus::FAILED_TIMEOUT;
}

MotionSupervisor::MotionSupervisor(const MotionParams& params)
    : params_(params),
      job_(Job::NONE),
      phase_(Phase::IDLE),
      status_(MotionStatus::IDLE),
      cmd_(MotorCmd::STOP),
      jobPending_(false),
      lastDir_(MotorCmd::STOP),
      stopSinceMs_(0),
      pendingDir_(MotorCmd::STOP),
      pendingPhase_(Phase::IDLE),
      interruptedDir_(MotorCmd::STOP),
      jobStartMs_(0),
      strokeStartMs_(0),
      phaseStartMs_(0),
      motorStartMs_(0),
      jamActive_(false),
      jamSinceMs_(0),
      retries_(0) {}

float MotionSupervisor::jamThresholdA() const {
  const float scaled = params_.jamFactor * params_.iRunA;
  const float offset = params_.iRunA + params_.jamMinDeltaA;
  return scaled > offset ? scaled : offset;
}

void MotionSupervisor::setCmd(MotorCmd c, uint32_t nowMs) {
  if (c == cmd_) {
    return;
  }
  if (c == MotorCmd::STOP) {
    stopSinceMs_ = nowMs;
  } else {
    lastDir_ = c;
  }
  cmd_ = c;
}

bool MotionSupervisor::needsDeadtime(MotorCmd dir, uint32_t nowMs) const {
  if (cmd_ == dir) {
    return false; // already running that way
  }
  if (cmd_ != MotorCmd::STOP) {
    return true; // running the other way: must stop first
  }
  if (lastDir_ == MotorCmd::STOP || lastDir_ == dir) {
    return false; // never moved, or resuming the same direction
  }
  return (nowMs - stopSinceMs_) < params_.deadtimeMs;
}

void MotionSupervisor::startStroke(MotorCmd dir, Phase next, uint32_t nowMs) {
  setCmd(dir, nowMs);
  phase_ = next;
  strokeStartMs_ = nowMs;
  phaseStartMs_ = nowMs;
  motorStartMs_ = nowMs;
  jamActive_ = false;
  jamSinceMs_ = 0;
}

void MotionSupervisor::requestDirection(MotorCmd dir, Phase next,
                                        uint32_t nowMs) {
  if (needsDeadtime(dir, nowMs)) {
    pendingDir_ = dir;
    pendingPhase_ = next;
    setCmd(MotorCmd::STOP, nowMs);
    phase_ = Phase::DEADTIME;
    phaseStartMs_ = nowMs;
    jamActive_ = false;
    jamSinceMs_ = 0;
    return;
  }
  startStroke(dir, next, nowMs);
}

void MotionSupervisor::finish(MotionStatus s, uint32_t nowMs) {
  setCmd(MotorCmd::STOP, nowMs);
  phase_ = Phase::FINISHED;
  status_ = s;
  jamActive_ = false;
  jamSinceMs_ = 0;
}

void MotionSupervisor::retryOrFail(uint32_t nowMs) {
  if (retries_ >= params_.maxRetriesPerJob) {
    finish(MotionStatus::FAILED_JAM, nowMs);
    return;
  }
  ++retries_;
  // Remember which stroke was interrupted, then nudge the other way.
  interruptedDir_ = cmd_;
  const MotorCmd reverse =
      (interruptedDir_ == MotorCmd::RAISE) ? MotorCmd::LOWER : MotorCmd::RAISE;
  requestDirection(reverse, Phase::REVERSE_PULSE, nowMs);
}

bool MotionSupervisor::jamHeld(const MotionInput& in) {
  if ((in.nowMs - motorStartMs_) < params_.inrushBlankMs) {
    jamActive_ = false; // inrush window: current says nothing yet
    return false;
  }
  if (in.currentA < jamThresholdA()) {
    jamActive_ = false;
    return false;
  }
  if (!jamActive_) {
    jamActive_ = true;
    jamSinceMs_ = in.nowMs;
    return false;
  }
  return (in.nowMs - jamSinceMs_) >= params_.jamHoldMs;
}

void MotionSupervisor::requestJob(Job job, uint32_t nowMs) {
  // Stop first, unconditionally: nothing keeps moving across a job change.
  setCmd(MotorCmd::STOP, nowMs);
  job_ = job;
  jobStartMs_ = nowMs;
  retries_ = 0;
  jamActive_ = false;
  jamSinceMs_ = 0;
  phase_ = Phase::IDLE;

  if (job == Job::HOME || job == Job::SWEEP) {
    status_ = MotionStatus::BUSY;
    jobPending_ = true;
  } else {
    status_ = MotionStatus::IDLE;
    jobPending_ = false;
  }
}

MotionOutput MotionSupervisor::tick(const MotionInput& in) {
  // Wiring fault, checked before anything else can command motion (D40).
  const bool bothSwitches = in.homePressed && in.endPressed;
  if (bothSwitches && (jobPending_ || phase_ != Phase::IDLE) &&
      phase_ != Phase::FINISHED) {
    jobPending_ = false;
    finish(MotionStatus::FAILED_TIMEOUT, in.nowMs);
    return MotionOutput{cmd_, status_, retries_};
  }

  if (jobPending_) {
    jobPending_ = false;
    if (job_ == Job::HOME) {
      if (in.homePressed) {
        // Already home: report success without ever energising the motor (D40).
        finish(MotionStatus::DONE, in.nowMs);
      } else {
        requestDirection(MotorCmd::RAISE, Phase::RAISING, in.nowMs);
      }
    } else if (job_ == Job::SWEEP) {
      if (in.endPressed) {
        requestDirection(MotorCmd::RAISE, Phase::RAISING, in.nowMs); // D40
      } else {
        requestDirection(MotorCmd::LOWER, Phase::LOWERING, in.nowMs);
      }
    }
    return MotionOutput{cmd_, status_, retries_};
  }

  switch (phase_) {
    case Phase::IDLE:
    case Phase::FINISHED:
      break;

    case Phase::DEADTIME:
      if ((in.nowMs - phaseStartMs_) >= params_.deadtimeMs) {
        startStroke(pendingDir_, pendingPhase_, in.nowMs);
      }
      break;

    case Phase::LOWERING:
      if (in.endPressed) {
        requestDirection(MotorCmd::RAISE, Phase::RAISING, in.nowMs);
      } else if ((in.nowMs - strokeStartMs_) >= params_.travelTimeoutMs) {
        // Going down, a stuck rake just leaves the cord slack, so the travel
        // timeout is the jam detector and feeds the same retry path (D10).
        retryOrFail(in.nowMs);
      }
      break;

    case Phase::RAISING:
      if (in.homePressed) {
        finish(retries_ > 0 ? MotionStatus::DONE_AFTER_RETRY
                            : MotionStatus::DONE,
               in.nowMs);
      } else if (job_ == Job::HOME) {
        // Homing never retries: any failure is FAILED_TIMEOUT (D40/D41).
        if (jamHeld(in) ||
            (in.nowMs - jobStartMs_) >= params_.homingTimeoutMs) {
          finish(MotionStatus::FAILED_TIMEOUT, in.nowMs);
        }
      } else if (jamHeld(in) ||
                 (in.nowMs - strokeStartMs_) >= params_.travelTimeoutMs) {
        retryOrFail(in.nowMs);
      }
      break;

    case Phase::REVERSE_PULSE:
      // Deliberate nudge: current is not judged here (D41).
      if ((in.nowMs - phaseStartMs_) >= params_.jamReverseMs) {
        setCmd(MotorCmd::STOP, in.nowMs);
        phase_ = Phase::PAUSE;
        phaseStartMs_ = in.nowMs;
      }
      break;

    case Phase::PAUSE:
      if ((in.nowMs - phaseStartMs_) >= params_.jamPauseMs) {
        // Resume the interrupted stroke; its travel timeout restarts (D17).
        const Phase next = (interruptedDir_ == MotorCmd::RAISE)
                               ? Phase::RAISING
                               : Phase::LOWERING;
        requestDirection(interruptedDir_, next, in.nowMs);
      }
      break;
  }

  return MotionOutput{cmd_, status_, retries_};
}

void MotionSupervisor::reset() {
  job_ = Job::NONE;
  phase_ = Phase::IDLE;
  status_ = MotionStatus::IDLE;
  cmd_ = MotorCmd::STOP;
  jobPending_ = false;
  lastDir_ = MotorCmd::STOP;
  stopSinceMs_ = 0;
  pendingDir_ = MotorCmd::STOP;
  pendingPhase_ = Phase::IDLE;
  interruptedDir_ = MotorCmd::STOP;
  jobStartMs_ = 0;
  strokeStartMs_ = 0;
  phaseStartMs_ = 0;
  motorStartMs_ = 0;
  jamActive_ = false;
  jamSinceMs_ = 0;
  retries_ = 0;
}

} // namespace dg
