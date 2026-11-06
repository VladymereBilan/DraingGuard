#include "plant.h"

namespace dgtest {
namespace {

constexpr float kStepMs = 10.0f;
constexpr float kAtHome = 0.0001f;
constexpr float kAtEnd = 0.9999f;

} // namespace

Plant::Plant(const PlantParams& params)
    : params_(params),
      nowMs_(0),
      posFrac_(0.0f),
      lastCmd_(dg::MotorCmd::STOP),
      startMs_(0),
      inrush_(false),
      jamOnRaise_(false),
      stallOnRaise_(false),
      stallOnLower_(false),
      forceBoth_(false),
      reachedEnd_(false),
      loweredSinceHome_(false),
      sweeps_(0),
      sweepsToClear_(1),
      blockedDiffCm_(0.0f),
      clearDiffCm_(0.0f),
      downstreamLevelCm_(0.0f),
      cleared_(false),
      diffOverride_(false),
      diffOverrideCm_(0.0f) {}

bool Plant::homePressed() const {
  return forceBoth_ || posFrac_ <= kAtHome;
}

bool Plant::endPressed() const {
  return forceBoth_ || posFrac_ >= kAtEnd;
}

float Plant::currentA() const {
  if (lastCmd_ == dg::MotorCmd::STOP) {
    return params_.iIdleA;
  }
  if (inrush_ && (nowMs_ - startMs_) < params_.inrushMs) {
    return params_.iInrushA;
  }
  if (lastCmd_ == dg::MotorCmd::LOWER) {
    return params_.iLowerA;
  }
  return jamOnRaise_ ? params_.iJamA : params_.iRaiseA;
}

void Plant::step10ms(dg::MotorCmd cmd) {
  if (cmd != lastCmd_) {
    if (cmd != dg::MotorCmd::STOP) {
      startMs_ = nowMs_; // motor just began turning: inrush starts now
    }
    lastCmd_ = cmd;
  }

  if (cmd == dg::MotorCmd::LOWER) {
    loweredSinceHome_ = true;
    if (!stallOnLower_) {
      posFrac_ += kStepMs / static_cast<float>(params_.travelMsDown);
    }
  } else if (cmd == dg::MotorCmd::RAISE) {
    if (!stallOnRaise_ && !jamOnRaise_) {
      posFrac_ -= kStepMs / static_cast<float>(params_.travelMsUp);
    }
  }

  if (posFrac_ < 0.0f) {
    posFrac_ = 0.0f;
  } else if (posFrac_ > 1.0f) {
    posFrac_ = 1.0f;
  }

  if (endPressed()) {
    reachedEnd_ = true;
  }
  // A sweep counts only once the rake has gone down and come back home.
  if (homePressed() && reachedEnd_ && loweredSinceHome_) {
    ++sweeps_;
    reachedEnd_ = false;
    loweredSinceHome_ = false;
    if (sweeps_ >= sweepsToClear_) {
      cleared_ = true;
    }
  }

  nowMs_ += 10;
}

void Plant::configureWater(uint8_t sweepsToClear, float blockedDiffCm,
                           float clearDiffCm, float downstreamLevelCm) {
  sweepsToClear_ = sweepsToClear == 0 ? 1 : sweepsToClear;
  blockedDiffCm_ = blockedDiffCm;
  clearDiffCm_ = clearDiffCm;
  downstreamLevelCm_ = downstreamLevelCm;
  cleared_ = false;
}

float Plant::diffCm() const {
  if (diffOverride_) {
    return diffOverrideCm_;
  }
  return cleared_ ? clearDiffCm_ : blockedDiffCm_;
}

void Plant::setDiffOverride(float diff) {
  diffOverride_ = true;
  diffOverrideCm_ = diff;
}

} // namespace dgtest
