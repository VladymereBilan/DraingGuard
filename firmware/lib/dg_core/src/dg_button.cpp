#include "dg_button.h"

namespace dg {

HoldButton::HoldButton(const ButtonParams& params)
    : params_(params),
      lastRaw_(false),
      lastChangeMs_(0),
      stable_(false),
      started_(false),
      pressedAtMs_(0),
      holdFired_(false),
      shortPressPending_(false),
      heldClearPending_(false) {}

void HoldButton::tick(uint32_t nowMs, bool rawPressed) {
  if (!started_) {
    started_ = true;
    lastRaw_ = rawPressed;
    lastChangeMs_ = nowMs;
    stable_ = rawPressed;
    pressedAtMs_ = nowMs;
    return;
  }

  if (rawPressed != lastRaw_) {
    // Level moved: restart the debounce window and wait for it to settle.
    lastRaw_ = rawPressed;
    lastChangeMs_ = nowMs;
  } else if (rawPressed != stable_ &&
             (nowMs - lastChangeMs_) >= params_.debounceMs) {
    stable_ = rawPressed;
    if (stable_) {
      pressedAtMs_ = nowMs;
      holdFired_ = false;
    } else if (!holdFired_) {
      // Released before reaching the hold time: that was a short press. A press
      // that already fired a hold reports nothing on release (D37).
      shortPressPending_ = true;
    }
  }

  if (stable_ && !holdFired_ && (nowMs - pressedAtMs_) >= params_.holdMs) {
    holdFired_ = true;
    heldClearPending_ = true;
  }
}

bool HoldButton::shortPress() {
  const bool edge = shortPressPending_;
  shortPressPending_ = false;
  return edge;
}

bool HoldButton::heldClear() {
  const bool edge = heldClearPending_;
  heldClearPending_ = false;
  return edge;
}

void HoldButton::reset() {
  lastRaw_ = false;
  lastChangeMs_ = 0;
  stable_ = false;
  started_ = false;
  pressedAtMs_ = 0;
  holdFired_ = false;
  shortPressPending_ = false;
  heldClearPending_ = false;
}

} // namespace dg
