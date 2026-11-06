// Debounced push button with short-press / long-hold discrimination.
//
// Pure logic: ticked by the caller at the fast cadence (10 ms) with the already
// inverted pin state, so this header knows nothing about INPUT_PULLUP or
// active-LOW wiring. No Arduino headers, no heap (D2).
//
// Per D37: a short press means "bin emptied" in any state, the long hold means
// "clear fault", and a hold never also produces a short press on release.

#ifndef DG_BUTTON_H
#define DG_BUTTON_H

#include <stdint.h>

namespace dg {

struct ButtonParams {
  uint32_t debounceMs; // contact must hold a new level this long to count
  uint32_t holdMs;     // press held at least this long is a hold, not a press
};

class HoldButton {
public:
  explicit HoldButton(const ButtonParams& params);

  // rawPressed is the debounced-input *candidate*: true means "contact closed".
  // Call every fast tick with a monotonically non-decreasing nowMs.
  void tick(uint32_t nowMs, bool rawPressed);

  // Edge accessors. Each returns true once per occurrence and clears the flag,
  // so the 200 ms consumer cannot miss or double-count an edge.
  bool shortPress();
  bool heldClear();

  // Debounced level, for callers that want the steady state.
  bool pressed() const { return stable_; }

  void reset();

private:
  ButtonParams params_;

  bool lastRaw_;          // last raw sample, for edge timing
  uint32_t lastChangeMs_; // when lastRaw_ last changed
  bool stable_;           // debounced level
  bool started_;          // first tick seen, so lastChangeMs_ is meaningful

  uint32_t pressedAtMs_; // when the debounced press began
  bool holdFired_;       // hold already reported for this press

  bool shortPressPending_;
  bool heldClearPending_;
};

} // namespace dg

#endif // DG_BUTTON_H
