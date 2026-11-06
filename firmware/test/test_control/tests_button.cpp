// HoldButton tests. The button is ticked at the 10 ms safety cadence, the same
// rate the firmware will use.

#include "tests_button.h"

#include <unity.h>

#include "config.h"
#include "dg_button.h"

namespace dgtest {
namespace {

const dg::ButtonParams kButton = {cfg::DEBOUNCE_MS, cfg::BTN_HOLD_CLEAR_MS};

constexpr uint32_t kTickMs = 10;

// Holds the raw line at `pressed` for `ms`, ticking every 10 ms.
void drive(dg::HoldButton& b, uint32_t& nowMs, bool pressed, uint32_t ms) {
  for (uint32_t t = 0; t < ms; t += kTickMs) {
    b.tick(nowMs, pressed);
    nowMs += kTickMs;
  }
}

} // namespace

// 19. A press released before the hold time is exactly one short press.
void test_button_short_press() {
  dg::HoldButton b(kButton);
  uint32_t now = 0;

  drive(b, now, false, 200);
  drive(b, now, true, 1000); // well past debounce, well short of the hold
  drive(b, now, false, 200);

  TEST_ASSERT_TRUE(b.shortPress());
  TEST_ASSERT_FALSE(b.shortPress()); // the edge was consumed, not repeated
  TEST_ASSERT_FALSE(b.heldClear());
}

// 20. A hold reports once and never also counts as a short press (D37).
void test_button_hold_suppresses_short_press() {
  dg::HoldButton b(kButton);
  uint32_t now = 0;

  drive(b, now, false, 200);
  drive(b, now, true, cfg::BTN_HOLD_CLEAR_MS + 500);

  TEST_ASSERT_TRUE(b.heldClear());
  TEST_ASSERT_FALSE(b.heldClear()); // fires once per press, not per tick

  drive(b, now, true, 1000); // still held: no second hold
  TEST_ASSERT_FALSE(b.heldClear());

  drive(b, now, false, 200); // release after a hold stays silent
  TEST_ASSERT_FALSE(b.shortPress());
  TEST_ASSERT_FALSE(b.heldClear());
}

// 21. A contact bounce shorter than the debounce window produces nothing.
void test_button_bounce_ignored() {
  dg::HoldButton b(kButton);
  uint32_t now = 0;

  TEST_ASSERT_TRUE(kButton.debounceMs > 20);

  drive(b, now, false, 200);
  drive(b, now, true, 20); // 20 ms glitch against a 30 ms debounce
  drive(b, now, false, 300);

  TEST_ASSERT_FALSE(b.shortPress());
  TEST_ASSERT_FALSE(b.heldClear());
  TEST_ASSERT_FALSE(b.pressed());
}

} // namespace dgtest
