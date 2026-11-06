// S2a test suite: MotionSupervisor and HoldButton, driven against the plant
// model. Run with:  pio test -e native

#include <unity.h>

#include "tests_button.h"
#include "tests_motion.h"

void setUp(void) {}
void tearDown(void) {}

int main(int argc, char **argv) {
  (void)argc;
  (void)argv;

  UNITY_BEGIN();

  // MotionSupervisor
  RUN_TEST(dgtest::test_home_reaches_home);
  RUN_TEST(dgtest::test_home_timeout);
  RUN_TEST(dgtest::test_home_jam_fails_immediately);
  RUN_TEST(dgtest::test_sweep_clean);
  RUN_TEST(dgtest::test_sweep_reversal_has_deadtime);
  RUN_TEST(dgtest::test_inrush_spike_ignored);
  RUN_TEST(dgtest::test_raise_jam_cleared_by_retry);
  RUN_TEST(dgtest::test_raise_jam_exhausts_retries);
  RUN_TEST(dgtest::test_down_stall_retries_and_succeeds);
  RUN_TEST(dgtest::test_travel_timeout_restarts_per_retry);
  RUN_TEST(dgtest::test_retry_budget_shared_across_strokes);
  RUN_TEST(dgtest::test_terminal_status_latches);
  RUN_TEST(dgtest::test_invariant_holds_across_reversals);
  RUN_TEST(dgtest::test_home_already_home_never_moves);
  RUN_TEST(dgtest::test_sweep_from_end_skips_lower);
  RUN_TEST(dgtest::test_both_switches_fail_immediately);
  RUN_TEST(dgtest::test_stop_job_while_busy);
  RUN_TEST(dgtest::test_plant_model_self_check);

  // HoldButton
  RUN_TEST(dgtest::test_button_short_press);
  RUN_TEST(dgtest::test_button_hold_suppresses_short_press);
  RUN_TEST(dgtest::test_button_bounce_ignored);

  return UNITY_END();
}
