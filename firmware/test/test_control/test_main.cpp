// S2a test suite: MotionSupervisor and HoldButton, driven against the plant
// model. Run with:  pio test -e native

#include <unity.h>

#include "tests_button.h"
#include "tests_controller.h"
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
  RUN_TEST(dgtest::test_negative_jam_current_detected);
  RUN_TEST(dgtest::test_negative_inrush_spike_ignored);
  RUN_TEST(dgtest::test_stroke_peak_tracks_magnitude);

  // HoldButton
  RUN_TEST(dgtest::test_button_short_press);
  RUN_TEST(dgtest::test_button_hold_suppresses_short_press);
  RUN_TEST(dgtest::test_button_bounce_ignored);

  // Controller
  RUN_TEST(dgtest::test_c1_one_cycle_clear);
  RUN_TEST(dgtest::test_c2_two_cycle_clear);
  RUN_TEST(dgtest::test_c3_three_cycles_still_blocked);
  RUN_TEST(dgtest::test_c4_false_alarm_in_confirming);
  RUN_TEST(dgtest::test_c5_raise_jam_cleared_by_retry);
  RUN_TEST(dgtest::test_c6_three_jams_fault2);
  RUN_TEST(dgtest::test_c7_down_stall_retry_then_settling);
  RUN_TEST(dgtest::test_c8_homing_timeout_fault4);
  RUN_TEST(dgtest::test_c9_fault3_cleared_by_hold);
  RUN_TEST(dgtest::test_c10_fault3_auto_recovers);
  RUN_TEST(dgtest::test_c11_fault2_does_not_auto_recover);
  RUN_TEST(dgtest::test_c12_fault4_first_boot_rehomes_to_baseline);
  RUN_TEST(dgtest::test_c13_short_press_bin_emptied);
  RUN_TEST(dgtest::test_c14_button_scope);
  RUN_TEST(dgtest::test_c15_baseline_rejected);
  RUN_TEST(dgtest::test_c16_first_boot_baseline_saved);
  RUN_TEST(dgtest::test_c17_cycles_since_emptied);
  RUN_TEST(dgtest::test_c18_snapshot_in_every_state);
  RUN_TEST(dgtest::test_c19_event_resets_snapshot_timer);
  RUN_TEST(dgtest::test_c20_two_events_same_tick);
  RUN_TEST(dgtest::test_c21_invalid_in_monitoring);
  RUN_TEST(dgtest::test_c22_invalid_in_confirming);
  RUN_TEST(dgtest::test_c23_invalid_at_settling_verdict);
  RUN_TEST(dgtest::test_c24_invalid_through_baseline);
  RUN_TEST(dgtest::test_c25_fault5_auto_recovers);
  RUN_TEST(dgtest::test_c26_led_table);
  RUN_TEST(dgtest::test_c27_fault_stops_motor);
  RUN_TEST(dgtest::test_c28_record_contents);
  RUN_TEST(dgtest::test_c29_records_carry_sensor_validity);

  return UNITY_END();
}
