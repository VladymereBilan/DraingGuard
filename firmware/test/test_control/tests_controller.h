#ifndef DG_TESTS_CONTROLLER_H
#define DG_TESTS_CONTROLLER_H

namespace dgtest {

void test_c1_one_cycle_clear();
void test_c2_two_cycle_clear();
void test_c3_three_cycles_still_blocked();
void test_c4_false_alarm_in_confirming();
void test_c5_raise_jam_cleared_by_retry();
void test_c6_three_jams_fault2();
void test_c7_down_stall_retry_then_settling();
void test_c8_homing_timeout_fault4();
void test_c9_fault3_cleared_by_hold();
void test_c10_fault3_auto_recovers();
void test_c11_fault2_does_not_auto_recover();
void test_c12_fault4_first_boot_rehomes_to_baseline();
void test_c13_short_press_bin_emptied();
void test_c14_button_scope();
void test_c15_baseline_rejected();
void test_c16_first_boot_baseline_saved();
void test_c17_cycles_since_emptied();
void test_c18_snapshot_in_every_state();
void test_c19_event_resets_snapshot_timer();
void test_c20_two_events_same_tick();
void test_c21_invalid_in_monitoring();
void test_c22_invalid_in_confirming();
void test_c23_invalid_at_settling_verdict();
void test_c24_invalid_through_baseline();
void test_c25_fault5_auto_recovers();
void test_c26_led_table();
void test_c27_fault_stops_motor();
void test_c28_record_contents();

} // namespace dgtest

#endif // DG_TESTS_CONTROLLER_H
