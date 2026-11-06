#ifndef DG_TESTS_MOTION_H
#define DG_TESTS_MOTION_H

namespace dgtest {

void test_home_reaches_home();
void test_home_timeout();
void test_home_jam_fails_immediately();
void test_sweep_clean();
void test_sweep_reversal_has_deadtime();
void test_inrush_spike_ignored();
void test_raise_jam_cleared_by_retry();
void test_raise_jam_exhausts_retries();
void test_down_stall_retries_and_succeeds();
void test_travel_timeout_restarts_per_retry();
void test_retry_budget_shared_across_strokes();
void test_terminal_status_latches();
void test_invariant_holds_across_reversals();
void test_home_already_home_never_moves();
void test_sweep_from_end_skips_lower();
void test_both_switches_fail_immediately();
void test_stop_job_while_busy();
void test_plant_model_self_check();

} // namespace dgtest

#endif // DG_TESTS_MOTION_H
