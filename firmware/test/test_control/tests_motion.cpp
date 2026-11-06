// MotionSupervisor tests. Every one runs through MotionHarness, so the motor
// safety invariant is asserted on every 10 ms tick of every test.

#include "tests_motion.h"

#include <unity.h>

#include "config.h"
#include "dg_motion.h"
#include "harness.h"
#include "params.h"
#include "plant.h"

namespace dgtest {

// 1. HOME raises until the home switch closes.
void test_home_reaches_home() {
  dg::MotionSupervisor sup(kMotion);
  Plant plant(kPlant);
  MotionHarness h(sup, plant, cfg::DEADTIME_BEFORE_REVERSE_MS);
  plant.placeAtEnd();

  const MotionObs obs = h.runJob(dg::Job::HOME, 30000);

  TEST_ASSERT_TRUE(obs.terminal);
  TEST_ASSERT_EQUAL(dg::MotionStatus::DONE, obs.status);
  TEST_ASSERT_EQUAL_UINT8(0, obs.retriesUsed);
  TEST_ASSERT_TRUE(h.sawRaise());
  TEST_ASSERT_FALSE(h.sawLower());
  TEST_ASSERT_UINT32_WITHIN(kSlackMs, kStrokeMs, obs.elapsedMs);
  TEST_ASSERT_EQUAL(dg::MotorCmd::STOP, h.last().cmd);
}

// 2. HOME that never arrives fails on the homing timeout.
void test_home_timeout() {
  dg::MotionSupervisor sup(kMotion);
  Plant plant(kPlant);
  MotionHarness h(sup, plant, cfg::DEADTIME_BEFORE_REVERSE_MS);
  plant.placeAtEnd();
  plant.injectStallOnRaise(true); // frozen, but current stays normal

  const MotionObs obs = h.runJob(dg::Job::HOME, 30000);

  TEST_ASSERT_EQUAL(dg::MotionStatus::FAILED_TIMEOUT, obs.status);
  TEST_ASSERT_UINT32_WITHIN(kSlackMs, cfg::HOMING_TIMEOUT_MS, obs.elapsedMs);
  TEST_ASSERT_EQUAL(dg::MotorCmd::STOP, h.last().cmd);
}

// 3. A jam while homing fails at once instead of stalling for 15 s (D41).
void test_home_jam_fails_immediately() {
  dg::MotionSupervisor sup(kMotion);
  Plant plant(kPlant);
  MotionHarness h(sup, plant, cfg::DEADTIME_BEFORE_REVERSE_MS);
  plant.placeAtEnd();
  plant.injectJamOnRaise(true);

  const MotionObs obs = h.runJob(dg::Job::HOME, 30000);

  TEST_ASSERT_EQUAL(dg::MotionStatus::FAILED_TIMEOUT, obs.status);
  TEST_ASSERT_EQUAL_UINT8(0, obs.retriesUsed); // homing never retries
  TEST_ASSERT_UINT32_WITHIN(kSlackMs, kJamDetectMs, obs.elapsedMs);
  TEST_ASSERT_TRUE(obs.elapsedMs < cfg::HOMING_TIMEOUT_MS);
  TEST_ASSERT_EQUAL(dg::MotorCmd::STOP, h.last().cmd);
}

// 4. A clean sweep lowers to the end switch then raises back home.
void test_sweep_clean() {
  dg::MotionSupervisor sup(kMotion);
  Plant plant(kPlant);
  MotionHarness h(sup, plant, cfg::DEADTIME_BEFORE_REVERSE_MS);
  plant.placeAtHome();

  const MotionObs obs = h.runJob(dg::Job::SWEEP, 30000);

  TEST_ASSERT_EQUAL(dg::MotionStatus::DONE, obs.status);
  TEST_ASSERT_EQUAL_UINT8(0, obs.retriesUsed);
  TEST_ASSERT_TRUE(h.sawLower());
  TEST_ASSERT_TRUE(h.sawRaise());
  TEST_ASSERT_UINT32_WITHIN(kSlackMs, kSweepMs, obs.elapsedMs);
  TEST_ASSERT_EQUAL_UINT16(1, plant.sweepsCompleted());
}

// 5. The lower -> raise reversal is separated by a commanded STOP.
void test_sweep_reversal_has_deadtime() {
  dg::MotionSupervisor sup(kMotion);
  Plant plant(kPlant);
  MotionHarness h(sup, plant, cfg::DEADTIME_BEFORE_REVERSE_MS);
  plant.placeAtHome();

  const MotionObs obs = h.runJob(dg::Job::SWEEP, 30000);

  TEST_ASSERT_EQUAL(dg::MotionStatus::DONE, obs.status);
  TEST_ASSERT_EQUAL_UINT32(1, h.reversals()); // exactly one, lower -> raise
  TEST_ASSERT_GREATER_OR_EQUAL_UINT32(cfg::DEADTIME_BEFORE_REVERSE_MS,
                                      h.minReversalStopMs());
}

// 6. The start-up current spike falls inside the blanking window.
void test_inrush_spike_ignored() {
  dg::MotionSupervisor sup(kMotion);
  Plant plant(kPlant);
  MotionHarness h(sup, plant, cfg::DEADTIME_BEFORE_REVERSE_MS);
  plant.placeAtHome();
  plant.setInrush(true); // 2.5 A for 250 ms, well over the 0.8 A threshold

  TEST_ASSERT_TRUE(kPlant.iInrushA > sup.jamThresholdA());
  TEST_ASSERT_TRUE(kPlant.inrushMs < cfg::INRUSH_BLANK_MS);

  const MotionObs obs = h.runJob(dg::Job::SWEEP, 30000);

  TEST_ASSERT_EQUAL(dg::MotionStatus::DONE, obs.status);
  TEST_ASSERT_EQUAL_UINT8(0, obs.retriesUsed);
}

// 7. A jam on the raise stroke is shaken loose and the sweep completes.
void test_raise_jam_cleared_by_retry() {
  dg::MotionSupervisor sup(kMotion);
  Plant plant(kPlant);
  MotionHarness h(sup, plant, cfg::DEADTIME_BEFORE_REVERSE_MS);
  plant.placeAtHome();
  plant.injectJamOnRaise(true); // only bites on the raise stroke

  h.request(dg::Job::SWEEP);
  // Down stroke, reversal, jam detection, then part of the recovery.
  h.runFor(kStrokeMs + cfg::DEADTIME_BEFORE_REVERSE_MS + kJamDetectMs +
           cfg::DEADTIME_BEFORE_REVERSE_MS + cfg::JAM_REVERSE_MS + 100);
  TEST_ASSERT_FALSE(dg::isTerminal(h.last().status));
  TEST_ASSERT_EQUAL_UINT8(1, h.last().retriesUsed);

  plant.injectJamOnRaise(false); // obstruction gone before the retry stroke
  const MotionObs obs = h.runUntilTerminal(30000);

  TEST_ASSERT_EQUAL(dg::MotionStatus::DONE_AFTER_RETRY, obs.status);
  TEST_ASSERT_EQUAL_UINT8(1, obs.retriesUsed);
  TEST_ASSERT_EQUAL(dg::MotorCmd::STOP, h.last().cmd);
}

// 8. A jam that will not clear burns exactly the retry budget, then fails.
void test_raise_jam_exhausts_retries() {
  dg::MotionSupervisor sup(kMotion);
  Plant plant(kPlant);
  MotionHarness h(sup, plant, cfg::DEADTIME_BEFORE_REVERSE_MS);
  plant.placeAtHome();
  plant.injectJamOnRaise(true);

  const MotionObs obs = h.runJob(dg::Job::SWEEP, 60000);

  TEST_ASSERT_EQUAL(dg::MotionStatus::FAILED_JAM, obs.status);
  TEST_ASSERT_EQUAL_UINT8(cfg::MAX_RETRIES_PER_CYCLE, obs.retriesUsed);
  TEST_ASSERT_EQUAL(dg::MotorCmd::STOP, h.last().cmd);

  // Three jams: the first plus one per retry, each preceded by a recovery.
  const uint32_t expected = kStrokeMs + cfg::DEADTIME_BEFORE_REVERSE_MS +
                            kJamDetectMs +
                            2 * (kRecoverMs + kJamDetectMs);
  TEST_ASSERT_UINT32_WITHIN(kSlackMs, expected, obs.elapsedMs);
}

// 9. A stuck down stroke is caught by the travel timeout and retried (D10).
void test_down_stall_retries_and_succeeds() {
  dg::MotionSupervisor sup(kMotion);
  Plant plant(kPlant);
  MotionHarness h(sup, plant, cfg::DEADTIME_BEFORE_REVERSE_MS);
  plant.placeAtHome();
  plant.injectStallOnLower(true);

  h.request(dg::Job::SWEEP);
  h.runFor(cfg::TRAVEL_TIMEOUT_MS + 500); // timeout fires, recovery starts
  TEST_ASSERT_FALSE(dg::isTerminal(h.last().status));
  TEST_ASSERT_EQUAL_UINT8(1, h.last().retriesUsed);

  plant.injectStallOnLower(false);
  const MotionObs obs = h.runUntilTerminal(60000);

  TEST_ASSERT_EQUAL(dg::MotionStatus::DONE_AFTER_RETRY, obs.status);
  TEST_ASSERT_EQUAL_UINT8(1, obs.retriesUsed);
  TEST_ASSERT_TRUE(h.sawLower());
  TEST_ASSERT_TRUE(h.sawRaise());
}

// 10. Each retry gets a fresh travel timeout rather than a shared budget (D17).
void test_travel_timeout_restarts_per_retry() {
  dg::MotionSupervisor sup(kMotion);
  Plant plant(kPlant);
  MotionHarness h(sup, plant, cfg::DEADTIME_BEFORE_REVERSE_MS);
  plant.placeAtHome();
  plant.injectStallOnLower(true); // never clears

  const MotionObs obs = h.runJob(dg::Job::SWEEP, 120000);

  TEST_ASSERT_EQUAL(dg::MotionStatus::FAILED_JAM, obs.status);
  TEST_ASSERT_EQUAL_UINT8(cfg::MAX_RETRIES_PER_CYCLE, obs.retriesUsed);

  // Three full timeout windows, not one: 3 x 15 s plus two recoveries.
  const uint32_t expected =
      3 * cfg::TRAVEL_TIMEOUT_MS + 2 * kRecoverMs;
  TEST_ASSERT_UINT32_WITHIN(kSlackMs, expected, obs.elapsedMs);
  TEST_ASSERT_TRUE(obs.elapsedMs > 2 * cfg::TRAVEL_TIMEOUT_MS);
}

// 11. The retry budget belongs to the job, not to each stroke.
void test_retry_budget_shared_across_strokes() {
  dg::MotionSupervisor sup(kMotion);
  Plant plant(kPlant);
  MotionHarness h(sup, plant, cfg::DEADTIME_BEFORE_REVERSE_MS);
  plant.placeAtHome();
  plant.injectStallOnLower(true);

  h.request(dg::Job::SWEEP);
  h.runFor(cfg::TRAVEL_TIMEOUT_MS + 500); // down stroke burns retry 1
  TEST_ASSERT_EQUAL_UINT8(1, h.last().retriesUsed);

  plant.injectStallOnLower(false);
  plant.injectJamOnRaise(true); // raise stroke will burn retry 2
  const MotionObs obs = h.runUntilTerminal(120000);

  TEST_ASSERT_EQUAL(dg::MotionStatus::FAILED_JAM, obs.status);
  TEST_ASSERT_EQUAL_UINT8(cfg::MAX_RETRIES_PER_CYCLE, obs.retriesUsed);
  TEST_ASSERT_TRUE(h.sawLower());
  TEST_ASSERT_TRUE(h.sawRaise());
}

// 12. A terminal status survives far longer than one controller period.
void test_terminal_status_latches() {
  dg::MotionSupervisor sup(kMotion);
  Plant plant(kPlant);
  MotionHarness h(sup, plant, cfg::DEADTIME_BEFORE_REVERSE_MS);
  plant.placeAtEnd();

  const MotionObs obs = h.runJob(dg::Job::HOME, 30000);
  TEST_ASSERT_EQUAL(dg::MotionStatus::DONE, obs.status);

  // 25 ticks is more than one 200 ms controller tick, so the Controller cannot
  // miss the result however the two cadences line up.
  for (int i = 0; i < 25; ++i) {
    const dg::MotionOutput out = h.step();
    TEST_ASSERT_EQUAL(dg::MotionStatus::DONE, out.status);
    TEST_ASSERT_EQUAL(dg::MotorCmd::STOP, out.cmd);
  }
}

// 13. The invariant is exercised, not merely never triggered.
void test_invariant_holds_across_reversals() {
  dg::MotionSupervisor sup(kMotion);
  Plant plant(kPlant);
  MotionHarness h(sup, plant, cfg::DEADTIME_BEFORE_REVERSE_MS);
  plant.placeAtHome();
  plant.injectStallOnLower(true);

  h.request(dg::Job::SWEEP);
  h.runFor(cfg::TRAVEL_TIMEOUT_MS + 500);
  plant.injectStallOnLower(false);
  const MotionObs obs = h.runUntilTerminal(60000);

  TEST_ASSERT_TRUE(obs.terminal);
  // lower -> reverse nudge -> lower again -> raise: at least three reversals.
  TEST_ASSERT_GREATER_OR_EQUAL_UINT32(3, h.reversals());
  TEST_ASSERT_GREATER_OR_EQUAL_UINT32(cfg::DEADTIME_BEFORE_REVERSE_MS,
                                      h.minReversalStopMs());
}

// 14. HOME with the switch already closed never energises the motor (D40).
void test_home_already_home_never_moves() {
  dg::MotionSupervisor sup(kMotion);
  Plant plant(kPlant);
  MotionHarness h(sup, plant, cfg::DEADTIME_BEFORE_REVERSE_MS);
  plant.placeAtHome();

  const MotionObs obs = h.runJob(dg::Job::HOME, 1000);

  TEST_ASSERT_EQUAL(dg::MotionStatus::DONE, obs.status);
  TEST_ASSERT_EQUAL_UINT32(kTickMs, obs.elapsedMs); // decided on the first tick
  TEST_ASSERT_EQUAL_UINT32(1, h.ticks());
  TEST_ASSERT_FALSE(h.sawLower());
  TEST_ASSERT_FALSE(h.sawRaise());
  TEST_ASSERT_EQUAL(dg::MotorCmd::STOP, h.last().cmd);
}

// 15. SWEEP with the end switch already closed skips the lower stroke (D40).
void test_sweep_from_end_skips_lower() {
  dg::MotionSupervisor sup(kMotion);
  Plant plant(kPlant);
  MotionHarness h(sup, plant, cfg::DEADTIME_BEFORE_REVERSE_MS);
  plant.placeAtEnd();

  const MotionObs obs = h.runJob(dg::Job::SWEEP, 30000);

  TEST_ASSERT_EQUAL(dg::MotionStatus::DONE, obs.status);
  TEST_ASSERT_FALSE(h.sawLower()); // never commanded LOWER at all
  TEST_ASSERT_TRUE(h.sawRaise());
  // Only the raise stroke, so a sweep's worth of time is never spent.
  TEST_ASSERT_UINT32_WITHIN(kSlackMs + cfg::DEADTIME_BEFORE_REVERSE_MS,
                            kStrokeMs, obs.elapsedMs);
}

// 16. Both limit switches closed at once is a wiring fault: stop and fail (D40).
void test_both_switches_fail_immediately() {
  dg::MotionSupervisor sup(kMotion);
  Plant plant(kPlant);
  MotionHarness h(sup, plant, cfg::DEADTIME_BEFORE_REVERSE_MS);
  plant.placeAtHome();

  h.request(dg::Job::SWEEP);
  h.runFor(500);
  TEST_ASSERT_EQUAL(dg::MotorCmd::LOWER, h.last().cmd); // mid stroke

  plant.forceBothSwitches(true);
  const dg::MotionOutput out = h.step();

  TEST_ASSERT_EQUAL(dg::MotorCmd::STOP, out.cmd);
  TEST_ASSERT_EQUAL(dg::MotionStatus::FAILED_TIMEOUT, out.status);
}

// 17. A STOP job takes effect without waiting for anything.
void test_stop_job_while_busy() {
  dg::MotionSupervisor sup(kMotion);
  Plant plant(kPlant);
  MotionHarness h(sup, plant, cfg::DEADTIME_BEFORE_REVERSE_MS);
  plant.placeAtHome();

  h.request(dg::Job::SWEEP);
  h.runFor(500);
  TEST_ASSERT_EQUAL(dg::MotorCmd::LOWER, h.last().cmd);

  h.request(dg::Job::STOP);
  const dg::MotionOutput out = h.step();

  TEST_ASSERT_EQUAL(dg::MotorCmd::STOP, out.cmd);
  TEST_ASSERT_EQUAL(dg::MotionStatus::IDLE, out.status);
}

// 19. A negative jam current is still a jam: an ACS712 fitted the other way
// round must not silently disable jam detection (D64).
void test_negative_jam_current_detected() {
  // Same plant, but the jam and inrush currents read negative.
  PlantParams reversed = kPlant;
  reversed.iJamA = -2.00f;
  reversed.iRaiseA = -0.50f;
  reversed.iLowerA = -0.10f;

  dg::MotionSupervisor sup(kMotion);
  Plant plant(reversed);
  MotionHarness h(sup, plant, cfg::DEADTIME_BEFORE_REVERSE_MS);
  plant.placeAtEnd();
  plant.injectJamOnRaise(true);

  const MotionObs obs = h.runJob(dg::Job::HOME, 30000);

  // Detected on the magnitude, at the same moment a +2 A jam would be.
  TEST_ASSERT_EQUAL(dg::MotionStatus::FAILED_TIMEOUT, obs.status);
  TEST_ASSERT_UINT32_WITHIN(kSlackMs, kJamDetectMs, obs.elapsedMs);
  TEST_ASSERT_EQUAL(dg::MotorCmd::STOP, h.last().cmd);

  // And -0.5 A of ordinary running current is still under the 0.8 A threshold,
  // so a reversed sensor does not invent jams either.
  dg::MotionSupervisor sup2(kMotion);
  Plant plant2(reversed);
  MotionHarness h2(sup2, plant2, cfg::DEADTIME_BEFORE_REVERSE_MS);
  plant2.placeAtHome();
  const MotionObs clean = h2.runJob(dg::Job::SWEEP, 30000);
  TEST_ASSERT_EQUAL(dg::MotionStatus::DONE, clean.status);
  TEST_ASSERT_EQUAL_UINT8(0, clean.retriesUsed);
}

// 20. A negative inrush spike inside the blanking window is still ignored.
void test_negative_inrush_spike_ignored() {
  PlantParams reversed = kPlant;
  reversed.iInrushA = -2.50f;
  reversed.iRaiseA = -0.50f;
  reversed.iLowerA = -0.10f;

  dg::MotionSupervisor sup(kMotion);
  Plant plant(reversed);
  MotionHarness h(sup, plant, cfg::DEADTIME_BEFORE_REVERSE_MS);
  plant.placeAtHome();
  plant.setInrush(true);

  // The spike's magnitude clears the threshold, so only blanking saves us.
  TEST_ASSERT_TRUE(-reversed.iInrushA > sup.jamThresholdA());
  TEST_ASSERT_TRUE(reversed.inrushMs < cfg::INRUSH_BLANK_MS);

  const MotionObs obs = h.runJob(dg::Job::SWEEP, 30000);
  TEST_ASSERT_EQUAL(dg::MotionStatus::DONE, obs.status);
  TEST_ASSERT_EQUAL_UINT8(0, obs.retriesUsed);
}

// 21. The per-stroke peak tracks magnitude and resets each stroke, so it is
// usable for calibrating I_RUN_A from the bench.
void test_stroke_peak_tracks_magnitude() {
  PlantParams reversed = kPlant;
  reversed.iRaiseA = -0.60f;
  reversed.iLowerA = -0.10f;

  dg::MotionSupervisor sup(kMotion);
  Plant plant(reversed);
  MotionHarness h(sup, plant, cfg::DEADTIME_BEFORE_REVERSE_MS);
  plant.placeAtEnd();

  const MotionObs obs = h.runJob(dg::Job::HOME, 30000);
  TEST_ASSERT_EQUAL(dg::MotionStatus::DONE, obs.status);
  // Reported as a positive magnitude despite the negative reading.
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.60f, sup.strokePeakA());
}

// 18. The plant itself behaves, so a failing motion test means the supervisor.
void test_plant_model_self_check() {
  Plant plant(kPlant);
  plant.placeAtHome();
  plant.configureWater(2, 6.0f, 0.5f, 10.0f);

  TEST_ASSERT_TRUE(plant.homePressed());
  TEST_ASSERT_FALSE(plant.endPressed());
  TEST_ASSERT_FLOAT_WITHIN(0.001f, kPlant.iIdleA, plant.currentA());

  // A full down stroke takes travelMsDown and closes the end switch.
  for (uint32_t t = 0; t < kPlant.travelMsDown; t += kTickMs) {
    plant.step10ms(dg::MotorCmd::LOWER);
  }
  TEST_ASSERT_TRUE(plant.endPressed());
  TEST_ASSERT_FALSE(plant.homePressed());
  TEST_ASSERT_FLOAT_WITHIN(0.001f, kPlant.iLowerA, plant.currentA());

  // Raising draws the free-running current and returns home, completing a sweep.
  for (uint32_t t = 0; t < kPlant.travelMsUp; t += kTickMs) {
    plant.step10ms(dg::MotorCmd::RAISE);
  }
  TEST_ASSERT_TRUE(plant.homePressed());
  TEST_ASSERT_EQUAL_UINT16(1, plant.sweepsCompleted());
  TEST_ASSERT_FLOAT_WITHIN(0.001f, kPlant.iRaiseA, plant.currentA());

  // Water needs two sweeps to clear, so it is still blocked.
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 6.0f, plant.diffCm());
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 16.0f, plant.upstreamCm());
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.0f, plant.downstreamCm());

  // A jammed raise freezes the rake and pins the current high.
  plant.placeAt(0.5f);
  plant.injectJamOnRaise(true);
  plant.step10ms(dg::MotorCmd::RAISE);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.5f, plant.posFrac());
  TEST_ASSERT_FLOAT_WITHIN(0.001f, kPlant.iJamA, plant.currentA());

  // Both switches at once is reportable for the wiring-fault test.
  plant.forceBothSwitches(true);
  TEST_ASSERT_TRUE(plant.homePressed());
  TEST_ASSERT_TRUE(plant.endPressed());
}

} // namespace dgtest
