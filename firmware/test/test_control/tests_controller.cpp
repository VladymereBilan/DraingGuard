// Controller tests. The Controller runs at 200 ms on top of the 10 ms
// MotionSupervisor and the plant, and the dead-time invariant is still asserted
// on every 10 ms step (ControlHarness::step).

#include "tests_controller.h"

#include <unity.h>

#include "config.h"
#include "dg_controller.h"
#include "harness.h"
#include "params.h"
#include "plant.h"

namespace dgtest {
namespace {

constexpr uint32_t kBootBudget = cfg::BASELINE_MS + 20000;
constexpr uint32_t kCycleBudget = cfg::SETTLE_MS + kSweepMs + 10000;
constexpr uint32_t kDeadtime = cfg::DEADTIME_BEFORE_REVERSE_MS;

// Boots from the home switch, averages a clear-channel baseline and lands in
// MONITORING with the record log cleared.
void bootToMonitoring(ControlHarness& h, bool savedValid = false,
                      float savedCm = 0.0f, uint16_t cycles = 0) {
  h.plant().configureWater(1, kBlockedDiffCm, kClearDiffCm, kDownstreamCm);
  h.plant().setDiffOverride(kClearDiffCm);
  h.begin(savedValid, savedCm, cycles);
  TEST_ASSERT_TRUE(h.runUntilState(dg::State::MONITORING, kBootBudget));
  h.clearRecords();
}

void block(ControlHarness& h) { h.plant().setDiffOverride(kBlockedDiffCm); }
void unblock(ControlHarness& h) { h.plant().setDiffOverride(kClearDiffCm); }

// Hands the differential to the water model, which drops it after `sweeps`.
void blockUntilSweeps(ControlHarness& h, uint8_t sweeps) {
  h.plant().configureWater(sweeps, kBlockedDiffCm, kClearDiffCm, kDownstreamCm);
  h.plant().clearDiffOverride();
}

// Drives a full blockage episode to FAULT 3 (still blocked after MAX_CYCLES).
void driveToFault3(ControlHarness& h) {
  bootToMonitoring(h);
  blockUntilSweeps(h, 99); // never clears
  TEST_ASSERT_TRUE(h.runUntilEvent(cfg::EVENT_FAULT,
                                   cfg::CONFIRM_MS +
                                       cfg::MAX_CYCLES * kCycleBudget));
  TEST_ASSERT_EQUAL_UINT8(cfg::FAULT_STILL_BLOCKED, h.ctrl().faultCode());
}

// Boots straight into a homing failure: rake parked at the end switch and
// unable to move.
void driveToFault4(ControlHarness& h) {
  h.plant().configureWater(1, kBlockedDiffCm, kClearDiffCm, kDownstreamCm);
  h.plant().setDiffOverride(kClearDiffCm);
  h.plant().placeAtEnd();
  h.plant().injectStallOnRaise(true);
  h.begin(false, 0.0f, 0);
  TEST_ASSERT_TRUE(
      h.runUntilEvent(cfg::EVENT_FAULT, cfg::HOMING_TIMEOUT_MS + 5000));
  TEST_ASSERT_EQUAL_UINT8(cfg::FAULT_HOMING_FAILED, h.ctrl().faultCode());
}

} // namespace

// C1 -----------------------------------------------------------------------
void test_c1_one_cycle_clear() {
  ControlHarness h(kMotion, kCtrl, kPlant, kDeadtime);
  bootToMonitoring(h);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, kClearDiffCm, h.ctrl().baselineCm());

  blockUntilSweeps(h, 1);

  TEST_ASSERT_TRUE(h.runUntilState(dg::State::CONFIRMING, 2000));
  TEST_ASSERT_TRUE(
      h.runUntilState(dg::State::CLEARING, cfg::CONFIRM_MS + 2000));
  TEST_ASSERT_TRUE(h.runUntilState(dg::State::SETTLING, kSweepMs + 5000));
  TEST_ASSERT_TRUE(h.runUntilEvent(cfg::EVENT_CLEARED_VERIFIED,
                                   cfg::SETTLE_MS + 5000));

  TEST_ASSERT_EQUAL_UINT8(cfg::FAULT_NONE, h.lastRecord().fault);
  TEST_ASSERT_EQUAL_UINT8(cfg::STATE_SETTLING, h.lastRecord().state);
  TEST_ASSERT_EQUAL_UINT16(1, h.lastRecord().cycles);
  TEST_ASSERT_EQUAL(dg::State::MONITORING, h.ctrl().state());
  TEST_ASSERT_EQUAL_UINT8(cfg::FAULT_NONE, h.ctrl().faultCode());
  TEST_ASSERT_EQUAL_UINT8(0, h.ctrl().blockageCycles());
  TEST_ASSERT_EQUAL_UINT16(1, h.ctrl().cyclesSinceEmptied());
}

// C2 -----------------------------------------------------------------------
void test_c2_two_cycle_clear() {
  ControlHarness h(kMotion, kCtrl, kPlant, kDeadtime);
  bootToMonitoring(h);
  blockUntilSweeps(h, 2); // the first sweep is not enough

  TEST_ASSERT_TRUE(h.runUntilEvent(cfg::EVENT_CLEARED_VERIFIED,
                                   cfg::CONFIRM_MS + 2 * kCycleBudget));
  TEST_ASSERT_EQUAL_UINT16(2, h.ctrl().cyclesSinceEmptied());
  TEST_ASSERT_EQUAL(dg::State::MONITORING, h.ctrl().state());
  TEST_ASSERT_EQUAL_UINT8(cfg::FAULT_NONE, h.ctrl().faultCode());
}

// C3 -----------------------------------------------------------------------
void test_c3_three_cycles_still_blocked() {
  ControlHarness h(kMotion, kCtrl, kPlant, kDeadtime);
  driveToFault3(h);

  TEST_ASSERT_EQUAL(dg::State::FAULT, h.ctrl().state());
  TEST_ASSERT_EQUAL_UINT8(cfg::EVENT_FAULT, h.lastRecord().event);
  TEST_ASSERT_EQUAL_UINT8(cfg::FAULT_STILL_BLOCKED, h.lastRecord().fault);
  TEST_ASSERT_EQUAL_UINT8(cfg::STATE_FAULT, h.lastRecord().state);
  // Three sweeps were performed before giving up.
  TEST_ASSERT_EQUAL_UINT16(cfg::MAX_CYCLES, h.ctrl().cyclesSinceEmptied());
}

// C4 -----------------------------------------------------------------------
void test_c4_false_alarm_in_confirming() {
  ControlHarness h(kMotion, kCtrl, kPlant, kDeadtime);
  bootToMonitoring(h);

  block(h);
  TEST_ASSERT_TRUE(h.runUntilState(dg::State::CONFIRMING, 2000));

  h.runMs(cfg::CONFIRM_MS / 2); // halfway through the confirm window
  unblock(h);
  TEST_ASSERT_TRUE(h.runUntilState(dg::State::MONITORING, 2000));

  // Sit a while: no clearing, no events, and the rake never moved.
  h.runMs(5000);
  TEST_ASSERT_EQUAL(dg::State::MONITORING, h.ctrl().state());
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, h.plant().posFrac());
  TEST_ASSERT_EQUAL(dg::MotorCmd::STOP, h.lastCmd());
  for (uint16_t i = 0; i < h.recordsStored(); ++i) {
    TEST_ASSERT_EQUAL_UINT8(cfg::EVENT_SNAPSHOT, h.recordAt(i).event);
  }
}

// C5 -----------------------------------------------------------------------
void test_c5_raise_jam_cleared_by_retry() {
  ControlHarness h(kMotion, kCtrl, kPlant, kDeadtime);
  bootToMonitoring(h);
  blockUntilSweeps(h, 1);
  h.plant().injectJamOnRaise(true);

  TEST_ASSERT_TRUE(
      h.runUntilState(dg::State::CLEARING, cfg::CONFIRM_MS + 3000));
  // Let the jam be detected and the recovery start, then free the rake.
  h.runMs(kStrokeMs + kDeadtime + kJamDetectMs + kDeadtime +
          cfg::JAM_REVERSE_MS + 200);
  h.plant().injectJamOnRaise(false);

  TEST_ASSERT_TRUE(
      h.runUntilEvent(cfg::EVENT_JAM_RETRY_CLEARED, kSweepMs + 10000));
  TEST_ASSERT_EQUAL_UINT8(cfg::FAULT_JAM_CLEARED_BY_RETRY,
                          h.lastRecord().fault);
  TEST_ASSERT_EQUAL_UINT8(cfg::STATE_CLEARING, h.lastRecord().state);
  TEST_ASSERT_EQUAL_UINT16(1, h.lastRecord().cycles);
  TEST_ASSERT_EQUAL_UINT8(cfg::FAULT_JAM_CLEARED_BY_RETRY,
                          h.ctrl().faultCode());

  // The episode still ends cleanly, and the informational fault clears.
  TEST_ASSERT_TRUE(h.runUntilEvent(cfg::EVENT_CLEARED_VERIFIED,
                                   cfg::SETTLE_MS + 5000));
  TEST_ASSERT_EQUAL_UINT8(cfg::FAULT_NONE, h.lastRecord().fault);
  TEST_ASSERT_EQUAL_UINT8(cfg::FAULT_NONE, h.ctrl().faultCode());
}

// C6 -----------------------------------------------------------------------
void test_c6_three_jams_fault2() {
  ControlHarness h(kMotion, kCtrl, kPlant, kDeadtime);
  bootToMonitoring(h);
  blockUntilSweeps(h, 1);
  h.plant().injectJamOnRaise(true); // never clears

  TEST_ASSERT_TRUE(h.runUntilEvent(cfg::EVENT_FAULT,
                                   cfg::CONFIRM_MS + 3 * kCycleBudget));
  TEST_ASSERT_EQUAL(dg::State::FAULT, h.ctrl().state());
  TEST_ASSERT_EQUAL_UINT8(cfg::FAULT_JAM_NOT_CLEARED, h.lastRecord().fault);
  TEST_ASSERT_EQUAL_UINT8(cfg::STATE_FAULT, h.lastRecord().state);
  // A failed sweep deposits nothing, so the bin counter never moved (D35).
  TEST_ASSERT_EQUAL_UINT16(0, h.ctrl().cyclesSinceEmptied());
}

// C7 -----------------------------------------------------------------------
void test_c7_down_stall_retry_then_settling() {
  ControlHarness h(kMotion, kCtrl, kPlant, kDeadtime);
  bootToMonitoring(h);
  blockUntilSweeps(h, 1);
  h.plant().injectStallOnLower(true);

  TEST_ASSERT_TRUE(
      h.runUntilState(dg::State::CLEARING, cfg::CONFIRM_MS + 3000));
  h.runMs(cfg::TRAVEL_TIMEOUT_MS + 1000); // timeout fires, recovery starts
  TEST_ASSERT_EQUAL(dg::State::CLEARING, h.ctrl().state());
  h.plant().injectStallOnLower(false);

  TEST_ASSERT_TRUE(h.runUntilState(dg::State::SETTLING,
                                   cfg::TRAVEL_TIMEOUT_MS + kSweepMs + 10000));
  TEST_ASSERT_EQUAL_UINT16(1, h.ctrl().cyclesSinceEmptied());
}

// C8 -----------------------------------------------------------------------
void test_c8_homing_timeout_fault4() {
  ControlHarness h(kMotion, kCtrl, kPlant, kDeadtime);
  driveToFault4(h);

  TEST_ASSERT_EQUAL(dg::State::FAULT, h.ctrl().state());
  TEST_ASSERT_EQUAL_UINT8(cfg::EVENT_FAULT, h.lastRecord().event);
  TEST_ASSERT_EQUAL_UINT8(cfg::FAULT_HOMING_FAILED, h.lastRecord().fault);
  TEST_ASSERT_EQUAL_UINT8(cfg::STATE_FAULT, h.lastRecord().state);
  TEST_ASSERT_EQUAL(dg::MotorCmd::STOP, h.lastCmd());
}

// C9 -----------------------------------------------------------------------
void test_c9_fault3_cleared_by_hold() {
  ControlHarness h(kMotion, kCtrl, kPlant, kDeadtime);
  driveToFault3(h);
  const float baselineBefore = h.ctrl().baselineCm();

  h.pressHold();
  TEST_ASSERT_TRUE(h.runUntilState(dg::State::HOMING, 1000));
  TEST_ASSERT_TRUE(
      h.runUntilEvent(cfg::EVENT_FAULT_RECOVERED, cfg::HOMING_TIMEOUT_MS));

  TEST_ASSERT_EQUAL(dg::State::MONITORING, h.ctrl().state()); // D34, baseline kept
  TEST_ASSERT_EQUAL_UINT8(cfg::FAULT_NONE, h.ctrl().faultCode());
  TEST_ASSERT_EQUAL_UINT8(0, h.ctrl().blockageCycles());
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, baselineBefore, h.ctrl().baselineCm());
}

// C10 ----------------------------------------------------------------------
void test_c10_fault3_auto_recovers() {
  ControlHarness h(kMotion, kCtrl, kPlant, kDeadtime);
  driveToFault3(h);

  unblock(h); // differential back at baseline, no button press
  TEST_ASSERT_TRUE(h.runUntilEvent(cfg::EVENT_FAULT_RECOVERED,
                                   cfg::FAULT_AUTORECOVER_MS +
                                       cfg::HOMING_TIMEOUT_MS + 5000));
  TEST_ASSERT_EQUAL(dg::State::MONITORING, h.ctrl().state());
  TEST_ASSERT_EQUAL_UINT8(cfg::FAULT_NONE, h.ctrl().faultCode());
}

// C11 ----------------------------------------------------------------------
void test_c11_fault2_does_not_auto_recover() {
  ControlHarness h(kMotion, kCtrl, kPlant, kDeadtime);
  bootToMonitoring(h);
  blockUntilSweeps(h, 1);
  h.plant().injectJamOnRaise(true);
  TEST_ASSERT_TRUE(h.runUntilEvent(cfg::EVENT_FAULT,
                                   cfg::CONFIRM_MS + 3 * kCycleBudget));
  TEST_ASSERT_EQUAL_UINT8(cfg::FAULT_JAM_NOT_CLEARED, h.ctrl().faultCode());

  // Clear water for well over the auto-recovery window: fault 2 must hold (D33).
  unblock(h);
  h.runMs(cfg::FAULT_AUTORECOVER_MS + 10000);
  TEST_ASSERT_EQUAL(dg::State::FAULT, h.ctrl().state());
  TEST_ASSERT_EQUAL_UINT8(cfg::FAULT_JAM_NOT_CLEARED, h.ctrl().faultCode());

  // Only the operator clears it.
  h.plant().injectJamOnRaise(false);
  h.pressHold();
  TEST_ASSERT_TRUE(h.runUntilState(dg::State::HOMING, 1000));
  TEST_ASSERT_TRUE(
      h.runUntilEvent(cfg::EVENT_FAULT_RECOVERED, cfg::HOMING_TIMEOUT_MS));
  TEST_ASSERT_EQUAL_UINT8(cfg::FAULT_NONE, h.ctrl().faultCode());
}

// C12 ----------------------------------------------------------------------
void test_c12_fault4_first_boot_rehomes_to_baseline() {
  ControlHarness h(kMotion, kCtrl, kPlant, kDeadtime);
  driveToFault4(h);
  TEST_ASSERT_FALSE(h.ctrl().haveBaseline()); // nothing learned yet

  h.plant().injectStallOnRaise(false);
  h.pressHold();
  TEST_ASSERT_TRUE(h.runUntilState(dg::State::HOMING, 1000));
  TEST_ASSERT_TRUE(
      h.runUntilEvent(cfg::EVENT_FAULT_RECOVERED, cfg::HOMING_TIMEOUT_MS));

  // No baseline exists, so the re-home lands in BASELINE, not MONITORING (D34).
  TEST_ASSERT_EQUAL(dg::State::BASELINE, h.ctrl().state());
  TEST_ASSERT_EQUAL_UINT8(cfg::FAULT_NONE, h.ctrl().faultCode());
}

// C13 ----------------------------------------------------------------------
void test_c13_short_press_bin_emptied() {
  ControlHarness h(kMotion, kCtrl, kPlant, kDeadtime);
  bootToMonitoring(h, false, 0.0f, 7); // 7 cycles carried over from NVS
  TEST_ASSERT_EQUAL_UINT16(7, h.ctrl().cyclesSinceEmptied());

  h.pressShort();
  TEST_ASSERT_TRUE(h.runUntilEvent(cfg::EVENT_BIN_EMPTIED, 1000));

  TEST_ASSERT_EQUAL_UINT16(0, h.lastRecord().cycles);
  TEST_ASSERT_EQUAL_UINT16(0, h.ctrl().cyclesSinceEmptied());
  TEST_ASSERT_TRUE(h.out().saveCyclesSinceEmptied);
  TEST_ASSERT_EQUAL_UINT16(0, h.out().cyclesSinceEmptied);
}

// C14 ----------------------------------------------------------------------
void test_c14_button_scope() {
  // A short press is accepted in FAULT and leaves the fault standing (D37).
  ControlHarness h(kMotion, kCtrl, kPlant, kDeadtime);
  driveToFault3(h);
  const uint8_t faultBefore = h.ctrl().faultCode();

  h.pressShort();
  TEST_ASSERT_TRUE(h.runUntilEvent(cfg::EVENT_BIN_EMPTIED, 1000));
  TEST_ASSERT_EQUAL_UINT16(0, h.ctrl().cyclesSinceEmptied());
  TEST_ASSERT_EQUAL_UINT8(faultBefore, h.ctrl().faultCode());
  TEST_ASSERT_EQUAL(dg::State::FAULT, h.ctrl().state());

  // A hold outside FAULT does nothing at all.
  ControlHarness h2(kMotion, kCtrl, kPlant, kDeadtime);
  bootToMonitoring(h2);
  h2.pressHold();
  h2.runMs(2000);
  TEST_ASSERT_EQUAL(dg::State::MONITORING, h2.ctrl().state());
  for (uint16_t i = 0; i < h2.recordsStored(); ++i) {
    TEST_ASSERT_EQUAL_UINT8(cfg::EVENT_SNAPSHOT, h2.recordAt(i).event);
  }
}

// C15 ----------------------------------------------------------------------
void test_c15_baseline_rejected() {
  ControlHarness h(kMotion, kCtrl, kPlant, kDeadtime);
  h.plant().configureWater(1, kBlockedDiffCm, kClearDiffCm, kDownstreamCm);
  h.plant().setDiffOverride(8.0f); // measures 8.0 against a saved 4.0
  h.begin(true, 4.0f, 0);

  TEST_ASSERT_TRUE(
      h.runUntilEvent(cfg::EVENT_BASELINE_REJECTED, kBootBudget));
  TEST_ASSERT_EQUAL_UINT8(cfg::STATE_BASELINE, h.lastRecord().state);
  TEST_ASSERT_FALSE(h.out().saveBaseline); // nothing new worth storing
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 4.0f, h.ctrl().baselineCm());
  TEST_ASSERT_TRUE(h.ctrl().haveBaseline());
}

// C16 ----------------------------------------------------------------------
void test_c16_first_boot_baseline_saved() {
  ControlHarness h(kMotion, kCtrl, kPlant, kDeadtime);
  h.plant().configureWater(1, kBlockedDiffCm, kClearDiffCm, kDownstreamCm);
  h.plant().setDiffOverride(kClearDiffCm);
  h.begin(false, 0.0f, 0); // first boot: accept whatever we measure

  TEST_ASSERT_TRUE(h.runUntilState(dg::State::MONITORING, kBootBudget));
  TEST_ASSERT_TRUE(h.out().saveBaseline);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, kClearDiffCm, h.out().baselineCm);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, kClearDiffCm, h.ctrl().baselineCm());
  TEST_ASSERT_TRUE(h.ctrl().haveBaseline());
}

// C17 ----------------------------------------------------------------------
void test_c17_cycles_since_emptied() {
  // Two completed sweeps, two increments.
  ControlHarness h(kMotion, kCtrl, kPlant, kDeadtime);
  bootToMonitoring(h);
  blockUntilSweeps(h, 2);
  TEST_ASSERT_TRUE(h.runUntilState(dg::State::SETTLING, kCycleBudget));
  TEST_ASSERT_EQUAL_UINT16(1, h.ctrl().cyclesSinceEmptied());
  TEST_ASSERT_TRUE(h.runUntilEvent(cfg::EVENT_CLEARED_VERIFIED,
                                   2 * kCycleBudget));
  TEST_ASSERT_EQUAL_UINT16(2, h.ctrl().cyclesSinceEmptied());

  // A sweep that fails leaves the counter alone.
  ControlHarness h2(kMotion, kCtrl, kPlant, kDeadtime);
  bootToMonitoring(h2, false, 0.0f, 4);
  blockUntilSweeps(h2, 1);
  h2.plant().injectJamOnRaise(true);
  TEST_ASSERT_TRUE(h2.runUntilEvent(cfg::EVENT_FAULT,
                                    cfg::CONFIRM_MS + 3 * kCycleBudget));
  TEST_ASSERT_EQUAL_UINT8(cfg::FAULT_JAM_NOT_CLEARED, h2.ctrl().faultCode());
  TEST_ASSERT_EQUAL_UINT16(4, h2.ctrl().cyclesSinceEmptied());
}

// C18 ----------------------------------------------------------------------
void test_c18_snapshot_in_every_state() {
  // BASELINE: the first snapshot at 20 s lands inside the 30 s window.
  {
    ControlHarness h(kMotion, kCtrl, kPlant, kDeadtime);
    h.plant().configureWater(1, kBlockedDiffCm, kClearDiffCm, kDownstreamCm);
    h.plant().setDiffOverride(kClearDiffCm);
    h.begin(false, 0.0f, 0);
    TEST_ASSERT_TRUE(h.runUntilRecord(cfg::SNAPSHOT_MS + 2000));
    TEST_ASSERT_EQUAL_UINT8(cfg::EVENT_SNAPSHOT, h.lastRecord().event);
    TEST_ASSERT_EQUAL_UINT8(cfg::STATE_BASELINE, h.lastRecord().state);
  }

  // MONITORING: just sit there.
  {
    ControlHarness h(kMotion, kCtrl, kPlant, kDeadtime);
    bootToMonitoring(h);
    TEST_ASSERT_TRUE(h.runUntilRecord(cfg::SNAPSHOT_MS + 2000));
    TEST_ASSERT_EQUAL_UINT8(cfg::EVENT_SNAPSHOT, h.lastRecord().event);
    TEST_ASSERT_EQUAL_UINT8(cfg::STATE_MONITORING, h.lastRecord().state);
  }

  // CONFIRMING: trigger late enough that the snapshot falls inside the 10 s.
  {
    ControlHarness h(kMotion, kCtrl, kPlant, kDeadtime);
    bootToMonitoring(h);
    TEST_ASSERT_TRUE(h.runUntilSince(12000, cfg::SNAPSHOT_MS));
    block(h);
    TEST_ASSERT_TRUE(h.runUntilRecord(cfg::SNAPSHOT_MS));
    TEST_ASSERT_EQUAL_UINT8(cfg::EVENT_SNAPSHOT, h.lastRecord().event);
    TEST_ASSERT_EQUAL_UINT8(cfg::STATE_CONFIRMING, h.lastRecord().state);
  }

  // CLEARING: a stalled down stroke keeps the state alive for a long time.
  {
    ControlHarness h(kMotion, kCtrl, kPlant, kDeadtime);
    bootToMonitoring(h);
    blockUntilSweeps(h, 99);
    h.plant().injectStallOnLower(true);
    TEST_ASSERT_TRUE(
        h.runUntilState(dg::State::CLEARING, cfg::CONFIRM_MS + 3000));
    h.clearRecords();
    TEST_ASSERT_TRUE(h.runUntilRecord(cfg::SNAPSHOT_MS + 2000));
    TEST_ASSERT_EQUAL_UINT8(cfg::EVENT_SNAPSHOT, h.lastRecord().event);
    TEST_ASSERT_EQUAL_UINT8(cfg::STATE_CLEARING, h.lastRecord().state);
  }

  // SETTLING: a dead sensor holds the verdict, so the state persists.
  {
    ControlHarness h(kMotion, kCtrl, kPlant, kDeadtime);
    bootToMonitoring(h);
    blockUntilSweeps(h, 99);
    TEST_ASSERT_TRUE(h.runUntilState(dg::State::SETTLING, kCycleBudget));
    h.setSensorsValid(false, true);
    h.clearRecords();
    TEST_ASSERT_TRUE(h.runUntilRecord(cfg::SNAPSHOT_MS + 2000));
    TEST_ASSERT_EQUAL_UINT8(cfg::EVENT_SNAPSHOT, h.lastRecord().event);
    TEST_ASSERT_EQUAL_UINT8(cfg::STATE_SETTLING, h.lastRecord().state);
  }

  // FAULT: sits until an operator or auto-recovery intervenes.
  {
    ControlHarness h(kMotion, kCtrl, kPlant, kDeadtime);
    driveToFault4(h);
    h.clearRecords();
    TEST_ASSERT_TRUE(h.runUntilRecord(cfg::SNAPSHOT_MS + 2000));
    TEST_ASSERT_EQUAL_UINT8(cfg::EVENT_SNAPSHOT, h.lastRecord().event);
    TEST_ASSERT_EQUAL_UINT8(cfg::STATE_FAULT, h.lastRecord().state);
  }

  // HOMING: start the re-home just before a snapshot comes due, with the rake
  // far enough from home that homing is still running when it fires.
  {
    ControlHarness h(kMotion, kCtrl, kPlant, kDeadtime);
    driveToFault4(h);
    h.plant().injectStallOnRaise(false);
    h.plant().placeAtEnd();
    TEST_ASSERT_TRUE(h.runUntilSince(cfg::SNAPSHOT_MS - 1500, cfg::SNAPSHOT_MS));
    h.pressHold();
    TEST_ASSERT_TRUE(h.runUntilState(dg::State::HOMING, 1000));
    TEST_ASSERT_TRUE(h.runUntilRecord(3000));
    TEST_ASSERT_EQUAL_UINT8(cfg::EVENT_SNAPSHOT, h.lastRecord().event);
    TEST_ASSERT_EQUAL_UINT8(cfg::STATE_HOMING, h.lastRecord().state);
  }
}

// C19 ----------------------------------------------------------------------
void test_c19_event_resets_snapshot_timer() {
  ControlHarness h(kMotion, kCtrl, kPlant, kDeadtime);
  bootToMonitoring(h);

  // Let the snapshot clock run most of the way, then fire an event.
  TEST_ASSERT_TRUE(h.runUntilSince(15000, cfg::SNAPSHOT_MS));
  h.pressShort();
  TEST_ASSERT_TRUE(h.runUntilEvent(cfg::EVENT_BIN_EMPTIED, 1000));

  const uint32_t eventMs = h.nowMs();
  h.clearRecords();

  // Nothing for nearly a full interval, proving the clock restarted.
  h.runMs(cfg::SNAPSHOT_MS - 2000);
  TEST_ASSERT_EQUAL_UINT16(0, h.recordCount());

  TEST_ASSERT_TRUE(h.runUntilRecord(4000));
  TEST_ASSERT_EQUAL_UINT8(cfg::EVENT_SNAPSHOT, h.lastRecord().event);
  TEST_ASSERT_UINT32_WITHIN(2 * cfg::CONTROL_TICK_MS, cfg::SNAPSHOT_MS,
                            h.nowMs() - eventMs);
}

// C20 ----------------------------------------------------------------------
void test_c20_two_events_same_tick() {
  ControlHarness h(kMotion, kCtrl, kPlant, kDeadtime);
  bootToMonitoring(h);
  blockUntilSweeps(h, 1);
  TEST_ASSERT_TRUE(h.runUntilState(dg::State::SETTLING, kCycleBudget));

  // Step to the tick immediately before the verdict, then add a bin press so
  // both events are produced by the same controller tick.
  while ((h.lastControllerMs() + cfg::CONTROL_TICK_MS -
          h.ctrl().stateEnteredMs()) < cfg::SETTLE_MS) {
    h.controllerTick();
    TEST_ASSERT_EQUAL(dg::State::SETTLING, h.ctrl().state());
  }
  h.clearRecords();
  h.pressShort();

  h.controllerTick();
  TEST_ASSERT_TRUE(h.justRecorded());
  TEST_ASSERT_EQUAL_UINT8(cfg::EVENT_BIN_EMPTIED, h.lastRecord().event);
  TEST_ASSERT_EQUAL(dg::State::MONITORING, h.ctrl().state()); // verdict taken too

  h.controllerTick();
  TEST_ASSERT_TRUE(h.justRecorded());
  TEST_ASSERT_EQUAL_UINT8(cfg::EVENT_CLEARED_VERIFIED, h.lastRecord().event);

  // Exactly two records, both events, consecutive, no snapshot wedged between.
  TEST_ASSERT_EQUAL_UINT16(2, h.recordsStored());
  TEST_ASSERT_EQUAL_UINT8(0, h.ctrl().pendingEvents());
  TEST_ASSERT_EQUAL_UINT16(0, h.ctrl().eventOverflowCount());

  // And an event always beats an overdue snapshot.
  ControlHarness h2(kMotion, kCtrl, kPlant, kDeadtime);
  driveToFault4(h2);
  TEST_ASSERT_TRUE(h2.runUntilSince(cfg::SNAPSHOT_MS - cfg::CONTROL_TICK_MS,
                                    cfg::SNAPSHOT_MS));
  h2.clearRecords();
  h2.pressShort();
  TEST_ASSERT_TRUE(h2.runUntilRecord(1000));
  TEST_ASSERT_EQUAL_UINT8(cfg::EVENT_BIN_EMPTIED, h2.lastRecord().event);
}

// C21 ----------------------------------------------------------------------
void test_c21_invalid_in_monitoring() {
  ControlHarness h(kMotion, kCtrl, kPlant, kDeadtime);
  bootToMonitoring(h);

  // Blocked channel but a dead upstream sensor: clearing must not start (D25).
  block(h);
  h.setSensorsValid(false, true);

  const uint32_t start = h.nowMs();
  while ((h.nowMs() - start) < cfg::SENSOR_FAULT_MS - 2000) {
    h.step();
    TEST_ASSERT_EQUAL(dg::State::MONITORING, h.ctrl().state());
  }

  TEST_ASSERT_TRUE(h.runUntilEvent(cfg::EVENT_FAULT, 5000));
  TEST_ASSERT_EQUAL(dg::State::FAULT, h.ctrl().state());
  TEST_ASSERT_EQUAL_UINT8(cfg::FAULT_SENSOR, h.ctrl().faultCode());
  TEST_ASSERT_UINT32_WITHIN(2 * cfg::CONTROL_TICK_MS, cfg::SENSOR_FAULT_MS,
                            h.nowMs() - start);
}

// C22 ----------------------------------------------------------------------
void test_c22_invalid_in_confirming() {
  ControlHarness h(kMotion, kCtrl, kPlant, kDeadtime);
  bootToMonitoring(h);
  block(h);
  TEST_ASSERT_TRUE(h.runUntilState(dg::State::CONFIRMING, 2000));

  h.runMs(2000);
  h.setSensorsValid(true, false); // downstream drops out
  TEST_ASSERT_TRUE(h.runUntilState(dg::State::MONITORING, 1000)); // abort (D41)

  for (uint16_t i = 0; i < h.recordsStored(); ++i) {
    TEST_ASSERT_EQUAL_UINT8(cfg::EVENT_SNAPSHOT, h.recordAt(i).event);
  }
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, h.plant().posFrac()); // never swept
}

// C23 ----------------------------------------------------------------------
void test_c23_invalid_at_settling_verdict() {
  // Held at the verdict, then faulted at 60 s.
  {
    ControlHarness h(kMotion, kCtrl, kPlant, kDeadtime);
    bootToMonitoring(h);
    blockUntilSweeps(h, 1);
    TEST_ASSERT_TRUE(h.runUntilState(dg::State::SETTLING, kCycleBudget));

    h.setSensorsValid(false, false);
    const uint32_t badAt = h.nowMs();

    h.runMs(cfg::SETTLE_MS + 5000); // well past the verdict moment
    TEST_ASSERT_EQUAL(dg::State::SETTLING, h.ctrl().state());

    TEST_ASSERT_TRUE(h.runUntilEvent(cfg::EVENT_FAULT, cfg::SENSOR_FAULT_MS));
    TEST_ASSERT_EQUAL_UINT8(cfg::FAULT_SENSOR, h.ctrl().faultCode());
    TEST_ASSERT_UINT32_WITHIN(2 * cfg::CONTROL_TICK_MS, cfg::SENSOR_FAULT_MS,
                              h.nowMs() - badAt);
  }

  // Sensors back before 60 s: the held verdict runs as normal.
  {
    ControlHarness h(kMotion, kCtrl, kPlant, kDeadtime);
    bootToMonitoring(h);
    blockUntilSweeps(h, 1);
    TEST_ASSERT_TRUE(h.runUntilState(dg::State::SETTLING, kCycleBudget));

    h.setSensorsValid(false, false);
    h.runMs(cfg::SETTLE_MS + 5000);
    TEST_ASSERT_EQUAL(dg::State::SETTLING, h.ctrl().state());

    h.setSensorsValid(true, true);
    TEST_ASSERT_TRUE(h.runUntilEvent(cfg::EVENT_CLEARED_VERIFIED, 5000));
    TEST_ASSERT_EQUAL(dg::State::MONITORING, h.ctrl().state());
    TEST_ASSERT_EQUAL_UINT8(cfg::FAULT_NONE, h.ctrl().faultCode());
  }
}

// C24 ----------------------------------------------------------------------
void test_c24_invalid_through_baseline() {
  ControlHarness h(kMotion, kCtrl, kPlant, kDeadtime);
  h.plant().configureWater(1, kBlockedDiffCm, kClearDiffCm, kDownstreamCm);
  h.plant().setDiffOverride(kClearDiffCm);
  h.setSensorsValid(false, false);
  h.begin(false, 0.0f, 0);

  TEST_ASSERT_TRUE(h.runUntilEvent(cfg::EVENT_FAULT,
                                   cfg::SENSOR_FAULT_MS + 5000));
  TEST_ASSERT_EQUAL(dg::State::FAULT, h.ctrl().state());
  TEST_ASSERT_EQUAL_UINT8(cfg::FAULT_SENSOR, h.ctrl().faultCode());
  TEST_ASSERT_FALSE(h.ctrl().haveBaseline()); // never managed to learn one
  TEST_ASSERT_UINT32_WITHIN(2 * cfg::CONTROL_TICK_MS, cfg::SENSOR_FAULT_MS,
                            h.nowMs());
}

// C25 ----------------------------------------------------------------------
void test_c25_fault5_auto_recovers() {
  ControlHarness h(kMotion, kCtrl, kPlant, kDeadtime);
  bootToMonitoring(h); // gives us a baseline to compare against

  h.setSensorsValid(false, true);
  TEST_ASSERT_TRUE(h.runUntilEvent(cfg::EVENT_FAULT,
                                   cfg::SENSOR_FAULT_MS + 5000));
  TEST_ASSERT_EQUAL_UINT8(cfg::FAULT_SENSOR, h.ctrl().faultCode());

  // Sensors return and the channel reads clear: fault 5 clears itself (D33).
  h.setSensorsValid(true, true);
  unblock(h);
  TEST_ASSERT_TRUE(h.runUntilEvent(cfg::EVENT_FAULT_RECOVERED,
                                   cfg::FAULT_AUTORECOVER_MS +
                                       cfg::HOMING_TIMEOUT_MS + 5000));
  TEST_ASSERT_EQUAL(dg::State::MONITORING, h.ctrl().state());
  TEST_ASSERT_EQUAL_UINT8(cfg::FAULT_NONE, h.ctrl().faultCode());
}

// C26 ----------------------------------------------------------------------
void test_c26_led_table() {
  // HOMING: yellow on, because the motor is moving (D38).
  {
    ControlHarness h(kMotion, kCtrl, kPlant, kDeadtime);
    h.plant().configureWater(1, kBlockedDiffCm, kClearDiffCm, kDownstreamCm);
    h.plant().setDiffOverride(kClearDiffCm);
    h.plant().placeAtEnd();
    h.begin(false, 0.0f, 0);
    h.controllerTick();
    TEST_ASSERT_EQUAL(dg::State::HOMING, h.ctrl().state());
    TEST_ASSERT_TRUE(h.out().ledStatus);
    TEST_ASSERT_FALSE(h.out().ledFault);
    TEST_ASSERT_TRUE(h.out().ledClearing);
  }

  // BASELINE / MONITORING / CONFIRMING / SETTLING: green only.
  {
    ControlHarness h(kMotion, kCtrl, kPlant, kDeadtime);
    h.plant().configureWater(1, kBlockedDiffCm, kClearDiffCm, kDownstreamCm);
    h.plant().setDiffOverride(kClearDiffCm);
    h.begin(false, 0.0f, 0);
    TEST_ASSERT_TRUE(h.runUntilState(dg::State::BASELINE, 2000));
    TEST_ASSERT_TRUE(h.out().ledStatus);
    TEST_ASSERT_FALSE(h.out().ledFault);
    TEST_ASSERT_FALSE(h.out().ledClearing);

    TEST_ASSERT_TRUE(h.runUntilState(dg::State::MONITORING, kBootBudget));
    TEST_ASSERT_TRUE(h.out().ledStatus);
    TEST_ASSERT_FALSE(h.out().ledFault);
    TEST_ASSERT_FALSE(h.out().ledClearing);

    blockUntilSweeps(h, 1);
    TEST_ASSERT_TRUE(h.runUntilState(dg::State::CONFIRMING, 2000));
    TEST_ASSERT_TRUE(h.out().ledStatus);
    TEST_ASSERT_FALSE(h.out().ledFault);
    TEST_ASSERT_FALSE(h.out().ledClearing);

    // CLEARING: yellow on.
    TEST_ASSERT_TRUE(
        h.runUntilState(dg::State::CLEARING, cfg::CONFIRM_MS + 2000));
    TEST_ASSERT_TRUE(h.out().ledStatus);
    TEST_ASSERT_FALSE(h.out().ledFault);
    TEST_ASSERT_TRUE(h.out().ledClearing);

    TEST_ASSERT_TRUE(h.runUntilState(dg::State::SETTLING, kSweepMs + 5000));
    TEST_ASSERT_TRUE(h.out().ledStatus);
    TEST_ASSERT_FALSE(h.out().ledFault);
    TEST_ASSERT_FALSE(h.out().ledClearing);
  }

  // FAULT: red only, green dark.
  {
    ControlHarness h(kMotion, kCtrl, kPlant, kDeadtime);
    driveToFault4(h);
    TEST_ASSERT_FALSE(h.out().ledStatus);
    TEST_ASSERT_TRUE(h.out().ledFault);
    TEST_ASSERT_FALSE(h.out().ledClearing);
  }
}

// C27 ----------------------------------------------------------------------
void test_c27_fault_stops_motor() {
  // A jam fault asks for STOP on the tick it latches.
  {
    ControlHarness h(kMotion, kCtrl, kPlant, kDeadtime);
    bootToMonitoring(h);
    blockUntilSweeps(h, 1);
    h.plant().injectJamOnRaise(true);
    TEST_ASSERT_TRUE(h.runUntilEvent(cfg::EVENT_FAULT,
                                     cfg::CONFIRM_MS + 3 * kCycleBudget));
    TEST_ASSERT_EQUAL(dg::Job::STOP, h.out().motionJob);
    TEST_ASSERT_EQUAL(dg::MotorCmd::STOP, h.lastCmd());
  }

  // And with the motor genuinely running, FAULT kills it on the same tick.
  {
    ControlHarness h(kMotion, kCtrl, kPlant, kDeadtime);
    bootToMonitoring(h);
    h.setSensorsValid(false, true);
    h.runMs(cfg::SENSOR_FAULT_MS - 3000);
    TEST_ASSERT_EQUAL(dg::State::MONITORING, h.ctrl().state());

    // A stray job: the supervisor is BUSY and the rake is moving down.
    h.sup().requestJob(dg::Job::SWEEP, h.nowMs());
    h.runMs(1000);
    TEST_ASSERT_EQUAL(dg::MotorCmd::LOWER, h.lastCmd());

    TEST_ASSERT_TRUE(h.runUntilState(dg::State::FAULT, 10000));
    TEST_ASSERT_EQUAL_UINT8(cfg::FAULT_SENSOR, h.ctrl().faultCode());
    TEST_ASSERT_EQUAL(dg::Job::STOP, h.out().motionJob);
    TEST_ASSERT_EQUAL(dg::MotorCmd::STOP, h.lastCmd());
  }
}

// C28 ----------------------------------------------------------------------
void test_c28_record_contents() {
  ControlHarness h(kMotion, kCtrl, kPlant, kDeadtime);
  bootToMonitoring(h, false, 0.0f, 3);
  blockUntilSweeps(h, 1);
  h.plant().injectJamOnRaise(true);

  TEST_ASSERT_TRUE(
      h.runUntilState(dg::State::CLEARING, cfg::CONFIRM_MS + 3000));
  h.runMs(kStrokeMs + kDeadtime + kJamDetectMs + kDeadtime +
          cfg::JAM_REVERSE_MS + 200);
  h.plant().injectJamOnRaise(false);
  TEST_ASSERT_TRUE(h.runUntilEvent(cfg::EVENT_CLEARED_VERIFIED,
                                   cfg::SETTLE_MS + kSweepMs + 10000));
  h.pressShort();
  TEST_ASSERT_TRUE(h.runUntilEvent(cfg::EVENT_BIN_EMPTIED, 1000));

  TEST_ASSERT_TRUE(h.recordsStored() >= 3);

  bool sawRetryEvent = false;
  bool sawVerified = false;
  bool sawBin = false;
  for (uint16_t i = 0; i < h.recordsStored(); ++i) {
    const dg::Record& r = h.recordAt(i);

    // Every code stays inside the wire protocol.
    TEST_ASSERT_TRUE(r.state <= cfg::STATE_BASELINE);
    TEST_ASSERT_TRUE(r.fault <= cfg::FAULT_SENSOR);
    TEST_ASSERT_TRUE(r.event <= cfg::EVENT_JAM_RETRY_CLEARED);
    // Field 3 is always the difference of fields 1 and 2.
    TEST_ASSERT_FLOAT_WITHIN(0.001f, r.uCm - r.dCm, r.diffCm);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, kDownstreamCm, r.dCm);

    if (r.event == cfg::EVENT_JAM_RETRY_CLEARED) {
      sawRetryEvent = true;
      TEST_ASSERT_EQUAL_UINT8(cfg::FAULT_JAM_CLEARED_BY_RETRY, r.fault);
      TEST_ASSERT_EQUAL_UINT8(cfg::STATE_CLEARING, r.state);
      TEST_ASSERT_EQUAL_UINT16(4, r.cycles); // 3 carried over plus this sweep
    } else if (r.event == cfg::EVENT_CLEARED_VERIFIED) {
      sawVerified = true;
      TEST_ASSERT_EQUAL_UINT8(cfg::FAULT_NONE, r.fault);
      TEST_ASSERT_EQUAL_UINT16(4, r.cycles);
    } else if (r.event == cfg::EVENT_BIN_EMPTIED) {
      sawBin = true;
      TEST_ASSERT_EQUAL_UINT16(0, r.cycles); // field 5 follows the counter
    }
  }
  TEST_ASSERT_TRUE(sawRetryEvent);
  TEST_ASSERT_TRUE(sawVerified);
  TEST_ASSERT_TRUE(sawBin);
}

} // namespace dgtest
