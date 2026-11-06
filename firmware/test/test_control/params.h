// Shared test parameters. Every tunable comes from config.h (D2), so the tests
// exercise the values the firmware will actually ship with.

#ifndef DG_TEST_PARAMS_H
#define DG_TEST_PARAMS_H

#include <stdint.h>

#include "config.h"
#include "dg_controller.h"
#include "dg_motion.h"
#include "plant.h"

// dg_core declares the wire protocol codes itself so the library stays
// self-contained. These assertions are the guard that it never drifts from
// config.h, which is the project's source of truth.
static_assert(static_cast<uint8_t>(dg::State::MONITORING) == cfg::STATE_MONITORING, "");
static_assert(static_cast<uint8_t>(dg::State::CONFIRMING) == cfg::STATE_CONFIRMING, "");
static_assert(static_cast<uint8_t>(dg::State::CLEARING) == cfg::STATE_CLEARING, "");
static_assert(static_cast<uint8_t>(dg::State::SETTLING) == cfg::STATE_SETTLING, "");
static_assert(static_cast<uint8_t>(dg::State::FAULT) == cfg::STATE_FAULT, "");
static_assert(static_cast<uint8_t>(dg::State::HOMING) == cfg::STATE_HOMING, "");
static_assert(static_cast<uint8_t>(dg::State::BASELINE) == cfg::STATE_BASELINE, "");

static_assert(dg::kFaultNone == cfg::FAULT_NONE, "");
static_assert(dg::kFaultJamClearedByRetry == cfg::FAULT_JAM_CLEARED_BY_RETRY, "");
static_assert(dg::kFaultJamNotCleared == cfg::FAULT_JAM_NOT_CLEARED, "");
static_assert(dg::kFaultStillBlocked == cfg::FAULT_STILL_BLOCKED, "");
static_assert(dg::kFaultHomingFailed == cfg::FAULT_HOMING_FAILED, "");
static_assert(dg::kFaultSensor == cfg::FAULT_SENSOR, "");

static_assert(dg::kEventSnapshot == cfg::EVENT_SNAPSHOT, "");
static_assert(dg::kEventClearedVerified == cfg::EVENT_CLEARED_VERIFIED, "");
static_assert(dg::kEventFault == cfg::EVENT_FAULT, "");
static_assert(dg::kEventFaultRecovered == cfg::EVENT_FAULT_RECOVERED, "");
static_assert(dg::kEventBinEmptied == cfg::EVENT_BIN_EMPTIED, "");
static_assert(dg::kEventBaselineRejected == cfg::EVENT_BASELINE_REJECTED, "");
static_assert(dg::kEventJamRetryCleared == cfg::EVENT_JAM_RETRY_CLEARED, "");

namespace dgtest {

inline constexpr dg::MotionParams kMotion = {
    cfg::I_RUN_A,
    cfg::JAM_FACTOR,
    cfg::JAM_MIN_DELTA_A,
    cfg::JAM_HOLD_MS,
    cfg::INRUSH_BLANK_MS,
    cfg::JAM_REVERSE_MS,
    cfg::JAM_PAUSE_MS,
    cfg::DEADTIME_BEFORE_REVERSE_MS,
    cfg::TRAVEL_TIMEOUT_MS,
    cfg::HOMING_TIMEOUT_MS,
    cfg::MAX_RETRIES_PER_CYCLE,
};

inline constexpr dg::ControllerParams kCtrl = {
    cfg::THRESHOLD_CM,
    cfg::VERIFY_MARGIN_CM,
    cfg::BASELINE_REJECT_CM,
    cfg::BASELINE_MS,
    cfg::CONFIRM_MS,
    cfg::SETTLE_MS,
    cfg::SNAPSHOT_MS,
    cfg::FAULT_AUTORECOVER_MS,
    cfg::SENSOR_FAULT_MS,
    cfg::MAX_CYCLES,
};

// jam threshold = max(1.5 * 0.5, 0.5 + 0.3) = 0.8 A, so iRaise is comfortably
// under it and iJam / iInrush are comfortably over.
inline constexpr PlantParams kPlant = {
    2000,  // travelMsDown
    2000,  // travelMsUp
    0.05f, // iIdleA
    0.10f, // iLowerA
    0.50f, // iRaiseA
    2.00f, // iJamA
    250,   // inrushMs
    2.50f, // iInrushA
};

constexpr uint32_t kTickMs = 10;
constexpr uint32_t kSlackMs = 200;

// One full stroke, plus the dead time between the two halves of a sweep.
constexpr uint32_t kStrokeMs = 2000;
constexpr uint32_t kSweepMs =
    kStrokeMs + cfg::DEADTIME_BEFORE_REVERSE_MS + kStrokeMs;
// Jam is reported inrush-blank + hold after a raise stroke starts.
constexpr uint32_t kJamDetectMs = cfg::INRUSH_BLANK_MS + cfg::JAM_HOLD_MS;
// Dead time + reverse nudge + pause, between a jam and the retry stroke.
constexpr uint32_t kRecoverMs = cfg::DEADTIME_BEFORE_REVERSE_MS +
                                cfg::JAM_REVERSE_MS + cfg::JAM_PAUSE_MS;

// Water column used by the controller tests. The baseline settles at
// kClearDiffCm; kBlockedDiffCm clears the 2 cm threshold on top of it, and
// kClearDiffCm sits under the 1 cm verify margin.
constexpr float kClearDiffCm = 0.5f;
constexpr float kBlockedDiffCm = 6.0f;
constexpr float kDownstreamCm = 10.0f;

} // namespace dgtest

#endif // DG_TEST_PARAMS_H
