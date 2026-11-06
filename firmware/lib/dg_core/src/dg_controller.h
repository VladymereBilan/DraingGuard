// Controller: owns the state machine, the baseline, the counters and the record
// stream. Ticked by the caller every CONTROL_TICK_MS (200 ms) with filtered
// sensor levels, latched button edges and the MotionSupervisor's status (D29).
//
// Pure logic: no Arduino headers, no heap, fixed-size storage. Tunables arrive
// in ControllerParams (D2); the state, fault and event *codes* come from
// config.h, which is the single source of truth for the wire protocol.
//
// The Controller never touches NVS or the network. It asks for persistence
// through saveBaseline / saveCyclesSinceEmptied and hands records to the caller.

#ifndef DG_CONTROLLER_H
#define DG_CONTROLLER_H

#include <stdint.h>

#include "dg_motion.h"

namespace dg {

// Wire protocol codes. dg_core is self-contained and does not include the
// project's config.h, so these are declared here and checked against the
// cfg:: constants by static_assert in the tests.
//
// Fault codes, ThingSpeak field 6.
inline constexpr uint8_t kFaultNone = 0;
inline constexpr uint8_t kFaultJamClearedByRetry = 1;
inline constexpr uint8_t kFaultJamNotCleared = 2;
inline constexpr uint8_t kFaultStillBlocked = 3;
inline constexpr uint8_t kFaultHomingFailed = 4;
inline constexpr uint8_t kFaultSensor = 5;

// Event codes, ThingSpeak field 7.
inline constexpr uint8_t kEventSnapshot = 0;
inline constexpr uint8_t kEventClearedVerified = 1;
inline constexpr uint8_t kEventFault = 2;
inline constexpr uint8_t kEventFaultRecovered = 3;
inline constexpr uint8_t kEventBinEmptied = 4;
inline constexpr uint8_t kEventBaselineRejected = 5;
inline constexpr uint8_t kEventJamRetryCleared = 6;

// Values are the wire codes uploaded to ThingSpeak field 4 (D6).
enum class State : uint8_t {
  MONITORING = 0,
  CONFIRMING = 1,
  CLEARING = 2,
  SETTLING = 3,
  FAULT = 4,
  HOMING = 5,
  BASELINE = 6,
};

// One log line. Fields map onto ThingSpeak 1-7; `cycles` is field 5 and carries
// cyclesSinceEmptied, not the per-blockage cycle count (D35).
//
// uValid/dValid say whether the two levels are real measurements or the filter's
// frozen last value (D41). A level that is not valid must never be uploaded as
// if it were data (D62): the upload builders leave the field out entirely, and
// the serial log prints "nan".
struct Record {
  uint32_t ms;
  float uCm;
  float dCm;
  float diffCm;
  uint8_t state;
  uint16_t cycles;
  uint8_t fault;
  uint8_t event;
  bool uValid;
  bool dValid;
};

struct ControllerParams {
  float thresholdCm;          // cfg::THRESHOLD_CM
  float verifyMarginCm;       // cfg::VERIFY_MARGIN_CM
  float baselineRejectCm;     // cfg::BASELINE_REJECT_CM
  uint32_t baselineMs;        // cfg::BASELINE_MS
  uint32_t confirmMs;         // cfg::CONFIRM_MS
  uint32_t settleMs;          // cfg::SETTLE_MS
  uint32_t snapshotMs;        // cfg::SNAPSHOT_MS
  uint32_t faultAutoRecoverMs;// cfg::FAULT_AUTORECOVER_MS
  uint32_t sensorFaultMs;     // cfg::SENSOR_FAULT_MS
  uint8_t maxCycles;          // cfg::MAX_CYCLES
};

struct ControllerInput {
  uint32_t nowMs;
  float upstreamCm;   // filtered level; the frozen last value when invalid (D41)
  float downstreamCm;
  bool upstreamValid;
  bool downstreamValid;
  bool buttonShortPress; // latched edge, cleared by the caller after the tick
  bool buttonHeldClear;
  MotionStatus motionStatus;
  uint8_t motionRetriesUsed;
};

struct ControllerOutput {
  Job motionJob; // Job::NONE means "leave the current job alone"

  bool ledStatus;   // green: the heartbeat is live (the 10 ms layer blinks it)
  bool ledFault;    // red
  bool ledClearing; // yellow: the motor is moving (D38)

  bool hasRecord; // at most one record per tick (D39)
  Record record;

  bool saveBaseline;
  float baselineCm;
  bool saveCyclesSinceEmptied;
  uint16_t cyclesSinceEmptied;
};

class Controller {
public:
  // Events are queued rather than dropped (D39). Four is the floor the decision
  // asks for; eight costs nothing and swallows any plausible burst.
  static constexpr uint8_t kEventQueueCapacity = 8;

  explicit Controller(const ControllerParams& params);

  // savedBaselineValid false means first boot: the measured baseline is taken
  // as-is and a post-fault re-home lands in BASELINE rather than MONITORING.
  void begin(uint32_t nowMs, bool savedBaselineValid, float savedBaselineCm,
             uint16_t savedCyclesSinceEmptied);

  ControllerOutput tick(const ControllerInput& in);

  State state() const { return state_; }
  uint8_t faultCode() const { return faultCode_; }
  uint8_t blockageCycles() const { return blockageCycles_; }
  float baselineCm() const { return baselineCm_; }
  bool haveBaseline() const { return haveBaseline_; }
  uint16_t cyclesSinceEmptied() const { return cyclesSinceEmptied_; }
  uint32_t stateEnteredMs() const { return stateEnteredMs_; }
  uint8_t pendingEvents() const { return queueCount_; }
  uint16_t eventOverflowCount() const { return eventOverflow_; }

private:
  void enterState(State s, uint32_t nowMs);
  void enterFault(uint8_t code, const ControllerInput& in);
  void queueEvent(uint8_t event, const ControllerInput& in);
  Record popEvent();
  Record buildRecord(uint8_t event, const ControllerInput& in) const;

  bool sensorsOk(const ControllerInput& in) const;
  bool sensorFaultDue(const ControllerInput& in) const;
  float diffOf(const ControllerInput& in) const;
  bool aboveThreshold(const ControllerInput& in) const;
  bool atBaseline(const ControllerInput& in) const;

  ControllerParams params_;

  State state_;
  uint32_t stateEnteredMs_;
  uint8_t faultCode_;
  uint8_t blockageCycles_; // per-blockage sweeps, 1..maxCycles, internal (D35)
  bool recoveringFromFault_;

  float baselineCm_;
  bool haveBaseline_;
  float savedBaselineCm_;
  bool savedBaselineValid_;
  float baselineSum_;
  uint32_t baselineCount_;

  uint16_t cyclesSinceEmptied_;

  bool sensorBad_;
  uint32_t sensorBadSinceMs_;

  bool atBaselineTimer_;
  uint32_t atBaselineSinceMs_;

  uint32_t lastRecordMs_;

  Record queue_[kEventQueueCapacity];
  uint8_t queueHead_;
  uint8_t queueCount_;
  uint16_t eventOverflow_;

  Job jobRequest_;
  bool saveBaseline_;
  bool saveCycles_;
};

} // namespace dg

#endif // DG_CONTROLLER_H
