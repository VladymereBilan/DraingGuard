// Motion supervisor: executes one motion job at a time and owns every rule that
// protects the motor and the gearbox.
//
// Ticked by the caller every SAFETY_TICK_MS (10 ms) with the live switch states
// and motor current (D3, D29). Pure logic: no Arduino headers, no heap, all
// tunables arrive in MotionParams (D2).
//
// Safety properties, by construction:
//   - MotorCmd is a single enum, so "IN1 and IN2 both HIGH" is unrepresentable.
//   - Any reversal is routed through a STOP phase of at least deadtimeMs.
//   - Jam current is only judged on RAISE strokes (D10), never during the
//     inrush window and never during the reverse pulse (D41).
//   - Both limit switches closed at once is a wiring fault: stop and fail (D40).

#ifndef DG_MOTION_H
#define DG_MOTION_H

#include <stdint.h>

namespace dg {

enum class Job : uint8_t {
  NONE,
  HOME,  // raise until the home switch closes
  SWEEP, // lower to the end switch, then raise back to home
  STOP,  // stop now and go idle
};

enum class MotorCmd : uint8_t {
  STOP,  // both H-bridge inputs LOW (coast; the worm gear self-locks, D11)
  LOWER, // IN1 HIGH: rake sweeps down the screen (D14)
  RAISE, // IN2 HIGH: rake returns to home (D14)
};

enum class MotionStatus : uint8_t {
  IDLE,             // no job
  BUSY,             // job in progress
  DONE,             // job finished, no retries needed
  DONE_AFTER_RETRY, // job finished but fought a jam (feeds fault 1 / event 6)
  FAILED_JAM,       // retries exhausted (feeds fault 2)
  FAILED_TIMEOUT,   // HOME job failed, or both switches closed (feeds fault 4)
};

// Internal progress, exposed for tests and debugging only.
enum class Phase : uint8_t {
  IDLE,
  DEADTIME,      // commanded STOP before a reversal
  LOWERING,
  RAISING,
  REVERSE_PULSE, // short nudge the other way to shake an obstruction loose
  PAUSE,         // settle after the nudge, before retrying
  FINISHED,      // terminal status latched
};

// True once a job has reached a terminal outcome.
bool isTerminal(MotionStatus s);

struct MotionParams {
  float iRunA;             // cfg::I_RUN_A
  float jamFactor;         // cfg::JAM_FACTOR
  float jamMinDeltaA;      // cfg::JAM_MIN_DELTA_A
  uint32_t jamHoldMs;      // cfg::JAM_HOLD_MS
  uint32_t inrushBlankMs;  // cfg::INRUSH_BLANK_MS
  uint32_t jamReverseMs;   // cfg::JAM_REVERSE_MS
  uint32_t jamPauseMs;     // cfg::JAM_PAUSE_MS
  uint32_t deadtimeMs;     // cfg::DEADTIME_BEFORE_REVERSE_MS
  uint32_t travelTimeoutMs;// cfg::TRAVEL_TIMEOUT_MS
  uint32_t homingTimeoutMs;// cfg::HOMING_TIMEOUT_MS
  uint8_t maxRetriesPerJob;// cfg::MAX_RETRIES_PER_CYCLE
};

struct MotionInput {
  uint32_t nowMs;
  bool homePressed; // already de-inverted: true means the switch is closed
  bool endPressed;
  float currentA;
};

struct MotionOutput {
  MotorCmd cmd;
  MotionStatus status;
  uint8_t retriesUsed;
};

class MotionSupervisor {
public:
  explicit MotionSupervisor(const MotionParams& params);

  // Starts a new job. Commands STOP synchronously so no motion can continue
  // past the request, then evaluates the switches on the next tick - which is
  // what lets HOME short-circuit when already home (D40).
  void requestJob(Job job, uint32_t nowMs);

  // One 10 ms step. The returned command is what the caller must drive onto the
  // motor pins this tick.
  MotionOutput tick(const MotionInput& in);

  Phase phase() const { return phase_; }
  Job job() const { return job_; }
  uint8_t retriesUsed() const { return retries_; }

  // Current level that counts as a jam: max(iRun * factor, iRun + minDelta).
  float jamThresholdA() const;

  // Largest |current| seen since the present stroke started, measured after
  // the inrush window so it reflects running current. Zero until a stroke
  // has run past blanking. Useful for calibrating I_RUN_A (D64).
  float strokePeakA() const { return strokePeakA_; }

  void reset();

private:
  bool needsDeadtime(MotorCmd dir, uint32_t nowMs) const;
  void requestDirection(MotorCmd dir, Phase next, uint32_t nowMs);
  void startStroke(MotorCmd dir, Phase next, uint32_t nowMs);
  void setCmd(MotorCmd c, uint32_t nowMs);
  void finish(MotionStatus s, uint32_t nowMs);
  void retryOrFail(uint32_t nowMs);
  bool jamHeld(const MotionInput& in);

  MotionParams params_;

  Job job_;
  Phase phase_;
  MotionStatus status_;
  MotorCmd cmd_;

  bool jobPending_; // job requested, switches not yet evaluated

  MotorCmd lastDir_;      // last non-STOP command, for reversal detection
  uint32_t stopSinceMs_;  // when cmd_ last became STOP
  MotorCmd pendingDir_;   // direction to enter after DEADTIME
  Phase pendingPhase_;
  MotorCmd interruptedDir_; // stroke direction a retry must resume

  uint32_t jobStartMs_;
  uint32_t strokeStartMs_; // restarted on every retry (D17)
  uint32_t phaseStartMs_;
  uint32_t motorStartMs_;  // last STOP -> direction transition, for inrush

  bool jamActive_;
  uint32_t jamSinceMs_;
  float strokePeakA_;

  uint8_t retries_;
};

} // namespace dg

#endif // DG_MOTION_H
