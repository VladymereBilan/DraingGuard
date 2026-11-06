// DrainGuard firmware - S3: the complete control loop on real pins.
//
// All the decision-making lives in lib/dg_core and is unit-tested on the host;
// everything here is scheduling and I/O. The same binary runs in Wokwi and on
// the board, the only difference being the simulated rake under DG_WOKWI (D44).
//
// House rule for src/ and lib/: nothing blocks. Timing comes from millis() and
// micros(); the single exception is the 10 us HC-SR04 trigger pulse in
// hal_sensors.cpp (D43).
//
// Cadences (D3), all on fixed-rate deadlines (D56):
//   every pass      ultrasonic scheduler and echo retirement
//   10 ms           switches, current, button, MotionSupervisor -> motor pins
//   200 ms          Controller -> motion jobs, LEDs, persistence, CSV line
//   10 s            "#STAT" diagnostics line
//
// #STAT is a worst-case breakdown over the preceding 10 s window:
//   #STAT,loop_max_us,sensors_max_us,safety_max_us,control_max_us,
//         serial_max_us,events_overflowed,acs_zero_mV
// The four section figures should roughly sum to loop_max_us, which tells you
// immediately which part of a pass is costing the time. Target: under 5000 us.
//
// Wi-Fi, ThingSpeak and the LittleFS ring buffer are S4.

#include <Arduino.h>

#include "config.h"
#include "net_config.h" // credentials; also enforces that secrets.h exists

#include "dg_controller.h"
#include "dg_motion.h"

#include "hal_current.h"
#include "hal_io.h"
#include "hal_motor.h"
#include "hal_sensors.h"
#include "storage.h"

#ifdef DG_WOKWI
#include "sim_rake.h"
#endif

// ---------------------------------------------------------------------------
// dg_core declares the wire protocol codes itself so the library stays free of
// project headers. These assertions are the guard that it never drifts from
// config.h, which is the source of truth for what goes on the wire.
// ---------------------------------------------------------------------------
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

namespace {

constexpr char kFirmwareName[] = "DrainGuard";
constexpr char kFirmwareVersion[] = "0.4.0-s3";

#ifdef DG_WOKWI
constexpr char kBuildEnv[] = "WOKWI";
#else
constexpr char kBuildEnv[] = "ESP32";
#endif

// Diagnostics cadence. Not a control tunable, so it stays out of config.h.
constexpr uint32_t kStatMs = 10000;

constexpr dg::MotionParams kMotionParams = {
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

constexpr dg::ControllerParams kControllerParams = {
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

dg::MotionSupervisor g_sup(kMotionParams);
dg::Controller g_ctrl(kControllerParams);

hal::Sensors g_sensors;
hal::CurrentSense g_current;
hal::MotorDriver g_motor;
hal::Io g_io;
hal::Storage g_storage;

#ifdef DG_WOKWI
sim::Rake g_rake;
#endif

// Fixed-rate scheduler deadlines (D56): each one advances by exactly its
// period, so jitter in one pass does not push every later tick late. If a
// deadline falls more than a whole period behind - which should not happen once
// loop_max_us is under budget - it resyncs to now rather than catching up in a
// burst.
uint32_t g_nextSafetyMs = 0;
uint32_t g_nextControlMs = 0;
uint32_t g_nextStatMs = 0;
uint32_t g_lastSafetyRunMs = 0; // real elapsed time, for the simulated rake

// Worst-case section times for the current #STAT window, in microseconds.
uint32_t g_loopMaxUs = 0;
uint32_t g_sensorsMaxUs = 0;
uint32_t g_safetyMaxUs = 0;
uint32_t g_controlMaxUs = 0;
uint32_t g_serialMaxUs = 0;

inline void keepMax(uint32_t& slot, uint32_t value) {
  if (value > slot) {
    slot = value;
  }
}

// Returns true when the deadline has arrived, advancing it by one period.
bool due(uint32_t& nextMs, uint32_t nowMs, uint32_t periodMs) {
  if (static_cast<int32_t>(nowMs - nextMs) < 0) {
    return false;
  }
  nextMs += periodMs;
  if (static_cast<int32_t>(nowMs - nextMs) > static_cast<int32_t>(periodMs)) {
    // More than a period behind: give up the backlog rather than running a
    // burst of catch-up ticks. A large loop_max_us in #STAT is the symptom.
    nextMs = nowMs + periodMs;
  }
  return true;
}

// Latched button edges, consumed by the next controller tick.
bool g_shortPressLatch = false;
bool g_holdLatch = false;

// Most recent supervisor result, handed to the controller on its own cadence.
dg::MotorCmd g_lastCmd = dg::MotorCmd::STOP;
dg::MotionStatus g_lastStatus = dg::MotionStatus::IDLE;
uint8_t g_lastRetries = 0;

void printRecord(const dg::Record& r) {
  Serial.printf("#REC,%lu,%.2f,%.2f,%.2f,%u,%u,%u,%u\r\n",
                static_cast<unsigned long>(r.ms), r.uCm, r.dCm, r.diffCm,
                static_cast<unsigned>(r.state), static_cast<unsigned>(r.cycles),
                static_cast<unsigned>(r.fault), static_cast<unsigned>(r.event));
}

} // namespace

void setup() {
  // Motor pins first, before anything else can take time or fail: the H-bridge
  // must be driven off from the very first instruction after reset (D12).
  pinMode(cfg::MOTOR_IN1, OUTPUT);
  pinMode(cfg::MOTOR_IN2, OUTPUT);
  digitalWrite(cfg::MOTOR_IN1, LOW);
  digitalWrite(cfg::MOTOR_IN2, LOW);
  g_motor.begin();

  // A roomy TX buffer before begin(), so a print never stalls the control loop
  // waiting on the UART. At 115200 baud the firmware emits ~250 B/s against a
  // ~11.5 kB/s link, so this buffer never fills in practice.
  Serial.setTxBufferSize(1024);
  Serial.begin(cfg::SERIAL_BAUD);

  // One '#'-prefixed banner, then the CSV header. Every line ends CRLF (D45).
  Serial.printf("# %s %s env=%s\r\n", kFirmwareName, kFirmwareVersion,
                kBuildEnv);

  g_io.begin();

  const hal::Saved saved = g_storage.load();
  Serial.printf("#INFO,baseline_valid=%u,baseline_cm=%.2f,cycles=%u\r\n",
                static_cast<unsigned>(saved.baselineValid ? 1 : 0),
                saved.baselineCm,
                static_cast<unsigned>(saved.cyclesSinceEmptied));

  // The motor is stopped here, which is the only time the zero is meaningful.
  g_current.begin();
  if (g_current.zeroSubstituted()) {
    Serial.printf(
        "#WARN,acs_zero_out_of_range,measured_mV=%u,using_mV=%u\r\n",
        static_cast<unsigned>(g_current.measuredZeroMv()),
        static_cast<unsigned>(g_current.zeroMv()));
  }

  const uint32_t nowMs = millis();
  g_sensors.begin(nowMs);
  g_ctrl.begin(nowMs, saved.baselineValid, saved.baselineCm,
               saved.cyclesSinceEmptied);

  g_nextSafetyMs = nowMs + cfg::SAFETY_TICK_MS;
  g_nextControlMs = nowMs + cfg::CONTROL_TICK_MS;
  g_nextStatMs = nowMs + kStatMs;
  g_lastSafetyRunMs = nowMs;

  Serial.print("ms,U_cm,D_cm,diff_cm,state,current_A\r\n");
}

void loop() {
  const uint32_t passStartUs = micros();
  const uint32_t nowMs = millis();

  // Ultrasonic scheduling runs as often as possible so an echo is retired
  // close to when it actually arrived.
  const uint32_t sensorsStartUs = micros();
  g_sensors.loop(nowMs);
  keepMax(g_sensorsMaxUs, micros() - sensorsStartUs);

  // ---- 10 ms safety tick ------------------------------------------------
  if (due(g_nextSafetyMs, nowMs, cfg::SAFETY_TICK_MS)) {
    const uint32_t safetyStartUs = micros();
    const uint32_t dtMs = nowMs - g_lastSafetyRunMs;
    g_lastSafetyRunMs = nowMs;

    g_io.tick(nowMs);
    g_current.sample();

    bool home = g_io.homePressed();
    bool end = g_io.endPressed();
#ifdef DG_WOKWI
    // Integrate the command the motor has actually been given, then let the
    // simulated rake close either switch. The real buttons still override.
    g_rake.update(g_lastCmd, dtMs);
    home = home || g_rake.homePressed();
    end = end || g_rake.endPressed();
#else
    (void)dtMs;
#endif

    const dg::MotionInput mi{nowMs, home, end, g_current.amps()};
    const dg::MotionOutput mo = g_sup.tick(mi);
    g_motor.apply(mo.cmd);

    g_lastCmd = mo.cmd;
    g_lastStatus = mo.status;
    g_lastRetries = mo.retriesUsed;

    // Latch the edges so the slower controller tick cannot miss one.
    if (g_io.shortPress()) {
      g_shortPressLatch = true;
    }
    if (g_io.heldClear()) {
      g_holdLatch = true;
    }

    keepMax(g_safetyMaxUs, micros() - safetyStartUs);
  }

  // ---- 200 ms control tick ---------------------------------------------
  if (due(g_nextControlMs, nowMs, cfg::CONTROL_TICK_MS)) {
    const uint32_t controlStartUs = micros();

    dg::ControllerInput ci{};
    ci.nowMs = nowMs;
    ci.upstreamCm = g_sensors.upstreamLevelCm();
    ci.downstreamCm = g_sensors.downstreamLevelCm();
    ci.upstreamValid = g_sensors.upstreamValid();
    ci.downstreamValid = g_sensors.downstreamValid();
    ci.buttonShortPress = g_shortPressLatch;
    ci.buttonHeldClear = g_holdLatch;
    ci.motionStatus = g_lastStatus;
    ci.motionRetriesUsed = g_lastRetries;
    g_shortPressLatch = false;
    g_holdLatch = false;

    const dg::ControllerOutput co = g_ctrl.tick(ci);

    if (co.motionJob != dg::Job::NONE) {
      g_sup.requestJob(co.motionJob, nowMs);
      if (co.motionJob == dg::Job::STOP) {
        // Do not let a stop wait up to 10 ms for the next safety tick.
        g_motor.apply(dg::MotorCmd::STOP);
        g_lastCmd = dg::MotorCmd::STOP;
      }
    }

    g_io.setLeds(co.ledStatus, co.ledFault, co.ledClearing, nowMs);

    if (co.saveBaseline) {
      g_storage.saveBaseline(co.baselineCm);
    }
    if (co.saveCyclesSinceEmptied) {
      g_storage.saveCycles(co.cyclesSinceEmptied);
    }

    keepMax(g_controlMaxUs, micros() - controlStartUs);

    // One CSV line per control tick (D45). Timed separately so the #STAT
    // breakdown shows whether serial or control logic is costing the time.
    const uint32_t serialStartUs = micros();
    Serial.printf("%lu,%.2f,%.2f,%.2f,%u,%.3f\r\n",
                  static_cast<unsigned long>(nowMs), ci.upstreamCm,
                  ci.downstreamCm, ci.upstreamCm - ci.downstreamCm,
                  static_cast<unsigned>(g_ctrl.state()), g_current.amps());
    if (co.hasRecord) {
      printRecord(co.record);
    }
    keepMax(g_serialMaxUs, micros() - serialStartUs);
  }

  // ---- 10 s diagnostics -------------------------------------------------
  if (due(g_nextStatMs, nowMs, kStatMs)) {
    Serial.printf("#STAT,%lu,%lu,%lu,%lu,%lu,%u,%u\r\n",
                  static_cast<unsigned long>(g_loopMaxUs),
                  static_cast<unsigned long>(g_sensorsMaxUs),
                  static_cast<unsigned long>(g_safetyMaxUs),
                  static_cast<unsigned long>(g_controlMaxUs),
                  static_cast<unsigned long>(g_serialMaxUs),
                  static_cast<unsigned>(g_ctrl.eventOverflowCount()),
                  static_cast<unsigned>(g_current.zeroMv()));
    g_loopMaxUs = 0;
    g_sensorsMaxUs = 0;
    g_safetyMaxUs = 0;
    g_controlMaxUs = 0;
    g_serialMaxUs = 0;
  }

  keepMax(g_loopMaxUs, micros() - passStartUs);
}
