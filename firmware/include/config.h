// DrainGuard central configuration.
//
// Every tunable lives here so the control logic never carries magic numbers.
// Values marked "// CALIBRATE" must be measured on the real rig before trusting them.
//
// This header holds no secrets and no Wi-Fi logic (decision D24) - those live in
// src/net_config.h. It includes only <stdint.h>, so it compiles unchanged on the
// ESP32 and on the host, and lib/dg_core may include it freely.

#ifndef DG_CONFIG_H
#define DG_CONFIG_H

#include <stdint.h>

namespace cfg {

// ---------------------------------------------------------------------------
// Pins (ESP32 DevKit v1 GPIO numbers)
// ---------------------------------------------------------------------------
inline constexpr uint8_t US_TRIG = 18;      // upstream HC-SR04 trigger
inline constexpr uint8_t US_ECHO = 19;      // upstream HC-SR04 echo (2.2k/3.3k divider on hardware)
inline constexpr uint8_t DS_TRIG = 21;      // downstream HC-SR04 trigger
inline constexpr uint8_t DS_ECHO = 22;      // downstream HC-SR04 echo (2.2k/3.3k divider on hardware)

inline constexpr uint8_t MOTOR_IN1 = 25;    // H-bridge IN1
inline constexpr uint8_t MOTOR_IN2 = 26;    // H-bridge IN2

inline constexpr uint8_t SW_HOME = 32;      // rake home (fully raised) limit switch
inline constexpr uint8_t SW_END = 33;       // rake end-of-travel (fully lowered) limit switch
inline constexpr uint8_t BTN_BIN = 27;      // "bin emptied" acknowledge button

inline constexpr uint8_t ACS712 = 34;       // motor current sense, ADC1 input-only pin

inline constexpr uint8_t LED_STATUS = 23;   // green  - heartbeat / monitoring
inline constexpr uint8_t LED_FAULT = 13;    // red    - fault latched
inline constexpr uint8_t LED_CLEARING = 14; // yellow - clearing cycle running

// ---------------------------------------------------------------------------
// Motor direction
// ---------------------------------------------------------------------------
// IN1 HIGH (IN2 LOW) = lower: the rake sweeps down across the screen.
// IN2 HIGH (IN1 LOW) = raise: the rake returns up to the home switch.
// Both LOW = coast/off (D11; the worm gear self-locks). Both HIGH is never commanded.
// If the rake moves the wrong way on the bench, swap the motor leads, not the code (D14).

// ---------------------------------------------------------------------------
// Switches and button
// ---------------------------------------------------------------------------
// All three are wired to GND and read with INPUT_PULLUP, so they are active LOW:
// a pressed button / closed switch reads 0.
inline constexpr uint32_t BTN_HOLD_CLEAR_MS = 3000; // hold BTN_BIN this long to clear a latched fault
inline constexpr uint32_t DEBOUNCE_MS = 30;         // contact debounce window

// ---------------------------------------------------------------------------
// Ultrasonic sensors
// ---------------------------------------------------------------------------
inline constexpr float SOUND_CM_PER_US = 0.0343f;   // cm per microsecond; no temperature compensation (D27)
inline constexpr uint32_t SENSOR_PING_PERIOD_MS = 100; // per sensor; the two are offset by SENSOR_STAGGER_MS (D21)
inline constexpr uint32_t SENSOR_STAGGER_MS = 50;   // gap between the two sensors so they do not hear each other
inline constexpr int SENSOR_STALE_SAMPLES = 10;     // pushes with no valid median -> sensor invalid (D25)
inline constexpr float RANGE_MIN_CM = 2.0f;         // below this the reading is noise, discard
inline constexpr float RANGE_MAX_CM = 200.0f;       // above this the echo is a miss, discard
inline constexpr uint8_t MEDIAN_N = 5;              // samples per median (spike rejection)
inline constexpr uint8_t AVG_N = 5;                 // medians per moving average (smoothing)
inline constexpr float US_MOUNT_HEIGHT_CM = 40.0f;  // CALIBRATE - upstream sensor face to dry channel bed
inline constexpr float DS_MOUNT_HEIGHT_CM = 40.0f;  // CALIBRATE - downstream sensor face to dry channel bed
inline constexpr uint32_t ECHO_TIMEOUT_US = 15000;  // give up on an echo after this (~257 cm round trip)

// ---------------------------------------------------------------------------
// ACS712 current sensor
// ---------------------------------------------------------------------------
// 0.185 V/A is the ACS712-05B sensitivity; the 10k/20k divider that drops its
// output into the ESP32 ADC range scales it by 20/(10+20).
inline constexpr float ACS_SENS_V_PER_A = 0.185f * 20.0f / 30.0f; // volts per amp at the ADC pin
inline constexpr uint16_t ACS_SAMPLES = 64;         // ADC samples averaged per current reading
inline constexpr float I_RUN_A = 0.5f;              // CALIBRATE - normal free-running rake current
// The sensor idles at ~2.5 V, which the 10k/20k divider presents as ~1667 mV.
// A zero-current reading outside this window means the sensor or divider is faulty.
inline constexpr uint16_t ACS_ZERO_MIN_MV = 1400;   // millivolts
inline constexpr uint16_t ACS_ZERO_MAX_MV = 1950;   // millivolts

// ---------------------------------------------------------------------------
// Jam detection and recovery
// ---------------------------------------------------------------------------
inline constexpr float JAM_FACTOR = 1.5f;           // jam when current exceeds I_RUN_A * JAM_FACTOR ...
inline constexpr float JAM_MIN_DELTA_A = 0.3f;      // ... and is at least this far above I_RUN_A
inline constexpr uint32_t JAM_HOLD_MS = 200;        // the over-current must persist this long to count
inline constexpr uint32_t INRUSH_BLANK_MS = 300;    // ignore current for this long after a motor start
inline constexpr uint32_t JAM_REVERSE_MS = 500;     // back off this long to shake the obstruction loose
inline constexpr uint32_t JAM_PAUSE_MS = 1000;      // settle after the back-off before trying again
inline constexpr uint8_t MAX_RETRIES_PER_CYCLE = 2; // jam retries allowed inside one clearing cycle

// ---------------------------------------------------------------------------
// Motor timing
// ---------------------------------------------------------------------------
inline constexpr uint32_t DEADTIME_BEFORE_REVERSE_MS = 100; // both IN pins LOW before flipping direction
inline constexpr uint32_t TRAVEL_TIMEOUT_MS = 15000;        // one sweep must finish inside this (restarts per retry, D17)
inline constexpr uint32_t HOMING_TIMEOUT_MS = 15000;        // reaching the home switch must finish inside this

// ---------------------------------------------------------------------------
// Control loop
// ---------------------------------------------------------------------------
inline constexpr uint32_t CONTROL_TICK_MS = 200;    // state machine / sensor cadence (D3)
inline constexpr uint32_t SAFETY_TICK_MS = 10;      // limit switch and over-current cadence (D3)
inline constexpr uint32_t BASELINE_MS = 30000;      // clear-channel averaging window at startup
inline constexpr float THRESHOLD_CM = 2.0f;         // CALIBRATE - upstream rise over baseline that means blocked
inline constexpr uint32_t CONFIRM_MS = 10000;       // CALIBRATE - threshold must hold this long before clearing
inline constexpr uint32_t SETTLE_MS = 15000;        // CALIBRATE - wait after a cycle before re-measuring
inline constexpr float VERIFY_MARGIN_CM = 1.0f;     // how far back under threshold counts as cleared
inline constexpr uint8_t MAX_CYCLES = 3;            // clearing cycles before declaring "still blocked"
inline constexpr uint32_t FAULT_AUTORECOVER_MS = 60000;   // auto-retry a latched fault after this (D16)
inline constexpr float BASELINE_REJECT_CM = THRESHOLD_CM; // reject a saved-vs-new baseline gap wider than this (D15)

// ---------------------------------------------------------------------------
// Logging, time and serial
// ---------------------------------------------------------------------------
inline constexpr uint32_t SNAPSHOT_MS = 20000;       // periodic snapshot event interval (D5)
inline constexpr uint32_t UPLOAD_SPACING_MS = 15500; // minimum gap between uploads (ThingSpeak free tier)
inline constexpr uint16_t RING_CAPACITY = 200;       // LittleFS event ring buffer entries (D7)
inline constexpr uint8_t LOG_QUEUE_DEPTH = 16;       // pending upload queue depth
inline constexpr char NTP_SERVER[] = "pool.ntp.org";
inline constexpr long GMT_OFFSET_S = 8 * 3600;       // UTC+8
inline constexpr int DST_OFFSET_S = 0;
inline constexpr uint32_t SERIAL_BAUD = 115200;

// ---------------------------------------------------------------------------
// State codes (reported in the CSV "state" column, uploaded to field 4, D6)
// ---------------------------------------------------------------------------
inline constexpr uint8_t STATE_MONITORING = 0;
inline constexpr uint8_t STATE_CONFIRMING = 1;
inline constexpr uint8_t STATE_CLEARING = 2;
inline constexpr uint8_t STATE_SETTLING = 3;
inline constexpr uint8_t STATE_FAULT = 4;
inline constexpr uint8_t STATE_HOMING = 5;
inline constexpr uint8_t STATE_BASELINE = 6;

// ---------------------------------------------------------------------------
// Fault codes
// ---------------------------------------------------------------------------
inline constexpr uint8_t FAULT_NONE = 0;
inline constexpr uint8_t FAULT_JAM_CLEARED_BY_RETRY = 1;
inline constexpr uint8_t FAULT_JAM_NOT_CLEARED = 2;
inline constexpr uint8_t FAULT_STILL_BLOCKED = 3;   // still blocked after MAX_CYCLES cycles
inline constexpr uint8_t FAULT_HOMING_FAILED = 4;

// ---------------------------------------------------------------------------
// Event codes
// ---------------------------------------------------------------------------
inline constexpr uint8_t EVENT_SNAPSHOT = 0;
inline constexpr uint8_t EVENT_CLEARED_VERIFIED = 1;
inline constexpr uint8_t EVENT_FAULT = 2;
inline constexpr uint8_t EVENT_FAULT_RECOVERED = 3;
inline constexpr uint8_t EVENT_BIN_EMPTIED = 4;

} // namespace cfg

#endif // DG_CONFIG_H
