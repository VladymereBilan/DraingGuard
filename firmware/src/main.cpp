// DrainGuard firmware - scaffold only.
//
// This file brings the board up into a known-safe state and prints the CSV
// header the logging tools expect. No sensing, motor control or state machine
// yet; those arrive in later steps.
//
// House rule for everything under src/ and lib/: no blocking calls - no sleeps,
// no busy-wait pulse measurement. All timing is non-blocking, driven from millis(),
// so the safety checks can never be stalled by a slow sensor. See README.md.

#include <Arduino.h>

#include "config.h"
#include "net_config.h" // credentials; also enforces that secrets.h exists

namespace {

constexpr char FIRMWARE_NAME[] = "DrainGuard";
constexpr char FIRMWARE_VERSION[] = "0.3.0-scaffold";

#ifdef DG_WOKWI
constexpr char BUILD_ENV[] = "WOKWI";
#else
constexpr char BUILD_ENV[] = "ESP32";
#endif

constexpr uint32_t HEARTBEAT_MS = 500;

uint32_t g_lastHeartbeatMs = 0;
bool g_statusLedOn = false;

} // namespace

void setup() {
  // Motor pins first, before anything else can take time or fail: the H-bridge
  // must be driven off from the very first instruction after reset.
  pinMode(cfg::MOTOR_IN1, OUTPUT);
  pinMode(cfg::MOTOR_IN2, OUTPUT);
  digitalWrite(cfg::MOTOR_IN1, LOW);
  digitalWrite(cfg::MOTOR_IN2, LOW);

  // Limit switches and the bin button: wired to GND, so pull up and read active LOW.
  pinMode(cfg::SW_HOME, INPUT_PULLUP);
  pinMode(cfg::SW_END, INPUT_PULLUP);
  pinMode(cfg::BTN_BIN, INPUT_PULLUP);

  // Indicator LEDs, all dark to start.
  pinMode(cfg::LED_STATUS, OUTPUT);
  pinMode(cfg::LED_FAULT, OUTPUT);
  pinMode(cfg::LED_CLEARING, OUTPUT);
  digitalWrite(cfg::LED_STATUS, LOW);
  digitalWrite(cfg::LED_FAULT, LOW);
  digitalWrite(cfg::LED_CLEARING, LOW);

  // Ultrasonic sensors: triggers idle low, echoes are inputs.
  pinMode(cfg::US_TRIG, OUTPUT);
  pinMode(cfg::DS_TRIG, OUTPUT);
  digitalWrite(cfg::US_TRIG, LOW);
  digitalWrite(cfg::DS_TRIG, LOW);
  pinMode(cfg::US_ECHO, INPUT);
  pinMode(cfg::DS_ECHO, INPUT);

  Serial.begin(cfg::SERIAL_BAUD);

  // One '#'-prefixed banner line, then the CSV header. Downstream tools skip
  // '#' lines and parse everything after the header as data.
  Serial.printf("# %s %s env=%s\n", FIRMWARE_NAME, FIRMWARE_VERSION, BUILD_ENV);
  Serial.println("ms,U_cm,D_cm,diff_cm,state,current_A");

  g_lastHeartbeatMs = millis();
}

void loop() {
  const uint32_t now = millis();

  if (now - g_lastHeartbeatMs >= HEARTBEAT_MS) {
    g_lastHeartbeatMs = now;
    g_statusLedOn = !g_statusLedOn;
    digitalWrite(cfg::LED_STATUS, g_statusLedOn ? HIGH : LOW);
  }
}
