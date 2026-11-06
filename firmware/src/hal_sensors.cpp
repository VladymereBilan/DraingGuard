#include "hal_sensors.h"

#include <Arduino.h>

#include "config.h"

namespace hal {
namespace {

// One per sensor. These live at file scope because the ISR needs a stable
// address that does not depend on where the Sensors object was placed.
struct Echo {
  uint8_t echoPin;
  volatile uint32_t riseUs;
  volatile uint32_t fallUs;
  volatile bool haveRise;
  volatile bool done;
};

Echo g_echo[2];

// Timestamps both edges of the echo pulse. Placed in IRAM so the handler itself
// never waits on flash; digitalRead is a normal flash call, which is safe here
// because attachInterruptArg does not allocate the interrupt with
// ESP_INTR_FLAG_IRAM.
void IRAM_ATTR echoIsr(void* arg) {
  Echo* e = static_cast<Echo*>(arg);
  if (digitalRead(e->echoPin) == HIGH) {
    e->riseUs = micros();
    e->haveRise = true;
    e->done = false;
  } else if (e->haveRise) {
    e->fallUs = micros();
    e->done = true;
  }
}

constexpr dg::FilterParams kFilter = {cfg::RANGE_MIN_CM, cfg::RANGE_MAX_CM,
                                      cfg::SENSOR_STALE_SAMPLES};

// Pushed when an echo never returns, so the filter ages the sensor out (D25).
constexpr float kInvalidSample = -1.0f;

} // namespace

Sensors::Sensors()
    : ch_{{cfg::US_TRIG, cfg::US_ECHO, cfg::US_MOUNT_HEIGHT_CM, 0, 0, false},
          {cfg::DS_TRIG, cfg::DS_ECHO, cfg::DS_MOUNT_HEIGHT_CM, 0, 0, false}},
      filt_{dg::DistanceFilter(kFilter), dg::DistanceFilter(kFilter)},
      timeouts_(0) {}

void Sensors::begin(uint32_t nowMs) {
  for (uint8_t i = 0; i < 2; ++i) {
    pinMode(ch_[i].trigPin, OUTPUT);
    digitalWrite(ch_[i].trigPin, LOW);
    pinMode(ch_[i].echoPin, INPUT);

    g_echo[i].echoPin = ch_[i].echoPin;
    g_echo[i].riseUs = 0;
    g_echo[i].fallUs = 0;
    g_echo[i].haveRise = false;
    g_echo[i].done = false;

    attachInterruptArg(digitalPinToInterrupt(ch_[i].echoPin), echoIsr,
                       &g_echo[i], CHANGE);
    ch_[i].waiting = false;
  }

  // Stagger the pair so one sensor's burst cannot be heard by the other.
  ch_[kUp].nextPingMs = nowMs;
  ch_[kDown].nextPingMs = nowMs + cfg::SENSOR_STAGGER_MS;
}

void Sensors::trigger(uint8_t i) {
  noInterrupts();
  g_echo[i].haveRise = false;
  g_echo[i].done = false;
  interrupts();

  digitalWrite(ch_[i].trigPin, HIGH);
  delayMicroseconds(10); // D43: the only busy-wait in the firmware
  digitalWrite(ch_[i].trigPin, LOW);

  ch_[i].pingUs = micros();
  ch_[i].waiting = true;
}

void Sensors::loop(uint32_t nowMs) {
  for (uint8_t i = 0; i < 2; ++i) {
    if (ch_[i].waiting) {
      if (g_echo[i].done) {
        noInterrupts();
        const uint32_t rise = g_echo[i].riseUs;
        const uint32_t fall = g_echo[i].fallUs;
        interrupts();

        ch_[i].waiting = false;
        // echoUsToCm maps a zero-width pulse to -1, which the filter rejects.
        filt_[i].push(dg::echoUsToCm(fall - rise, cfg::SOUND_CM_PER_US));
      } else if ((micros() - ch_[i].pingUs) >= cfg::ECHO_TIMEOUT_US) {
        ch_[i].waiting = false;
        ++timeouts_;
        filt_[i].push(kInvalidSample);
      }
    }

    if (!ch_[i].waiting &&
        static_cast<int32_t>(nowMs - ch_[i].nextPingMs) >= 0) {
      // Catch up rather than burst if a pass ever runs long.
      do {
        ch_[i].nextPingMs += cfg::SENSOR_PING_PERIOD_MS;
      } while (static_cast<int32_t>(nowMs - ch_[i].nextPingMs) >= 0);
      trigger(i);
    }
  }
}

float Sensors::upstreamDistanceCm() const { return filt_[kUp].value(); }
float Sensors::downstreamDistanceCm() const { return filt_[kDown].value(); }

float Sensors::upstreamLevelCm() const {
  return dg::levelFromDistance(ch_[kUp].mountCm, filt_[kUp].value());
}

float Sensors::downstreamLevelCm() const {
  return dg::levelFromDistance(ch_[kDown].mountCm, filt_[kDown].value());
}

bool Sensors::upstreamValid() const { return filt_[kUp].valid(); }
bool Sensors::downstreamValid() const { return filt_[kDown].valid(); }

} // namespace hal
