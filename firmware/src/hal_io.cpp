#include "hal_io.h"

#include <Arduino.h>

#include "config.h"

namespace hal {
namespace {

constexpr dg::ButtonParams kButton = {cfg::DEBOUNCE_MS,
                                      cfg::BTN_HOLD_CLEAR_MS};

// Status-LED heartbeat. Presentation only, so it lives here rather than in
// config.h with the control tunables.
constexpr uint32_t kHeartbeatMs = 500;

// Advances a two-sided debounce counter pair and returns the settled level.
bool debounce(bool rawPressed, uint8_t& pressCount, uint8_t& releaseCount,
              bool current, uint8_t needed) {
  if (rawPressed) {
    releaseCount = 0;
    if (pressCount < needed) {
      ++pressCount;
    }
  } else {
    pressCount = 0;
    if (releaseCount < needed) {
      ++releaseCount;
    }
  }
  if (pressCount >= needed) {
    return true;
  }
  if (releaseCount >= needed) {
    return false;
  }
  return current;
}

} // namespace

Io::Io()
    : button_(kButton),
      homeLow_(0),
      homeHigh_(0),
      endLow_(0),
      endHigh_(0),
      home_(false),
      end_(false),
      statusOn_(false),
      lastBlinkMs_(0) {}

void Io::begin() {
  pinMode(cfg::SW_HOME, INPUT_PULLUP);
  pinMode(cfg::SW_END, INPUT_PULLUP);
  pinMode(cfg::BTN_BIN, INPUT_PULLUP);

  pinMode(cfg::LED_STATUS, OUTPUT);
  pinMode(cfg::LED_FAULT, OUTPUT);
  pinMode(cfg::LED_CLEARING, OUTPUT);
  digitalWrite(cfg::LED_STATUS, LOW);
  digitalWrite(cfg::LED_FAULT, LOW);
  digitalWrite(cfg::LED_CLEARING, LOW);

  button_.reset();
}

void Io::tick(uint32_t nowMs) {
  // Active LOW: a closed contact reads 0.
  home_ = debounce(digitalRead(cfg::SW_HOME) == LOW, homeLow_, homeHigh_, home_,
                   kDebounceReads);
  end_ = debounce(digitalRead(cfg::SW_END) == LOW, endLow_, endHigh_, end_,
                  kDebounceReads);

  button_.tick(nowMs, digitalRead(cfg::BTN_BIN) == LOW);
}

void Io::setLeds(bool statusLive, bool fault, bool clearing, uint32_t nowMs) {
  if (statusLive) {
    if ((nowMs - lastBlinkMs_) >= kHeartbeatMs) {
      lastBlinkMs_ = nowMs;
      statusOn_ = !statusOn_;
    }
  } else {
    statusOn_ = false;
    lastBlinkMs_ = nowMs;
  }

  digitalWrite(cfg::LED_STATUS, statusOn_ ? HIGH : LOW);
  digitalWrite(cfg::LED_FAULT, fault ? HIGH : LOW);
  digitalWrite(cfg::LED_CLEARING, clearing ? HIGH : LOW);
}

} // namespace hal
