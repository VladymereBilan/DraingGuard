// Switches, button and indicator LEDs.
//
// The two limit switches and the bin button are wired to GND and read with
// INPUT_PULLUP, so they are active LOW. A switch counts as pressed after two
// consecutive LOW reads and released after two consecutive HIGH reads, which at
// the 10 ms safety cadence is a 20 ms debounce (D47).
//
// The bin button's press/hold discrimination is dg::HoldButton; this file only
// inverts the pin and forwards the level.

#ifndef DG_HAL_IO_H
#define DG_HAL_IO_H

#include <stdint.h>

#include "dg_button.h"

namespace hal {

class Io {
public:
  Io();

  void begin();

  // Call on the 10 ms safety tick.
  void tick(uint32_t nowMs);

  bool homePressed() const { return home_; }
  bool endPressed() const { return end_; }

  // Edges, consumed on read, so the 200 ms controller tick cannot miss one.
  bool shortPress() { return button_.shortPress(); }
  bool heldClear() { return button_.heldClear(); }

  // statusLive is the Controller's "heartbeat is running" output; the blink
  // itself is generated here because it is a presentation detail.
  void setLeds(bool statusLive, bool fault, bool clearing, uint32_t nowMs);

private:
  static constexpr uint8_t kDebounceReads = 2; // D47

  dg::HoldButton button_;

  uint8_t homeLow_;
  uint8_t homeHigh_;
  uint8_t endLow_;
  uint8_t endHigh_;
  bool home_;
  bool end_;

  bool statusOn_;
  uint32_t lastBlinkMs_;
};

} // namespace hal

#endif // DG_HAL_IO_H
