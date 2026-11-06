// Non-volatile settings in NVS (Preferences), namespace "dg".
//
// Only two things survive a reboot (D7): the learned baseline differential and
// the number of clearing cycles since the bin was last emptied. The event ring
// buffer is LittleFS and is not part of S3.
//
// Writes happen only when the Controller asks for them, never on a timer, so
// the flash sees a handful of writes per day rather than one per tick.

#ifndef DG_STORAGE_H
#define DG_STORAGE_H

#include <stdint.h>

namespace hal {

struct Saved {
  bool baselineValid;
  float baselineCm;
  uint16_t cyclesSinceEmptied;
};

class Storage {
public:
  Storage();

  // Opens the namespace and reads both values. Call once in setup().
  Saved load();

  void saveBaseline(float cm);
  void saveCycles(uint16_t cycles);

  uint32_t writes() const { return writes_; }

private:
  uint32_t writes_;
};

} // namespace hal

#endif // DG_STORAGE_H
