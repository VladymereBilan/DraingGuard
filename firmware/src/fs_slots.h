// dg::SlotIo backed by a fixed-size LittleFS file.
//
// /ring.bin is RING_CAPACITY x 32 bytes, pre-allocated on first mount so an
// append is always an in-place overwrite of one slot - never a file growth,
// which would involve more metadata and a wider window for a power cut.
//
// If LittleFS will not mount at all the volume is formatted and we carry on
// with an empty ring (D54): refusing to boot because a log buffer is corrupt
// would be the wrong trade for a drain guard.
//
// Every write is flushed, so a confirmed append really is on flash.

#ifndef DG_FS_SLOTS_H
#define DG_FS_SLOTS_H

#include <stdint.h>

#include <FS.h>

#include "dg_ring.h"

namespace hal {

class FsSlots : public dg::SlotIo {
public:
  FsSlots();

  // Mounts the filesystem and opens the backing file. False means the ring is
  // unavailable this boot; the logger then uploads live records only.
  bool begin(uint16_t slots);

  bool ready() const { return ready_; }
  bool formatted() const { return formatted_; }

  uint16_t slotCount() const override { return slots_; }
  bool readSlot(uint16_t index, uint8_t out[dg::kSlotBytes]) override;
  bool writeSlot(uint16_t index, const uint8_t in[dg::kSlotBytes]) override;

  uint32_t ioErrors() const { return ioErrors_; }

private:
  bool allocateFile(uint16_t slots);

  File file_;
  uint16_t slots_;
  bool ready_;
  bool formatted_;
  uint32_t ioErrors_;
};

} // namespace hal

#endif // DG_FS_SLOTS_H
