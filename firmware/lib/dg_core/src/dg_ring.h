// The offline record ring: index and ordering logic only.
//
// Slot storage is injected through SlotIo, so this compiles and is tested on
// the host against a RAM-backed fake and runs on the device against LittleFS.
// No heap, no Arduino, fixed-size index arrays.
//
// Ordering authority (D51): once the ring holds anything, every new record
// goes through it, so logical position 0 is always the oldest thing still
// waiting to be uploaded.
//
// Acknowledgement (D53): the caller persists lastDroppedSeq() in NVS after
// each confirmed upload and hands it back to mount(), which then skips every
// slot at or below it. Nothing already uploaded is re-sent after a reboot.

#ifndef DG_RING_H
#define DG_RING_H

#include <stdint.h>

#include "dg_record_codec.h"

namespace dg {

// Upper bound on the slot count this index can hold. RING_CAPACITY is 200.
inline constexpr uint16_t kRingMaxSlots = 256;

class SlotIo {
public:
  virtual ~SlotIo() = default;
  virtual uint16_t slotCount() const = 0;
  virtual bool readSlot(uint16_t index, uint8_t out[kSlotBytes]) = 0;
  virtual bool writeSlot(uint16_t index, const uint8_t in[kSlotBytes]) = 0;
};

struct RingStats {
  uint16_t count;       // records currently held
  uint32_t overwritten; // oldest dropped to make room; feeds field 8
  uint16_t badSlots;    // failed CRC or version on the mount scan
  uint32_t ackSkipped;  // slots skipped on mount because already uploaded
};

class Ring {
public:
  explicit Ring(SlotIo& io);

  // Scans every slot and rebuilds the order from the seq numbers. This is the
  // whole crash-recovery path: there is no metadata to replay. Slots with
  // seq <= lastAckSeq are treated as already delivered and dropped (D53).
  void mount(uint32_t lastAckSeq);

  // Assigns the next seq and writes one slot. Overwrites the oldest record
  // when the ring is full, counting that as a drop.
  bool append(const StoredRecord& r);

  uint16_t count() const { return count_; }
  bool empty() const { return count_ == 0; }
  bool full() const;

  // n == 0 is the oldest record held. Not const: the backing store may need to
  // seek, and lying about that would push the problem into the FS layer.
  bool readAt(uint16_t n, StoredRecord& out);
  bool peekOldest(StoredRecord& out) { return readAt(0, out); }

  // Called after an upload is confirmed. Advances lastDroppedSeq().
  void dropOldest(uint16_t howMany);

  uint32_t nextSeq() const { return nextSeq_; }

  // Highest seq no longer held, whether it was uploaded or overwritten.
  // Persist this as lastAckSeq; anything at or below it is gone for good.
  uint32_t lastDroppedSeq() const { return lastDroppedSeq_; }

  const RingStats& stats() const { return stats_; }

private:
  void removeAt(uint16_t n);
  void forget(uint16_t n); // drop logical n without touching flash

  SlotIo& io_;

  uint16_t slotOf_[kRingMaxSlots]; // logical position -> physical slot
  uint32_t seqOf_[kRingMaxSlots];  // logical position -> seq
  uint16_t count_;

  uint32_t nextSeq_;
  uint32_t lastDroppedSeq_;
  RingStats stats_;
};

} // namespace dg

#endif // DG_RING_H
