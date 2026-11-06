// Records waiting for the next upload window.
//
// Uploads are spaced 15.5 s apart but snapshots arrive every 20 s and events
// can arrive back to back, so something has to hold them in between. This is
// that buffer, and it is where the one permitted coalescing rule lives:
//
//   a snapshot may replace the newest pending entry, but only if that entry is
//   itself a snapshot.
//
// That keeps the newest reading without ever dropping an event or reordering
// anything (D39). Everything else is a plain FIFO.
//
// When the buffer fills, push() fails and the caller writes the record to the
// ring instead, so nothing is lost.

#ifndef DG_PENDING_H
#define DG_PENDING_H

#include <stdint.h>

#include "dg_record_codec.h"

namespace dg {

class PendingQueue {
public:
  static constexpr uint8_t kCapacity = 8;

  PendingQueue();

  // coalesced is set when r replaced a pending snapshot rather than being
  // added. Returns false only when the buffer is full.
  bool push(const StoredRecord& r, bool& coalesced);

  bool popOldest(StoredRecord& out);
  bool peekOldest(StoredRecord& out) const;

  uint8_t count() const { return count_; }
  bool empty() const { return count_ == 0; }
  bool full() const { return count_ >= kCapacity; }
  uint32_t coalescedTotal() const { return coalescedTotal_; }

  void clear();

private:
  StoredRecord buf_[kCapacity];
  uint8_t head_; // index of the oldest
  uint8_t count_;
  uint32_t coalescedTotal_;
};

} // namespace dg

#endif // DG_PENDING_H
