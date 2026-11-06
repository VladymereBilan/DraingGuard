#include "dg_pending.h"

namespace dg {

PendingQueue::PendingQueue()
    : buf_{}, head_(0), count_(0), coalescedTotal_(0) {}

bool PendingQueue::newestIsSnapshot() const {
  if (count_ == 0) {
    return false;
  }
  const uint8_t newest =
      static_cast<uint8_t>((head_ + count_ - 1) % kCapacity);
  return buf_[newest].rec.event == kEventSnapshot;
}

bool PendingQueue::push(const StoredRecord& r, bool allowCoalesce,
                        bool& coalesced) {
  coalesced = false;

  // Only a snapshot may absorb another snapshot, only the newest one, and only
  // when the caller says the link state permits it (D65).
  if (allowCoalesce && r.rec.event == kEventSnapshot && newestIsSnapshot()) {
    const uint8_t newest =
        static_cast<uint8_t>((head_ + count_ - 1) % kCapacity);
    buf_[newest] = r;
    coalesced = true;
    ++coalescedTotal_;
    return true;
  }

  if (count_ >= kCapacity) {
    return false; // caller rings it instead
  }

  const uint8_t slot = static_cast<uint8_t>((head_ + count_) % kCapacity);
  buf_[slot] = r;
  ++count_;
  return true;
}

bool PendingQueue::popOldest(StoredRecord& out) {
  if (count_ == 0) {
    return false;
  }
  out = buf_[head_];
  head_ = static_cast<uint8_t>((head_ + 1) % kCapacity);
  --count_;
  return true;
}

bool PendingQueue::peekOldest(StoredRecord& out) const {
  if (count_ == 0) {
    return false;
  }
  out = buf_[head_];
  return true;
}

void PendingQueue::clear() {
  head_ = 0;
  count_ = 0;
}

} // namespace dg
