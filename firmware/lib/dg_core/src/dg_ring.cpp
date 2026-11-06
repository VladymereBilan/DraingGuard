#include "dg_ring.h"

namespace dg {

Ring::Ring(SlotIo& io)
    : io_(io),
      slotOf_{},
      seqOf_{},
      count_(0),
      nextSeq_(1),
      lastDroppedSeq_(0),
      stats_{} {}

bool Ring::full() const { return count_ >= io_.slotCount(); }

void Ring::mount(uint32_t lastAckSeq) {
  count_ = 0;
  stats_ = RingStats{};
  lastDroppedSeq_ = lastAckSeq;

  const uint16_t slots = io_.slotCount();
  const uint16_t scanTo = slots < kRingMaxSlots ? slots : kRingMaxSlots;

  uint32_t highestSeq = lastAckSeq;
  uint8_t buf[kSlotBytes];

  for (uint16_t i = 0; i < scanTo; ++i) {
    if (!io_.readSlot(i, buf)) {
      continue;
    }
    if (slotIsBlank(buf)) {
      continue; // never written, not a fault
    }

    StoredRecord sr{};
    if (!decodeSlot(buf, sr)) {
      ++stats_.badSlots; // torn or corrupt: skip it, keep the rest
      continue;
    }
    if (sr.seq == 0) {
      continue;
    }
    if (sr.seq > highestSeq) {
      highestSeq = sr.seq;
    }
    if (sr.seq <= lastAckSeq) {
      ++stats_.ackSkipped; // already delivered before the reboot (D53)
      continue;
    }

    // Insert by seq so logical 0 ends up as the oldest. At most a couple of
    // hundred entries, and this runs once per boot.
    uint16_t pos = count_;
    while (pos > 0 && seqOf_[pos - 1] > sr.seq) {
      seqOf_[pos] = seqOf_[pos - 1];
      slotOf_[pos] = slotOf_[pos - 1];
      --pos;
    }
    seqOf_[pos] = sr.seq;
    slotOf_[pos] = i;
    ++count_;
  }

  nextSeq_ = highestSeq + 1;
  stats_.count = count_;
}

void Ring::forget(uint16_t n) {
  for (uint16_t i = n; i + 1 < count_; ++i) {
    seqOf_[i] = seqOf_[i + 1];
    slotOf_[i] = slotOf_[i + 1];
  }
  --count_;
  stats_.count = count_;
}

void Ring::removeAt(uint16_t n) {
  if (n >= count_) {
    return;
  }
  if (seqOf_[n] > lastDroppedSeq_) {
    lastDroppedSeq_ = seqOf_[n];
  }
  forget(n);
}

bool Ring::append(const StoredRecord& r) {
  const uint16_t slots = io_.slotCount();
  if (slots == 0 || slots > kRingMaxSlots) {
    return false;
  }

  StoredRecord sr = r;
  sr.seq = nextSeq_;
  const uint16_t slot = static_cast<uint16_t>((sr.seq - 1) % slots);

  // Whatever lived in this physical slot is about to be lost. Normally that is
  // the oldest record; after a mount with holes it can be any of them, so look
  // it up rather than assuming.
  for (uint16_t n = 0; n < count_; ++n) {
    if (slotOf_[n] == slot) {
      ++stats_.overwritten;
      if (seqOf_[n] > lastDroppedSeq_) {
        lastDroppedSeq_ = seqOf_[n];
      }
      forget(n);
      break;
    }
  }

  uint8_t buf[kSlotBytes];
  encodeSlot(sr, buf);
  if (!io_.writeSlot(slot, buf)) {
    return false;
  }

  ++nextSeq_;
  seqOf_[count_] = sr.seq;
  slotOf_[count_] = slot;
  ++count_;
  stats_.count = count_;
  return true;
}

bool Ring::readAt(uint16_t n, StoredRecord& out) {
  if (n >= count_) {
    return false;
  }
  uint8_t buf[kSlotBytes];
  if (!io_.readSlot(slotOf_[n], buf)) {
    return false;
  }
  return decodeSlot(buf, out);
}

void Ring::dropOldest(uint16_t howMany) {
  for (uint16_t i = 0; i < howMany && count_ > 0; ++i) {
    removeAt(0);
  }
}

} // namespace dg
