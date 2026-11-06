// RAM-backed dg::SlotIo for the logger tests, with the corruption and
// power-loss injection the ring's recovery path needs.
//
// Test code only - it never ships.

#ifndef DG_TEST_FAKE_SLOTS_H
#define DG_TEST_FAKE_SLOTS_H

#include <stdint.h>

#include "dg_ring.h"

namespace dgtest {

class FakeSlots : public dg::SlotIo {
public:
  static constexpr uint16_t kMaxSlots = 256;

  explicit FakeSlots(uint16_t slots);

  uint16_t slotCount() const override { return slots_; }
  bool readSlot(uint16_t index, uint8_t out[dg::kSlotBytes]) override;
  bool writeSlot(uint16_t index, const uint8_t in[dg::kSlotBytes]) override;

  // Blanks every slot, as a freshly formatted file would be.
  void eraseAll();

  // Flips one bit in a slot - what a torn write looks like after the fact.
  void corruptSlot(uint16_t index, uint16_t bitPos = 0);

  // Zeroes a slot, as if it had never been written.
  void blankSlot(uint16_t index);

  uint32_t writeCount() const { return writes_; }
  uint32_t readCount() const { return reads_; }

  // Makes the next n writes fail, to exercise append()'s error path.
  void failWrites(uint16_t n) { failWrites_ = n; }

private:
  uint16_t slots_;
  uint8_t data_[kMaxSlots][dg::kSlotBytes];
  uint32_t writes_;
  uint32_t reads_;
  uint16_t failWrites_;
};

} // namespace dgtest

#endif // DG_TEST_FAKE_SLOTS_H
