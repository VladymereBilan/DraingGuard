#include "fake_slots.h"

namespace dgtest {

FakeSlots::FakeSlots(uint16_t slots)
    : slots_(slots > kMaxSlots ? kMaxSlots : slots),
      data_{},
      writes_(0),
      reads_(0),
      failWrites_(0) {
  eraseAll();
}

void FakeSlots::eraseAll() {
  for (uint16_t i = 0; i < kMaxSlots; ++i) {
    for (uint16_t b = 0; b < dg::kSlotBytes; ++b) {
      data_[i][b] = 0x00;
    }
  }
}

bool FakeSlots::readSlot(uint16_t index, uint8_t out[dg::kSlotBytes]) {
  if (index >= slots_) {
    return false;
  }
  ++reads_;
  for (uint16_t b = 0; b < dg::kSlotBytes; ++b) {
    out[b] = data_[index][b];
  }
  return true;
}

bool FakeSlots::writeSlot(uint16_t index, const uint8_t in[dg::kSlotBytes]) {
  if (index >= slots_) {
    return false;
  }
  if (failWrites_ > 0) {
    --failWrites_;
    return false;
  }
  ++writes_;
  for (uint16_t b = 0; b < dg::kSlotBytes; ++b) {
    data_[index][b] = in[b];
  }
  return true;
}

void FakeSlots::corruptSlot(uint16_t index, uint16_t bitPos) {
  if (index >= slots_) {
    return;
  }
  const uint16_t byteIdx = (bitPos / 8) % dg::kSlotBytes;
  data_[index][byteIdx] =
      static_cast<uint8_t>(data_[index][byteIdx] ^ (1u << (bitPos % 8)));
}

void FakeSlots::blankSlot(uint16_t index) {
  if (index >= slots_) {
    return;
  }
  for (uint16_t b = 0; b < dg::kSlotBytes; ++b) {
    data_[index][b] = 0x00;
  }
}

} // namespace dgtest
