#include "fs_slots.h"

#include <Arduino.h>
#include <LittleFS.h>

namespace hal {
namespace {

constexpr char kPath[] = "/ring.bin";

} // namespace

FsSlots::FsSlots()
    : slots_(0), ready_(false), formatted_(false), ioErrors_(0) {}

bool FsSlots::allocateFile(uint16_t slots) {
  const size_t want = static_cast<size_t>(slots) * dg::kSlotBytes;

  // Reuse the file only when it is exactly the right size; a different size
  // means the capacity changed and the old contents cannot be trusted.
  File probe = LittleFS.open(kPath, "r");
  const bool sized = probe && probe.size() == want;
  if (probe) {
    probe.close();
  }

  if (!sized) {
    File fresh = LittleFS.open(kPath, "w");
    if (!fresh) {
      return false;
    }
    uint8_t blank[dg::kSlotBytes];
    for (uint16_t b = 0; b < dg::kSlotBytes; ++b) {
      blank[b] = 0x00;
    }
    for (uint16_t i = 0; i < slots; ++i) {
      if (fresh.write(blank, dg::kSlotBytes) != dg::kSlotBytes) {
        fresh.close();
        return false;
      }
    }
    fresh.close();
  }

  // "r+" keeps the file open for in-place updates, so an append is a seek and
  // a 32-byte write rather than an open/close pair.
  file_ = LittleFS.open(kPath, "r+");
  return static_cast<bool>(file_);
}

bool FsSlots::begin(uint16_t slots) {
  slots_ = slots;
  ready_ = false;
  formatted_ = false;

  if (!LittleFS.begin(/*formatOnFail=*/false)) {
    if (!LittleFS.begin(/*formatOnFail=*/true)) {
      return false;
    }
    formatted_ = true; // D54: format and carry on
  }

  if (!allocateFile(slots)) {
    return false;
  }
  ready_ = true;
  return true;
}

bool FsSlots::readSlot(uint16_t index, uint8_t out[dg::kSlotBytes]) {
  if (!ready_ || index >= slots_) {
    return false;
  }
  if (!file_.seek(static_cast<uint32_t>(index) * dg::kSlotBytes)) {
    ++ioErrors_;
    return false;
  }
  if (file_.read(out, dg::kSlotBytes) != dg::kSlotBytes) {
    ++ioErrors_;
    return false;
  }
  return true;
}

bool FsSlots::writeSlot(uint16_t index, const uint8_t in[dg::kSlotBytes]) {
  if (!ready_ || index >= slots_) {
    return false;
  }
  if (!file_.seek(static_cast<uint32_t>(index) * dg::kSlotBytes)) {
    ++ioErrors_;
    return false;
  }
  if (file_.write(in, dg::kSlotBytes) != dg::kSlotBytes) {
    ++ioErrors_;
    return false;
  }
  // Push it out now: a slot the ring believes is stored must really be stored.
  file_.flush();
  return true;
}

} // namespace hal
