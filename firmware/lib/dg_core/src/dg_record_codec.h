// On-flash slot format for the offline record ring.
//
// A slot is a fixed 32 bytes, little-endian, with a CRC-8 over the first 31
// (D54). Every slot is self-describing - magic, version, a monotonic seq and
// the CRC - so the ring needs no header, and therefore has no header to tear
// when power is lost mid-write. A torn slot simply fails its CRC and is
// skipped on the next mount.
//
// Layout:
//    0  2  magic 0xD6A1
//    2  1  version = 2
//    3  1  flags: bit0 uValid, bit1 dValid, rest reserved (0)
//    4  4  seq        monotonic, assigned on append; 0 means never written
//    8  4  uptimeMs   millis() when the record was made
//   12  4  epoch      UTC seconds, 0 = not known yet (D8)
//   16  2  bootId
//   18  2  cycles     ThingSpeak field 5
//   20  4  uCm        float
//   24  4  dCm        float
//   28  1  state
//   29  1  fault
//   30  1  event
//   31  1  crc8 over bytes 0..30
//
// diffCm is not stored: it is always uCm - dCm, so decode recomputes it. That
// is what buys room for both levels at full float precision inside 32 bytes.

#ifndef DG_RECORD_CODEC_H
#define DG_RECORD_CODEC_H

#include <stdint.h>

#include "dg_controller.h"

namespace dg {

inline constexpr uint16_t kSlotBytes = 32;
inline constexpr uint16_t kSlotMagic = 0xD6A1;
// Version 2 added the validity flags. A version-1 slot is rejected rather than
// reinterpreted, because its flags byte was always zero and would decode as
// "both sensors invalid" - a silent lie about data we cannot verify.
inline constexpr uint8_t kSlotVersion = 2;

inline constexpr uint8_t kFlagUValid = 0x01;
inline constexpr uint8_t kFlagDValid = 0x02;

// A Record plus the provenance D8 requires to date it later.
struct StoredRecord {
  uint32_t seq;
  uint32_t uptimeMs;
  uint32_t epoch; // 0 = not yet known
  uint16_t bootId;
  Record rec;
};

// CRC-8/ATM: polynomial 0x07, init 0x00, no reflection, no final xor.
uint8_t crc8(const uint8_t* p, uint16_t n);

// Levels beyond the float range are not clamped; the sensor range makes that
// unreachable. Always succeeds.
void encodeSlot(const StoredRecord& in, uint8_t out[kSlotBytes]);

// False when the magic, version or CRC do not check out - a blank or torn slot.
bool decodeSlot(const uint8_t in[kSlotBytes], StoredRecord& out);

// True for a slot that has never been written: all 0x00 or all 0xFF. Lets the
// ring tell "empty" apart from "corrupt" when it counts bad slots.
bool slotIsBlank(const uint8_t in[kSlotBytes]);

} // namespace dg

#endif // DG_RECORD_CODEC_H
