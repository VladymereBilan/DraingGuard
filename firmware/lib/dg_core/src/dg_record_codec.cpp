#include "dg_record_codec.h"

namespace dg {
namespace {

void put16(uint8_t* p, uint16_t v) {
  p[0] = static_cast<uint8_t>(v & 0xFF);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}

uint16_t get16(const uint8_t* p) {
  return static_cast<uint16_t>(p[0]) | static_cast<uint16_t>(p[1] << 8);
}

void put32(uint8_t* p, uint32_t v) {
  p[0] = static_cast<uint8_t>(v & 0xFF);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
  p[2] = static_cast<uint8_t>((v >> 16) & 0xFF);
  p[3] = static_cast<uint8_t>((v >> 24) & 0xFF);
}

uint32_t get32(const uint8_t* p) {
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) |
         (static_cast<uint32_t>(p[3]) << 24);
}

// Bit-exact float transport without type punning.
uint32_t floatBits(float f) {
  uint32_t bits = 0;
  const uint8_t* src = reinterpret_cast<const uint8_t*>(&f);
  uint8_t* dst = reinterpret_cast<uint8_t*>(&bits);
  for (uint8_t i = 0; i < 4; ++i) {
    dst[i] = src[i];
  }
  return bits;
}

float bitsToFloat(uint32_t bits) {
  float f = 0.0f;
  const uint8_t* src = reinterpret_cast<const uint8_t*>(&bits);
  uint8_t* dst = reinterpret_cast<uint8_t*>(&f);
  for (uint8_t i = 0; i < 4; ++i) {
    dst[i] = src[i];
  }
  return f;
}

} // namespace

uint8_t crc8(const uint8_t* p, uint16_t n) {
  uint8_t crc = 0x00;
  for (uint16_t i = 0; i < n; ++i) {
    crc ^= p[i];
    for (uint8_t b = 0; b < 8; ++b) {
      crc = static_cast<uint8_t>((crc & 0x80) ? ((crc << 1) ^ 0x07)
                                              : (crc << 1));
    }
  }
  return crc;
}

void encodeSlot(const StoredRecord& in, uint8_t out[kSlotBytes]) {
  for (uint16_t i = 0; i < kSlotBytes; ++i) {
    out[i] = 0;
  }

  put16(out + 0, kSlotMagic);
  out[2] = kSlotVersion;
  out[3] = static_cast<uint8_t>((in.rec.uValid ? kFlagUValid : 0) |
                                (in.rec.dValid ? kFlagDValid : 0));
  put32(out + 4, in.seq);
  put32(out + 8, in.uptimeMs);
  put32(out + 12, in.epoch);
  put16(out + 16, in.bootId);
  put16(out + 18, in.rec.cycles);
  put32(out + 20, floatBits(in.rec.uCm));
  put32(out + 24, floatBits(in.rec.dCm));
  out[28] = in.rec.state;
  out[29] = in.rec.fault;
  out[30] = in.rec.event;
  out[31] = crc8(out, kSlotBytes - 1);
}

bool decodeSlot(const uint8_t in[kSlotBytes], StoredRecord& out) {
  if (get16(in + 0) != kSlotMagic) {
    return false;
  }
  if (in[2] != kSlotVersion) {
    return false;
  }
  if (in[31] != crc8(in, kSlotBytes - 1)) {
    return false;
  }

  out.seq = get32(in + 4);
  out.uptimeMs = get32(in + 8);
  out.epoch = get32(in + 12);
  out.bootId = get16(in + 16);

  out.rec.ms = out.uptimeMs;
  out.rec.cycles = get16(in + 18);
  out.rec.uCm = bitsToFloat(get32(in + 20));
  out.rec.dCm = bitsToFloat(get32(in + 24));
  out.rec.diffCm = out.rec.uCm - out.rec.dCm; // derived, never stored
  out.rec.state = in[28];
  out.rec.fault = in[29];
  out.rec.event = in[30];
  out.rec.uValid = (in[3] & kFlagUValid) != 0;
  out.rec.dValid = (in[3] & kFlagDValid) != 0;
  return true;
}

bool slotIsBlank(const uint8_t in[kSlotBytes]) {
  bool allZero = true;
  bool allOnes = true;
  for (uint16_t i = 0; i < kSlotBytes; ++i) {
    if (in[i] != 0x00) {
      allZero = false;
    }
    if (in[i] != 0xFF) {
      allOnes = false;
    }
  }
  return allZero || allOnes;
}

} // namespace dg
