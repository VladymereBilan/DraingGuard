// S4a logger-logic tests. Everything here runs on the host: the ring's slot
// store is a RAM fake, so the crash-recovery and acknowledgement paths are
// exercised without a filesystem, a network or a board.
//
// No real credentials appear anywhere in this file (D58).

#include "tests_logger.h"

#include <unity.h>

#include "dg_bulk_json.h"
#include "dg_pending.h"
#include "dg_record_codec.h"
#include "dg_ring.h"
#include "dg_timestamp.h"
#include "fake_slots.h"

namespace dgtest {
namespace {

// Placeholder stands in for the write API key everywhere (D58).
constexpr char kWriteKey[] = "<WRITE_KEY>";

// 2026-11-06T00:00:00Z. Verified against the calendar, not assumed.
constexpr uint32_t kBaseEpoch = 1793923200u;

// A 60-entry batch (the D50 cap) needs more than the 8 KB that decision names:
// a typical entry is ~137 bytes and the worst case ~158, so 60 of them run to
// 8.3-9.5 KB plus the envelope. 10 KB covers the worst case with headroom.
// Flagged for D50; the production buffer is sized in S4b.
constexpr uint16_t kBulkBufBytes = 10240;
char g_bulkBuf[kBulkBufBytes];

dg::StoredRecord makeRec(uint16_t bootId, uint32_t uptimeMs, uint32_t epoch,
                         uint8_t state, uint8_t event, uint16_t cycles,
                         float uCm, float dCm, uint8_t fault = 0) {
  dg::StoredRecord r{};
  r.seq = 0; // assigned by Ring::append
  r.uptimeMs = uptimeMs;
  r.epoch = epoch;
  r.bootId = bootId;
  r.rec.ms = uptimeMs;
  r.rec.uCm = uCm;
  r.rec.dCm = dCm;
  r.rec.diffCm = uCm - dCm;
  r.rec.state = state;
  r.rec.cycles = cycles;
  r.rec.fault = fault;
  r.rec.event = event;
  return r;
}

dg::StoredRecord simpleRec(uint32_t uptimeMs) {
  return makeRec(1, uptimeMs, 0, dg::kFaultNone, dg::kEventSnapshot, 0, 13.0f,
                 10.0f);
}

uint16_t countOccurrences(const char* hay, const char* needle) {
  uint16_t n = 0;
  for (const char* p = hay; *p != '\0'; ++p) {
    const char* a = p;
    const char* b = needle;
    while (*b != '\0' && *a == *b) {
      ++a;
      ++b;
    }
    if (*b == '\0') {
      ++n;
    }
  }
  return n;
}

bool startsWith(const char* s, const char* prefix) {
  while (*prefix != '\0') {
    if (*s++ != *prefix++) {
      return false;
    }
  }
  return true;
}

bool endsWith(const char* s, const char* suffix) {
  uint16_t sl = 0;
  uint16_t fl = 0;
  while (s[sl] != '\0') {
    ++sl;
  }
  while (suffix[fl] != '\0') {
    ++fl;
  }
  if (fl > sl) {
    return false;
  }
  for (uint16_t i = 0; i < fl; ++i) {
    if (s[sl - fl + i] != suffix[i]) {
      return false;
    }
  }
  return true;
}

// Structural check: the right envelope, balanced braces, no empty slots.
void assertValidBulk(const char* body, uint16_t expectedEntries) {
  TEST_ASSERT_TRUE(
      startsWith(body, "{\"write_api_key\":\"<WRITE_KEY>\",\"updates\":["));
  TEST_ASSERT_TRUE(endsWith(body, "]}"));
  TEST_ASSERT_EQUAL_UINT16(expectedEntries,
                           countOccurrences(body, "\"created_at\""));
  TEST_ASSERT_EQUAL_UINT16(expectedEntries, countOccurrences(body, "\"field8\""));
  TEST_ASSERT_EQUAL_UINT16(0, countOccurrences(body, ",]"));
  TEST_ASSERT_EQUAL_UINT16(0, countOccurrences(body, "[,"));
  TEST_ASSERT_EQUAL_UINT16(0, countOccurrences(body, ",,"));

  int depth = 0;
  for (const char* p = body; *p != '\0'; ++p) {
    if (*p == '{') {
      ++depth;
    } else if (*p == '}') {
      --depth;
      TEST_ASSERT_TRUE(depth >= 0);
    }
  }
  TEST_ASSERT_EQUAL_INT(0, depth);
}

} // namespace

// 1 ------------------------------------------------------------------------
void test_l1_slot_round_trip() {
  dg::StoredRecord in = makeRec(/*bootId=*/0x1234, /*uptimeMs=*/123456789u,
                                /*epoch=*/1762400096u, dg::kFaultSensor,
                                dg::kEventJamRetryCleared, /*cycles=*/4321,
                                /*uCm=*/13.25f, /*dCm=*/10.75f,
                                dg::kFaultJamClearedByRetry);
  in.seq = 0x00ABCDEFu;
  in.rec.state = 6;

  uint8_t slot[dg::kSlotBytes];
  dg::encodeSlot(in, slot);

  dg::StoredRecord out{};
  TEST_ASSERT_TRUE(dg::decodeSlot(slot, out));

  TEST_ASSERT_EQUAL_UINT32(in.seq, out.seq);
  TEST_ASSERT_EQUAL_UINT32(in.uptimeMs, out.uptimeMs);
  TEST_ASSERT_EQUAL_UINT32(in.epoch, out.epoch);
  TEST_ASSERT_EQUAL_UINT16(in.bootId, out.bootId);
  TEST_ASSERT_EQUAL_UINT16(in.rec.cycles, out.rec.cycles);
  TEST_ASSERT_EQUAL_UINT8(in.rec.state, out.rec.state);
  TEST_ASSERT_EQUAL_UINT8(in.rec.fault, out.rec.fault);
  TEST_ASSERT_EQUAL_UINT8(in.rec.event, out.rec.event);
  TEST_ASSERT_EQUAL_FLOAT(in.rec.uCm, out.rec.uCm);
  TEST_ASSERT_EQUAL_FLOAT(in.rec.dCm, out.rec.dCm);
  // diffCm is derived on decode, not stored.
  TEST_ASSERT_EQUAL_FLOAT(in.rec.uCm - in.rec.dCm, out.rec.diffCm);
  TEST_ASSERT_FALSE(dg::slotIsBlank(slot));
}

// 2 ------------------------------------------------------------------------
void test_l2_flipped_bit_fails_crc() {
  const dg::StoredRecord in = simpleRec(1000);
  uint8_t good[dg::kSlotBytes];
  dg::encodeSlot(in, good);

  // Every single-bit flip in the slot must be caught.
  for (uint16_t bit = 0; bit < dg::kSlotBytes * 8; ++bit) {
    uint8_t slot[dg::kSlotBytes];
    for (uint16_t i = 0; i < dg::kSlotBytes; ++i) {
      slot[i] = good[i];
    }
    slot[bit / 8] = static_cast<uint8_t>(slot[bit / 8] ^ (1u << (bit % 8)));

    dg::StoredRecord out{};
    TEST_ASSERT_FALSE(dg::decodeSlot(slot, out));
  }
}

// 3 ------------------------------------------------------------------------
void test_l3_mount_blank_store() {
  FakeSlots io(8);
  dg::Ring ring(io);
  ring.mount(0);

  TEST_ASSERT_EQUAL_UINT16(0, ring.count());
  TEST_ASSERT_TRUE(ring.empty());
  TEST_ASSERT_EQUAL_UINT32(1, ring.nextSeq());
  TEST_ASSERT_EQUAL_UINT16(0, ring.stats().badSlots);
  TEST_ASSERT_EQUAL_UINT32(0, ring.stats().ackSkipped);
}

// 4 ------------------------------------------------------------------------
void test_l4_append_then_oldest_first() {
  FakeSlots io(8);
  dg::Ring ring(io);
  ring.mount(0);

  TEST_ASSERT_TRUE(ring.append(simpleRec(100)));
  TEST_ASSERT_TRUE(ring.append(simpleRec(200)));
  TEST_ASSERT_TRUE(ring.append(simpleRec(300)));
  TEST_ASSERT_EQUAL_UINT16(3, ring.count());

  dg::StoredRecord r{};
  TEST_ASSERT_TRUE(ring.readAt(0, r));
  TEST_ASSERT_EQUAL_UINT32(100, r.uptimeMs);
  TEST_ASSERT_EQUAL_UINT32(1, r.seq);
  TEST_ASSERT_TRUE(ring.readAt(2, r));
  TEST_ASSERT_EQUAL_UINT32(300, r.uptimeMs);
  TEST_ASSERT_EQUAL_UINT32(3, r.seq);
  TEST_ASSERT_FALSE(ring.readAt(3, r));
}

// 5 ------------------------------------------------------------------------
void test_l5_overwrite_counts_as_dropped() {
  FakeSlots io(4);
  dg::Ring ring(io);
  ring.mount(0);

  for (uint32_t i = 1; i <= 6; ++i) {
    TEST_ASSERT_TRUE(ring.append(simpleRec(i * 100)));
  }

  TEST_ASSERT_EQUAL_UINT16(4, ring.count());
  TEST_ASSERT_TRUE(ring.full());
  TEST_ASSERT_EQUAL_UINT32(2, ring.stats().overwritten);
  TEST_ASSERT_EQUAL_UINT32(2, ring.lastDroppedSeq());

  // The two oldest are gone; 3..6 remain, oldest first.
  dg::StoredRecord r{};
  for (uint16_t n = 0; n < 4; ++n) {
    TEST_ASSERT_TRUE(ring.readAt(n, r));
    TEST_ASSERT_EQUAL_UINT32(static_cast<uint32_t>(n + 3), r.seq);
  }
}

// 6 ------------------------------------------------------------------------
void test_l6_mount_rebuilds_after_reboot() {
  FakeSlots io(8);
  {
    dg::Ring ring(io);
    ring.mount(0);
    ring.append(simpleRec(100));
    ring.append(simpleRec(200));
    ring.append(simpleRec(300));
  }

  // Fresh Ring over the same flash: nothing but the slots survives.
  dg::Ring reborn(io);
  reborn.mount(0);

  TEST_ASSERT_EQUAL_UINT16(3, reborn.count());
  TEST_ASSERT_EQUAL_UINT32(4, reborn.nextSeq());
  TEST_ASSERT_EQUAL_UINT16(0, reborn.stats().badSlots);

  dg::StoredRecord r{};
  TEST_ASSERT_TRUE(reborn.readAt(0, r));
  TEST_ASSERT_EQUAL_UINT32(100, r.uptimeMs);
  TEST_ASSERT_TRUE(reborn.readAt(2, r));
  TEST_ASSERT_EQUAL_UINT32(300, r.uptimeMs);
}

// 7 ------------------------------------------------------------------------
void test_l7_mount_across_slot_wrap() {
  FakeSlots io(4);
  {
    dg::Ring ring(io);
    ring.mount(0);
    for (uint32_t i = 1; i <= 10; ++i) {
      ring.append(simpleRec(i * 100));
    }
  }

  // Seqs 7..10 survive, and they are not in slot order - the mount scan has to
  // sort them by seq rather than trusting the physical layout.
  dg::Ring reborn(io);
  reborn.mount(0);

  TEST_ASSERT_EQUAL_UINT16(4, reborn.count());
  TEST_ASSERT_EQUAL_UINT32(11, reborn.nextSeq());

  dg::StoredRecord r{};
  for (uint16_t n = 0; n < 4; ++n) {
    TEST_ASSERT_TRUE(reborn.readAt(n, r));
    TEST_ASSERT_EQUAL_UINT32(static_cast<uint32_t>(n + 7), r.seq);
    TEST_ASSERT_EQUAL_UINT32((n + 7) * 100u, r.uptimeMs);
  }
}

// 8 ------------------------------------------------------------------------
void test_l8_mount_skips_torn_slot() {
  FakeSlots io(8);
  {
    dg::Ring ring(io);
    ring.mount(0);
    for (uint32_t i = 1; i <= 5; ++i) {
      ring.append(simpleRec(i * 100));
    }
  }

  // Seq 3 lives in slot 2. Tear it, as a power cut mid-write would.
  io.corruptSlot(2, 5);

  dg::Ring reborn(io);
  reborn.mount(0);

  TEST_ASSERT_EQUAL_UINT16(4, reborn.count());
  TEST_ASSERT_EQUAL_UINT16(1, reborn.stats().badSlots);
  TEST_ASSERT_EQUAL_UINT32(6, reborn.nextSeq());

  // 1, 2, 4, 5 remain - the hole does not disturb the order of the rest.
  const uint32_t expected[4] = {1, 2, 4, 5};
  dg::StoredRecord r{};
  for (uint16_t n = 0; n < 4; ++n) {
    TEST_ASSERT_TRUE(reborn.readAt(n, r));
    TEST_ASSERT_EQUAL_UINT32(expected[n], r.seq);
  }
}

// 9 ------------------------------------------------------------------------
void test_l9_drop_oldest_advances_tail_only() {
  FakeSlots io(8);
  dg::Ring ring(io);
  ring.mount(0);
  for (uint32_t i = 1; i <= 5; ++i) {
    ring.append(simpleRec(i * 100));
  }

  ring.dropOldest(2);

  TEST_ASSERT_EQUAL_UINT16(3, ring.count());
  TEST_ASSERT_EQUAL_UINT32(2, ring.lastDroppedSeq());
  TEST_ASSERT_EQUAL_UINT32(0, ring.stats().overwritten); // a drop is not an overwrite

  dg::StoredRecord r{};
  TEST_ASSERT_TRUE(ring.readAt(0, r));
  TEST_ASSERT_EQUAL_UINT32(3, r.seq);
  TEST_ASSERT_TRUE(ring.readAt(2, r));
  TEST_ASSERT_EQUAL_UINT32(5, r.seq); // newest untouched
}

// 10 -----------------------------------------------------------------------
void test_l10_backfill_current_boot_synced() {
  dg::StoredRecord r = makeRec(7, /*uptimeMs=*/1000, /*epoch=*/0, 0,
                               dg::kEventSnapshot, 0, 13.0f, 10.0f);

  const dg::EpochFate fate =
      dg::resolveEpoch(r, /*currentBootId=*/7, /*nowUptimeMs=*/11000,
                       /*nowEpoch=*/kBaseEpoch, /*timeSynced=*/true);

  TEST_ASSERT_TRUE(fate == dg::EpochFate::BACKFILLED);
  TEST_ASSERT_EQUAL_UINT32(kBaseEpoch - 10u, r.epoch);
}

// 11 -----------------------------------------------------------------------
void test_l11_wait_when_not_synced() {
  dg::StoredRecord r = makeRec(7, 1000, 0, 0, dg::kEventSnapshot, 0, 13.0f,
                               10.0f);

  const dg::EpochFate fate =
      dg::resolveEpoch(r, 7, 11000, kBaseEpoch, /*timeSynced=*/false);

  TEST_ASSERT_TRUE(fate == dg::EpochFate::WAIT);
  TEST_ASSERT_EQUAL_UINT32(0, r.epoch); // left for a later attempt

  // Time "synced" but no epoch yet is the same situation.
  const dg::EpochFate fate2 = dg::resolveEpoch(r, 7, 11000, 0u, true);
  TEST_ASSERT_TRUE(fate2 == dg::EpochFate::WAIT);
}

// 12 -----------------------------------------------------------------------
void test_l12_drop_old_boot_undated() {
  dg::StoredRecord r = makeRec(/*bootId=*/6, 1000, /*epoch=*/0, 0,
                               dg::kEventSnapshot, 0, 13.0f, 10.0f);

  const dg::EpochFate fate =
      dg::resolveEpoch(r, /*currentBootId=*/7, 11000, kBaseEpoch, true);

  TEST_ASSERT_TRUE(fate == dg::EpochFate::DROP_OLD_BOOT);
  TEST_ASSERT_EQUAL_UINT32(0, r.epoch);
}

// 13 -----------------------------------------------------------------------
void test_l13_old_boot_already_dated() {
  dg::StoredRecord r = makeRec(/*bootId=*/6, 1000, /*epoch=*/(kBaseEpoch - 100000u), 0,
                               dg::kEventSnapshot, 0, 13.0f, 10.0f);

  const dg::EpochFate fate =
      dg::resolveEpoch(r, /*currentBootId=*/7, 11000, kBaseEpoch, true);

  TEST_ASSERT_TRUE(fate == dg::EpochFate::ALREADY_SET);
  TEST_ASSERT_EQUAL_UINT32((kBaseEpoch - 100000u), r.epoch); // untouched
}

// 14 -----------------------------------------------------------------------
void test_l14_backfill_across_millis_rollover() {
  // Record made 8096 ms ago, with millis() having wrapped in between.
  dg::StoredRecord r = makeRec(3, /*uptimeMs=*/0xFFFFF000u, 0, 0,
                               dg::kEventSnapshot, 0, 13.0f, 10.0f);

  const dg::EpochFate fate =
      dg::resolveEpoch(r, 3, /*nowUptimeMs=*/4000u, kBaseEpoch, true);

  TEST_ASSERT_TRUE(fate == dg::EpochFate::BACKFILLED);
  TEST_ASSERT_EQUAL_UINT32(kBaseEpoch - 8u, r.epoch);
}

// 15 -----------------------------------------------------------------------
void test_l15_iso8601_known_epochs() {
  char buf[dg::kIso8601Bytes];

  TEST_ASSERT_FALSE(dg::formatIso8601Utc(0, buf)); // 0 means "unknown"

  TEST_ASSERT_TRUE(dg::formatIso8601Utc(1u, buf));
  TEST_ASSERT_EQUAL_STRING("1970-01-01T00:00:01Z", buf);

  TEST_ASSERT_TRUE(dg::formatIso8601Utc(946684800u, buf));
  TEST_ASSERT_EQUAL_STRING("2000-01-01T00:00:00Z", buf);

  TEST_ASSERT_TRUE(dg::formatIso8601Utc(946688523u, buf));
  TEST_ASSERT_EQUAL_STRING("2000-01-01T01:02:03Z", buf);

  // Leap day, which a naive 365-day calculation gets wrong.
  TEST_ASSERT_TRUE(dg::formatIso8601Utc(951782400u, buf));
  TEST_ASSERT_EQUAL_STRING("2000-02-29T00:00:00Z", buf);

  // Last second of 2025, straight into a year boundary.
  TEST_ASSERT_TRUE(dg::formatIso8601Utc(1767225599u, buf));
  TEST_ASSERT_EQUAL_STRING("2025-12-31T23:59:59Z", buf);

  TEST_ASSERT_TRUE(dg::formatIso8601Utc(1767225600u, buf));
  TEST_ASSERT_EQUAL_STRING("2026-01-01T00:00:00Z", buf);
}

// 16 -----------------------------------------------------------------------
void test_l16_bulk_body_entry_counts() {
  const uint32_t base = kBaseEpoch;

  // One entry.
  {
    dg::BulkBuilder b(g_bulkBuf, kBulkBufBytes, kWriteKey);
    dg::StoredRecord r = makeRec(1, 1000, base, 2, dg::kEventClearedVerified,
                                 4, 13.25f, 10.75f,
                                 dg::kFaultJamClearedByRetry);
    TEST_ASSERT_TRUE(b.addEntry(r, /*droppedTotal=*/7));
    TEST_ASSERT_TRUE(b.finish());
    TEST_ASSERT_EQUAL_UINT16(1, b.entries());
    assertValidBulk(b.body(), 1);

    // Spot-check the values that go on the wire.
    TEST_ASSERT_EQUAL_UINT16(
        1, countOccurrences(b.body(), "\"created_at\":\"2026-11-06T00:00:00Z\""));
    TEST_ASSERT_EQUAL_UINT16(1, countOccurrences(b.body(), "\"field1\":13.25"));
    TEST_ASSERT_EQUAL_UINT16(1, countOccurrences(b.body(), "\"field2\":10.75"));
    TEST_ASSERT_EQUAL_UINT16(1, countOccurrences(b.body(), "\"field3\":2.50"));
    TEST_ASSERT_EQUAL_UINT16(1, countOccurrences(b.body(), "\"field4\":2"));
    TEST_ASSERT_EQUAL_UINT16(1, countOccurrences(b.body(), "\"field5\":4"));
    TEST_ASSERT_EQUAL_UINT16(1, countOccurrences(b.body(), "\"field6\":1"));
    TEST_ASSERT_EQUAL_UINT16(1, countOccurrences(b.body(), "\"field7\":1"));
    TEST_ASSERT_EQUAL_UINT16(1, countOccurrences(b.body(), "\"field8\":7"));
  }

  // Two entries.
  {
    dg::BulkBuilder b(g_bulkBuf, kBulkBufBytes, kWriteKey);
    TEST_ASSERT_TRUE(b.addEntry(
        makeRec(1, 1000, base, 0, dg::kEventSnapshot, 0, 10.0f, 10.0f), 0));
    TEST_ASSERT_TRUE(b.addEntry(
        makeRec(1, 21000, base + 20, 0, dg::kEventSnapshot, 0, 11.0f, 10.0f),
        0));
    TEST_ASSERT_TRUE(b.finish());
    TEST_ASSERT_EQUAL_UINT16(2, b.entries());
    assertValidBulk(b.body(), 2);
  }

  // A full 60-entry batch, the D50 cap.
  {
    dg::BulkBuilder b(g_bulkBuf, kBulkBufBytes, kWriteKey);
    for (uint16_t i = 0; i < 60; ++i) {
      const dg::StoredRecord r =
          makeRec(1, 1000u + i * 20000u, base + i * 20u, 0, dg::kEventSnapshot,
                  i, 10.0f + static_cast<float>(i) * 0.01f, 10.0f);
      TEST_ASSERT_TRUE(b.addEntry(r, 0));
    }
    TEST_ASSERT_TRUE(b.finish());
    TEST_ASSERT_EQUAL_UINT16(60, b.entries());
    TEST_ASSERT_FALSE(b.overflowed());
    TEST_ASSERT_TRUE(b.length() < kBulkBufBytes);
    // Record what a full batch actually costs, so S4b can size the real
    // buffer from a measurement rather than an estimate.
    TEST_ASSERT_TRUE(b.length() > 8192);
    assertValidBulk(b.body(), 60);
  }
}

// 17 -----------------------------------------------------------------------
void test_l17_bulk_body_overflow_stays_valid() {
  // Deliberately too small for ten entries.
  constexpr uint16_t kTiny = 320;
  char tiny[kTiny];

  dg::BulkBuilder b(tiny, kTiny, kWriteKey);
  uint16_t accepted = 0;
  uint16_t refused = 0;
  for (uint16_t i = 0; i < 10; ++i) {
    const dg::StoredRecord r =
        makeRec(1, 1000u + i * 1000u, kBaseEpoch + i, 0, dg::kEventSnapshot, i,
                13.0f, 10.0f);
    if (b.addEntry(r, 0)) {
      ++accepted;
    } else {
      ++refused;
    }
  }

  TEST_ASSERT_TRUE(accepted >= 1);
  TEST_ASSERT_TRUE(refused >= 1);
  TEST_ASSERT_TRUE(b.overflowed());
  TEST_ASSERT_EQUAL_UINT16(accepted, b.entries());

  // The refused entries left no partial JSON behind.
  TEST_ASSERT_TRUE(b.finish());
  assertValidBulk(b.body(), accepted);
}

// 18 -----------------------------------------------------------------------
void test_l18_duplicate_timestamps_bumped() {
  const uint32_t same = kBaseEpoch;

  dg::BulkBuilder b(g_bulkBuf, kBulkBufBytes, kWriteKey);
  for (uint16_t i = 0; i < 3; ++i) {
    TEST_ASSERT_TRUE(b.addEntry(
        makeRec(1, 1000u + i, same, 0, dg::kEventSnapshot, i, 13.0f, 10.0f),
        0));
  }
  TEST_ASSERT_TRUE(b.finish());

  // One collision would make ThingSpeak reject the whole batch, so each repeat
  // is nudged forward by a second (D49).
  TEST_ASSERT_EQUAL_UINT32(same + 2u, b.lastEpochUsed());
  assertValidBulk(b.body(), 3);

  char s0[dg::kIso8601Bytes];
  char s1[dg::kIso8601Bytes];
  char s2[dg::kIso8601Bytes];
  dg::formatIso8601Utc(same, s0);
  dg::formatIso8601Utc(same + 1u, s1);
  dg::formatIso8601Utc(same + 2u, s2);
  TEST_ASSERT_EQUAL_UINT16(1, countOccurrences(b.body(), s0));
  TEST_ASSERT_EQUAL_UINT16(1, countOccurrences(b.body(), s1));
  TEST_ASSERT_EQUAL_UINT16(1, countOccurrences(b.body(), s2));

  // An already-ordered stream is left alone.
  dg::BulkBuilder c(g_bulkBuf, kBulkBufBytes, kWriteKey);
  TEST_ASSERT_TRUE(c.addEntry(
      makeRec(1, 1000, same, 0, dg::kEventSnapshot, 0, 13.0f, 10.0f), 0));
  TEST_ASSERT_TRUE(c.addEntry(
      makeRec(1, 21000, same + 20u, 0, dg::kEventSnapshot, 0, 13.0f, 10.0f),
      0));
  TEST_ASSERT_EQUAL_UINT32(same + 20u, c.lastEpochUsed());
}

// 19 -----------------------------------------------------------------------
void test_l19_update_body_fields_and_encoding() {
  char body[256];
  const dg::StoredRecord r =
      makeRec(1, 1000, kBaseEpoch, 3, dg::kEventFault, 9, 13.25f, 10.75f,
              dg::kFaultStillBlocked);

  const uint16_t len = dg::buildUpdateBody(body, sizeof(body), kWriteKey, r,
                                           /*droppedTotal=*/12);
  TEST_ASSERT_TRUE(len > 0);

  // Reserved characters are percent-encoded, so the colons in the timestamp
  // and the angle brackets in the placeholder key come out escaped.
  TEST_ASSERT_TRUE(startsWith(body, "api_key=%3CWRITE_KEY%3E&created_at="));
  TEST_ASSERT_EQUAL_UINT16(
      1, countOccurrences(body, "created_at=2026-11-06T00%3A00%3A00Z"));
  TEST_ASSERT_EQUAL_UINT16(0, countOccurrences(body, ":"));

  // Fields in order 1..8.
  TEST_ASSERT_EQUAL_UINT16(1, countOccurrences(body, "&field1=13.25"));
  TEST_ASSERT_EQUAL_UINT16(1, countOccurrences(body, "&field2=10.75"));
  TEST_ASSERT_EQUAL_UINT16(1, countOccurrences(body, "&field3=2.50"));
  TEST_ASSERT_EQUAL_UINT16(1, countOccurrences(body, "&field4=3"));
  TEST_ASSERT_EQUAL_UINT16(1, countOccurrences(body, "&field5=9"));
  TEST_ASSERT_EQUAL_UINT16(1, countOccurrences(body, "&field6=3"));
  TEST_ASSERT_EQUAL_UINT16(1, countOccurrences(body, "&field7=2"));
  TEST_ASSERT_EQUAL_UINT16(1, countOccurrences(body, "&field8=12"));

  // An undated record cannot be sent.
  dg::StoredRecord undated = r;
  undated.epoch = 0;
  TEST_ASSERT_EQUAL_UINT16(
      0, dg::buildUpdateBody(body, sizeof(body), kWriteKey, undated, 0));

  // Too small a buffer fails cleanly rather than truncating.
  char small[32];
  TEST_ASSERT_EQUAL_UINT16(
      0, dg::buildUpdateBody(small, sizeof(small), kWriteKey, r, 0));
}

// 20 -----------------------------------------------------------------------
void test_l20_snapshot_replaces_adjacent_snapshot() {
  dg::PendingQueue q;
  bool coalesced = false;

  TEST_ASSERT_TRUE(q.push(simpleRec(1000), coalesced));
  TEST_ASSERT_FALSE(coalesced);

  TEST_ASSERT_TRUE(q.push(simpleRec(21000), coalesced));
  TEST_ASSERT_TRUE(coalesced); // only the newest snapshot is worth keeping
  TEST_ASSERT_EQUAL_UINT8(1, q.count());
  TEST_ASSERT_EQUAL_UINT32(1, q.coalescedTotal());

  dg::StoredRecord out{};
  TEST_ASSERT_TRUE(q.popOldest(out));
  TEST_ASSERT_EQUAL_UINT32(21000, out.uptimeMs);
  TEST_ASSERT_TRUE(q.empty());
}

// 21 -----------------------------------------------------------------------
void test_l21_snapshot_after_event_appends() {
  dg::PendingQueue q;
  bool coalesced = false;

  dg::StoredRecord ev = makeRec(1, 1000, 0, 4, dg::kEventFault, 0, 13.0f, 10.0f,
                                dg::kFaultStillBlocked);
  TEST_ASSERT_TRUE(q.push(ev, coalesced));
  TEST_ASSERT_FALSE(coalesced);

  TEST_ASSERT_TRUE(q.push(simpleRec(21000), coalesced));
  TEST_ASSERT_FALSE(coalesced); // an event is never absorbed
  TEST_ASSERT_EQUAL_UINT8(2, q.count());

  dg::StoredRecord out{};
  TEST_ASSERT_TRUE(q.popOldest(out));
  TEST_ASSERT_EQUAL_UINT8(dg::kEventFault, out.rec.event);
  TEST_ASSERT_TRUE(q.popOldest(out));
  TEST_ASSERT_EQUAL_UINT8(dg::kEventSnapshot, out.rec.event);
}

// 22 -----------------------------------------------------------------------
void test_l22_events_keep_order() {
  dg::PendingQueue q;
  bool coalesced = false;

  const uint8_t script[6] = {
      dg::kEventBinEmptied,       dg::kEventSnapshot,
      dg::kEventClearedVerified,  dg::kEventSnapshot,
      dg::kEventFault,            dg::kEventFaultRecovered,
  };
  for (uint8_t i = 0; i < 6; ++i) {
    dg::StoredRecord r = makeRec(1, 1000u + i * 100u, 0, 0, script[i], i, 13.0f,
                                 10.0f);
    TEST_ASSERT_TRUE(q.push(r, coalesced));
    TEST_ASSERT_FALSE(coalesced); // no two snapshots are ever adjacent here
  }
  TEST_ASSERT_EQUAL_UINT8(6, q.count());

  dg::StoredRecord out{};
  for (uint8_t i = 0; i < 6; ++i) {
    TEST_ASSERT_TRUE(q.popOldest(out));
    TEST_ASSERT_EQUAL_UINT8(script[i], out.rec.event);
    TEST_ASSERT_EQUAL_UINT32(1000u + i * 100u, out.uptimeMs);
  }
}

// 23 -----------------------------------------------------------------------
void test_l23_full_queue_refuses_push() {
  dg::PendingQueue q;
  bool coalesced = false;

  for (uint8_t i = 0; i < dg::PendingQueue::kCapacity; ++i) {
    dg::StoredRecord r =
        makeRec(1, 1000u + i, 0, 0, dg::kEventFault, i, 13.0f, 10.0f);
    TEST_ASSERT_TRUE(q.push(r, coalesced));
  }
  TEST_ASSERT_TRUE(q.full());

  dg::StoredRecord extra =
      makeRec(1, 9999, 0, 0, dg::kEventFault, 0, 13.0f, 10.0f);
  TEST_ASSERT_FALSE(q.push(extra, coalesced)); // caller rings it instead
  TEST_ASSERT_EQUAL_UINT8(dg::PendingQueue::kCapacity, q.count());

  // Popping one makes room again.
  dg::StoredRecord out{};
  TEST_ASSERT_TRUE(q.popOldest(out));
  TEST_ASSERT_TRUE(q.push(extra, coalesced));
}

// 24 -----------------------------------------------------------------------
void test_l24_dropped_accounting() {
  // Ring overwrites are one source of the field-8 total.
  FakeSlots io(4);
  dg::Ring ring(io);
  ring.mount(0);
  for (uint32_t i = 1; i <= 7; ++i) {
    TEST_ASSERT_TRUE(ring.append(simpleRec(i * 100)));
  }
  TEST_ASSERT_EQUAL_UINT32(3, ring.stats().overwritten);

  // Undatable records from a previous boot are the other source (D8).
  uint32_t oldBootDrops = 0;
  for (uint16_t i = 0; i < 2; ++i) {
    dg::StoredRecord r =
        makeRec(/*bootId=*/5, 1000u + i, 0, 0, dg::kEventSnapshot, 0, 13.0f,
                10.0f);
    if (dg::resolveEpoch(r, /*currentBootId=*/6, 5000, kBaseEpoch, true) ==
        dg::EpochFate::DROP_OLD_BOOT) {
      ++oldBootDrops;
    }
  }
  TEST_ASSERT_EQUAL_UINT32(2, oldBootDrops);

  // The logger sums these (plus queue-full, which arrives with S4b).
  const uint32_t droppedTotal = ring.stats().overwritten + oldBootDrops;
  TEST_ASSERT_EQUAL_UINT32(5, droppedTotal);
}

// 25 -----------------------------------------------------------------------
void test_l25_ack_not_resurfaced_after_reboot() {
  FakeSlots io(8);
  uint32_t persistedAck = 0;
  {
    dg::Ring ring(io);
    ring.mount(persistedAck);
    for (uint32_t i = 1; i <= 5; ++i) {
      ring.append(simpleRec(i * 100));
    }
    // All five uploaded and confirmed.
    ring.dropOldest(5);
    TEST_ASSERT_TRUE(ring.empty());
    persistedAck = ring.lastDroppedSeq();
    TEST_ASSERT_EQUAL_UINT32(5, persistedAck);
  }

  // Reboot. The slots are still on flash, but they are already delivered.
  dg::Ring reborn(io);
  reborn.mount(persistedAck);

  TEST_ASSERT_EQUAL_UINT16(0, reborn.count());
  TEST_ASSERT_EQUAL_UINT32(5, reborn.stats().ackSkipped);
  TEST_ASSERT_EQUAL_UINT16(0, reborn.stats().badSlots);
  // Sequence numbering continues, so a new record cannot collide with an acked one.
  TEST_ASSERT_EQUAL_UINT32(6, reborn.nextSeq());

  TEST_ASSERT_TRUE(reborn.append(simpleRec(600)));
  dg::StoredRecord r{};
  TEST_ASSERT_TRUE(reborn.readAt(0, r));
  TEST_ASSERT_EQUAL_UINT32(6, r.seq);
  TEST_ASSERT_EQUAL_UINT16(1, reborn.count());
}

// 26 -----------------------------------------------------------------------
void test_l26_partial_ack_leaves_only_unacked() {
  FakeSlots io(8);
  uint32_t persistedAck = 0;
  {
    dg::Ring ring(io);
    ring.mount(persistedAck);
    for (uint32_t i = 1; i <= 6; ++i) {
      ring.append(simpleRec(i * 100));
    }
    // One bulk call carried the first two; the power died before the rest.
    ring.dropOldest(2);
    persistedAck = ring.lastDroppedSeq();
    TEST_ASSERT_EQUAL_UINT32(2, persistedAck);
  }

  dg::Ring reborn(io);
  reborn.mount(persistedAck);

  TEST_ASSERT_EQUAL_UINT16(4, reborn.count());
  TEST_ASSERT_EQUAL_UINT32(2, reborn.stats().ackSkipped);
  TEST_ASSERT_EQUAL_UINT32(7, reborn.nextSeq());

  // Exactly 3..6 survive, still oldest-first.
  dg::StoredRecord r{};
  for (uint16_t n = 0; n < 4; ++n) {
    TEST_ASSERT_TRUE(reborn.readAt(n, r));
    TEST_ASSERT_EQUAL_UINT32(static_cast<uint32_t>(n + 3), r.seq);
    TEST_ASSERT_EQUAL_UINT32((n + 3) * 100u, r.uptimeMs);
  }
  TEST_ASSERT_FALSE(reborn.readAt(4, r));
}

} // namespace dgtest
