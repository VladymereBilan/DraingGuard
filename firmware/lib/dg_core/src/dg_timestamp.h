// Dating records, and formatting the result for ThingSpeak.
//
// A record is made with only an uptime; the wall-clock time may not be known
// yet (D8). Once NTP has synced, a record from the current boot can be dated
// by subtracting its age from the current epoch. A record from an earlier boot
// that was never dated cannot be reconstructed at all - its uptime refers to a
// clock that no longer exists - so it is dropped and counted.
//
// The calendar maths is done here rather than through <time.h> so the result is
// identical on the host and the device, with no timezone state involved. The
// output is always UTC (D49).

#ifndef DG_TIMESTAMP_H
#define DG_TIMESTAMP_H

#include <stdint.h>

#include "dg_record_codec.h"

namespace dg {

// Bytes needed for "2026-11-06T04:34:56Z" plus the terminator.
inline constexpr uint8_t kIso8601Bytes = 21;

enum class EpochFate : uint8_t {
  ALREADY_SET,   // the record was dated when it was made
  BACKFILLED,    // dated just now from the uptime delta
  DROP_OLD_BOOT, // undatable: previous boot, never synced
  WAIT,          // current boot, time not synced yet - ask again later
};

// Fills r.epoch when it can. Only BACKFILLED modifies r.
EpochFate resolveEpoch(StoredRecord& r, uint16_t currentBootId,
                       uint32_t nowUptimeMs, uint32_t nowEpoch,
                       bool timeSynced);

// "2026-11-06T04:34:56Z". False for epoch 0, which means "unknown".
bool formatIso8601Utc(uint32_t epoch, char out[kIso8601Bytes]);

} // namespace dg

#endif // DG_TIMESTAMP_H
