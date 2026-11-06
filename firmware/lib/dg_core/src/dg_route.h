// Where a freshly made record goes: the flash ring, or the RAM pending queue.
//
// This was originally a handful of conditions inlined in the logger task, and
// it got one of them wrong: while the link was down, snapshots were still being
// routed to the pending queue, where each new one coalesced over the last. Three
// records made during a two-minute outage arrived as one upload, with nothing
// written to flash and nothing counted as dropped.
//
// So the decision lives here instead, as a pure function with tests. The rules,
// in priority order:
//
//   0. Flash inhibited       -> PENDING, or DROP if RAM is full. Writing to
//                               flash stalls the instruction cache, which
//                               freezes the 10 ms safety loop on the other
//                               core; a sector erase can take ~50 ms and the
//                               rake would sail past a limit switch. So while
//                               the motor is running, nothing touches flash and
//                               records wait in RAM (D66).
//   1. Not ONLINE            -> RING. An offline record must reach flash; RAM is
//                               not durable and the pending queue coalesces.
//   2. Ring already has data -> RING. The ring is the single ordering authority,
//                               so nothing may overtake what is queued (D51).
//   3. Pending queue full    -> RING. Spill rather than drop.
//   4. Waiting too long for
//      NTP                   -> RING. Records can wait in RAM for the clock, but
//                               only up to preNtpHoldMs (D52).
//   5. Otherwise             -> PENDING.
//
// Coalescing is a separate, narrower question - see mayCoalesce - because it
// discards a record, and that is only acceptable while the link is up and both
// snapshots would have gone out in the same upload window anyway (D65).

#ifndef DG_ROUTE_H
#define DG_ROUTE_H

#include <stdint.h>

namespace dg {

enum class LinkState : uint8_t {
  OFFLINE, // no link, or connected but not yet usable for uploads
  ONLINE,  // associated, clock set, uploads are going out
};

enum class Route : uint8_t {
  RING,    // write to flash
  PENDING, // hold in RAM for the next upload window
  DROP,    // nowhere to put it; the caller counts it in field 8
};

struct RouteParams {
  uint8_t pendingCapacity;  // dg::PendingQueue::kCapacity
  uint32_t preNtpHoldMs;    // how long a record may wait in RAM for the clock
  uint32_t uploadSpacingMs; // cfg::UPLOAD_SPACING_MS, the coalescing window
};

Route routeRecord(const RouteParams& params, LinkState link, uint16_t ringCount,
                  uint8_t pendingCount, bool timeSynced,
                  uint32_t oldestPendingAgeMs, bool flashInhibited);

// True only when a snapshot may replace the newest pending snapshot. Requires
// the link to be up, the current upload window still open, and flash not
// inhibited - while it is, the pending records are destined for the ring and
// must not be coalesced away (D65 with D66).
bool mayCoalesce(const RouteParams& params, LinkState link,
                 bool flashInhibited, bool recordIsSnapshot,
                 bool newestPendingIsSnapshot, uint32_t msSinceLastUpload);

} // namespace dg

#endif // DG_ROUTE_H
