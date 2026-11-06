#include "dg_route.h"

namespace dg {

Route routeRecord(const RouteParams& params, LinkState link, uint16_t ringCount,
                  uint8_t pendingCount, bool timeSynced,
                  uint32_t oldestPendingAgeMs) {
  // 1. Offline: flash or nothing. This is the rule whose absence lost records.
  if (link != LinkState::ONLINE) {
    return Route::RING;
  }

  // 2. A backlog outranks everything, so the channel stays chronological (D51).
  if (ringCount > 0) {
    return Route::RING;
  }

  // 3. No room left in RAM.
  if (pendingCount >= params.pendingCapacity) {
    return Route::RING;
  }

  // 4. Still undated and the queue has been waiting its allowance (D52).
  if (!timeSynced && pendingCount > 0 &&
      oldestPendingAgeMs >= params.preNtpHoldMs) {
    return Route::RING;
  }

  return Route::PENDING;
}

bool mayCoalesce(const RouteParams& params, LinkState link,
                 bool recordIsSnapshot, bool newestPendingIsSnapshot,
                 uint32_t msSinceLastUpload) {
  if (link != LinkState::ONLINE) {
    return false; // never discard a record we cannot send
  }
  if (!recordIsSnapshot || !newestPendingIsSnapshot) {
    return false; // only a snapshot may absorb a snapshot
  }
  // Once the window has elapsed the pending record is about to be uploaded;
  // replacing it then would throw away something that was going out.
  return msSinceLastUpload < params.uploadSpacingMs;
}

} // namespace dg
