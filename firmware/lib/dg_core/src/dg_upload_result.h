// Deciding whether an upload actually succeeded.
//
// This looked trivial and was not. ThingSpeak's bulk endpoint answers
// 202 Accepted, not 200: the batch is queued and scanned asynchronously, so
// the server acknowledges receipt rather than completion. MathWorks' reference
// page documents 200 OK, so trusting the documentation made every bulk flush
// look like a failure - the records stayed in the ring, new ones piled up
// behind them, and the batch would have been re-sent and rejected wholesale
// for duplicate timestamps.
//
// The /update endpoint is different again: it answers 200 with the new entry
// id as plain text, or 200 with "0" when it rejected the write. A 200 there is
// not success on its own.
//
// Both rules live here as pure functions so the exact bytes that count as
// success are pinned by tests rather than discovered in the field (D67).

#ifndef DG_UPLOAD_RESULT_H
#define DG_UPLOAD_RESULT_H

#include <stdint.h>

namespace dg {

enum class UploadResult : uint8_t {
  OK,     // accepted; the records may be dropped from the ring
  FAILED, // keep the records and try again in the next window
};

// Success = HTTP 200 or 202 with "success":true somewhere in the body.
// Whitespace around the colon and the value is tolerated.
UploadResult classifyBulk(int httpCode, const char* body);

// Success = HTTP 200 with a body that is a positive integer entry id.
// "0", empty, and anything non-numeric are failures.
UploadResult classifyUpdate(int httpCode, const char* body);

} // namespace dg

#endif // DG_UPLOAD_RESULT_H
