// Building the ThingSpeak request bodies.
//
// Both builders write into a buffer the caller owns, so there is no heap and no
// String anywhere in the upload path. Keeping this in dg_core means the exact
// bytes that go on the wire are unit-tested on the host.
//
// ThingSpeak rejects an entire bulk batch if any two created_at values collide,
// so addEntry bumps a repeated timestamp by one second (D49). A batch is capped
// by the caller at 60 entries against an 8 KB buffer (D50).
//
// The write API key is a secret: it goes into the body and must never be
// printed anywhere (D58).

#ifndef DG_BULK_JSON_H
#define DG_BULK_JSON_H

#include <stdint.h>

#include "dg_record_codec.h"

namespace dg {

class BulkBuilder {
public:
  // buf must stay alive for the life of the builder. Writes the JSON prologue
  // immediately, so length() is meaningful from the start.
  BulkBuilder(char* buf, uint16_t cap, const char* writeApiKey);

  // Appends one update entry. Returns false and leaves the body untouched when
  // the entry will not fit or the record has no epoch - the caller sends what
  // it has and keeps the rest for the next window.
  bool addEntry(const StoredRecord& r, uint32_t droppedTotal);

  // Closes the JSON. Safe to call with zero entries.
  bool finish();

  const char* body() const { return buf_; }
  uint16_t length() const { return len_; }
  uint16_t entries() const { return entries_; }

  // The last created_at actually emitted, after any collision bump.
  uint32_t lastEpochUsed() const { return lastEpoch_; }

  // True when at least one entry was refused for lack of room.
  bool overflowed() const { return overflowed_; }

private:
  char* buf_;
  uint16_t cap_;
  uint16_t len_;
  uint16_t entries_;
  uint32_t lastEpoch_;
  bool overflowed_;
  bool finished_;
};

// One live record as an application/x-www-form-urlencoded /update body.
// Returns the length written, or 0 if it did not fit or the record has no
// epoch. Always NUL-terminates on success.
uint16_t buildUpdateBody(char* buf, uint16_t cap, const char* writeApiKey,
                         const StoredRecord& r, uint32_t droppedTotal);

} // namespace dg

#endif // DG_BULK_JSON_H
