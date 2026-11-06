#include "dg_bulk_json.h"

#include "dg_timestamp.h"

namespace dg {
namespace {

// Append-only writer over a caller-owned buffer. Every put() checks the room
// first, and ok_ latches false on the first overrun so a caller can roll back.
class Writer {
public:
  Writer(char* buf, uint16_t cap, uint16_t len)
      : buf_(buf), cap_(cap), len_(len), ok_(true) {}

  void putChar(char c) {
    if (!ok_ || len_ + 1 >= cap_) {
      ok_ = false;
      return;
    }
    buf_[len_++] = c;
  }

  void putStr(const char* s) {
    while (*s != '\0') {
      putChar(*s++);
    }
  }

  void putUint(uint32_t v) {
    char tmp[11];
    uint8_t n = 0;
    do {
      tmp[n++] = static_cast<char>('0' + (v % 10u));
      v /= 10u;
    } while (v != 0 && n < sizeof(tmp));
    while (n > 0) {
      putChar(tmp[--n]);
    }
  }

  // Two decimal places, matching the CSV the serial log prints.
  void putFixed2(float v) {
    bool negative = v < 0.0f;
    if (negative) {
      v = -v;
    }
    const uint32_t scaled = static_cast<uint32_t>(v * 100.0f + 0.5f);
    if (negative && scaled != 0) {
      putChar('-');
    }
    putUint(scaled / 100u);
    putChar('.');
    const uint32_t frac = scaled % 100u;
    putChar(static_cast<char>('0' + (frac / 10u)));
    putChar(static_cast<char>('0' + (frac % 10u)));
  }

  // Percent-encodes anything outside the unreserved set, which for an ISO 8601
  // timestamp means the colons become %3A.
  void putFormEncoded(const char* s) {
    static const char kHex[] = "0123456789ABCDEF";
    for (; *s != '\0'; ++s) {
      const char c = *s;
      const bool unreserved = (c >= 'A' && c <= 'Z') ||
                              (c >= 'a' && c <= 'z') ||
                              (c >= '0' && c <= '9') || c == '-' ||
                              c == '_' || c == '.' || c == '~';
      if (unreserved) {
        putChar(c);
      } else {
        putChar('%');
        putChar(kHex[(static_cast<uint8_t>(c) >> 4) & 0x0F]);
        putChar(kHex[static_cast<uint8_t>(c) & 0x0F]);
      }
    }
  }

  bool ok() const { return ok_; }
  uint16_t length() const { return len_; }

private:
  char* buf_;
  uint16_t cap_;
  uint16_t len_;
  bool ok_;
};

// field4..field7 and field8 are small integers; field1..field3 are levels.
void putFields(Writer& w, const StoredRecord& r, uint32_t droppedTotal,
               bool json) {
  if (json) {
    w.putStr(",\"field1\":");
    w.putFixed2(r.rec.uCm);
    w.putStr(",\"field2\":");
    w.putFixed2(r.rec.dCm);
    w.putStr(",\"field3\":");
    w.putFixed2(r.rec.diffCm);
    w.putStr(",\"field4\":");
    w.putUint(r.rec.state);
    w.putStr(",\"field5\":");
    w.putUint(r.rec.cycles);
    w.putStr(",\"field6\":");
    w.putUint(r.rec.fault);
    w.putStr(",\"field7\":");
    w.putUint(r.rec.event);
    w.putStr(",\"field8\":");
    w.putUint(droppedTotal);
  } else {
    w.putStr("&field1=");
    w.putFixed2(r.rec.uCm);
    w.putStr("&field2=");
    w.putFixed2(r.rec.dCm);
    w.putStr("&field3=");
    w.putFixed2(r.rec.diffCm);
    w.putStr("&field4=");
    w.putUint(r.rec.state);
    w.putStr("&field5=");
    w.putUint(r.rec.cycles);
    w.putStr("&field6=");
    w.putUint(r.rec.fault);
    w.putStr("&field7=");
    w.putUint(r.rec.event);
    w.putStr("&field8=");
    w.putUint(droppedTotal);
  }
}

} // namespace

BulkBuilder::BulkBuilder(char* buf, uint16_t cap, const char* writeApiKey)
    : buf_(buf),
      cap_(cap),
      len_(0),
      entries_(0),
      lastEpoch_(0),
      overflowed_(false),
      finished_(false) {
  Writer w(buf_, cap_, 0);
  w.putStr("{\"write_api_key\":\"");
  w.putStr(writeApiKey);
  w.putStr("\",\"updates\":[");
  if (w.ok()) {
    len_ = w.length();
  } else {
    overflowed_ = true;
  }
  if (cap_ > 0) {
    buf_[len_] = '\0';
  }
}

bool BulkBuilder::addEntry(const StoredRecord& r, uint32_t droppedTotal) {
  if (finished_ || r.epoch == 0) {
    return false;
  }

  // ThingSpeak rejects the whole batch on a duplicate timestamp, so nudge a
  // repeat forward by a second rather than losing everything (D49).
  uint32_t epoch = r.epoch;
  if (entries_ > 0 && epoch <= lastEpoch_) {
    epoch = lastEpoch_ + 1;
  }

  char stamp[kIso8601Bytes];
  if (!formatIso8601Utc(epoch, stamp)) {
    return false;
  }

  const uint16_t rollback = len_;
  Writer w(buf_, cap_, len_);
  if (entries_ > 0) {
    w.putChar(',');
  }
  w.putStr("{\"created_at\":\"");
  w.putStr(stamp);
  w.putChar('"');
  putFields(w, r, droppedTotal, /*json=*/true);
  w.putChar('}');

  if (!w.ok()) {
    len_ = rollback; // body stays valid; caller sends what already fits
    buf_[len_] = '\0';
    overflowed_ = true;
    return false;
  }

  len_ = w.length();
  buf_[len_] = '\0';
  ++entries_;
  lastEpoch_ = epoch;
  return true;
}

bool BulkBuilder::finish() {
  if (finished_) {
    return true;
  }
  Writer w(buf_, cap_, len_);
  w.putStr("]}");
  if (!w.ok()) {
    return false;
  }
  len_ = w.length();
  buf_[len_] = '\0';
  finished_ = true;
  return true;
}

uint16_t buildUpdateBody(char* buf, uint16_t cap, const char* writeApiKey,
                         const StoredRecord& r, uint32_t droppedTotal) {
  if (r.epoch == 0) {
    return 0;
  }
  char stamp[kIso8601Bytes];
  if (!formatIso8601Utc(r.epoch, stamp)) {
    return 0;
  }

  Writer w(buf, cap, 0);
  w.putStr("api_key=");
  w.putFormEncoded(writeApiKey);
  w.putStr("&created_at=");
  w.putFormEncoded(stamp);
  putFields(w, r, droppedTotal, /*json=*/false);

  if (!w.ok()) {
    return 0;
  }
  buf[w.length()] = '\0';
  return w.length();
}

} // namespace dg
