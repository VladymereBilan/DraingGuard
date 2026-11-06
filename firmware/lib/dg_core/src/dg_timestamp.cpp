#include "dg_timestamp.h"

namespace dg {
namespace {

// Howard Hinnant's civil_from_days: days since 1970-01-01 to y/m/d, valid for
// any proleptic Gregorian date and free of <time.h> and timezone state.
void civilFromDays(int32_t z, int32_t& y, uint8_t& m, uint8_t& d) {
  z += 719468;
  const int32_t era = (z >= 0 ? z : z - 146096) / 146097;
  const uint32_t doe = static_cast<uint32_t>(z - era * 146097);
  const uint32_t yoe =
      (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  const int32_t yy = static_cast<int32_t>(yoe) + era * 400;
  const uint32_t doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  const uint32_t mp = (5 * doy + 2) / 153;
  const uint32_t dd = doy - (153 * mp + 2) / 5 + 1;
  const uint32_t mm = mp < 10 ? mp + 3 : mp - 9;

  y = yy + static_cast<int32_t>(mm <= 2);
  m = static_cast<uint8_t>(mm);
  d = static_cast<uint8_t>(dd);
}

void write2(char* p, uint32_t v) {
  p[0] = static_cast<char>('0' + ((v / 10) % 10));
  p[1] = static_cast<char>('0' + (v % 10));
}

void write4(char* p, uint32_t v) {
  p[0] = static_cast<char>('0' + ((v / 1000) % 10));
  p[1] = static_cast<char>('0' + ((v / 100) % 10));
  p[2] = static_cast<char>('0' + ((v / 10) % 10));
  p[3] = static_cast<char>('0' + (v % 10));
}

} // namespace

EpochFate resolveEpoch(StoredRecord& r, uint16_t currentBootId,
                       uint32_t nowUptimeMs, uint32_t nowEpoch,
                       bool timeSynced) {
  if (r.epoch != 0) {
    return EpochFate::ALREADY_SET;
  }
  if (r.bootId != currentBootId) {
    // Its uptime refers to a previous boot's clock: undatable (D8).
    return EpochFate::DROP_OLD_BOOT;
  }
  if (!timeSynced || nowEpoch == 0) {
    return EpochFate::WAIT;
  }

  // Unsigned subtraction, so a millis() rollover between the record and now
  // still yields the correct elapsed time.
  const uint32_t ageMs = nowUptimeMs - r.uptimeMs;
  r.epoch = nowEpoch - (ageMs / 1000u);
  return EpochFate::BACKFILLED;
}

bool formatIso8601Utc(uint32_t epoch, char out[kIso8601Bytes]) {
  if (epoch == 0) {
    return false;
  }

  const uint32_t days = epoch / 86400u;
  uint32_t secOfDay = epoch % 86400u;

  int32_t year = 0;
  uint8_t month = 0;
  uint8_t day = 0;
  civilFromDays(static_cast<int32_t>(days), year, month, day);

  const uint32_t hour = secOfDay / 3600u;
  secOfDay %= 3600u;
  const uint32_t minute = secOfDay / 60u;
  const uint32_t second = secOfDay % 60u;

  write4(out + 0, static_cast<uint32_t>(year));
  out[4] = '-';
  write2(out + 5, month);
  out[7] = '-';
  write2(out + 8, day);
  out[10] = 'T';
  write2(out + 11, hour);
  out[13] = ':';
  write2(out + 14, minute);
  out[16] = ':';
  write2(out + 17, second);
  out[19] = 'Z';
  out[20] = '\0';
  return true;
}

} // namespace dg
