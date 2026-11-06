#include "dg_upload_result.h"

namespace dg {
namespace {

bool isSpace(char c) {
  return c == ' ' || c == '\t' || c == '\r' || c == '\n';
}

bool startsWith(const char* s, const char* prefix) {
  while (*prefix != '\0') {
    if (*s == '\0' || *s != *prefix) {
      return false;
    }
    ++s;
    ++prefix;
  }
  return true;
}

// Looks for the "success" key followed by a true value, allowing whitespace
// around the colon: {"success":true}, { "success" : true } and so on.
// "success":false must not match.
bool hasSuccessTrue(const char* body) {
  if (body == nullptr) {
    return false;
  }
  for (const char* p = body; *p != '\0'; ++p) {
    if (!startsWith(p, "\"success\"")) {
      continue;
    }
    const char* q = p + 9; // past the quoted key
    while (isSpace(*q)) {
      ++q;
    }
    if (*q != ':') {
      continue;
    }
    ++q;
    while (isSpace(*q)) {
      ++q;
    }
    if (startsWith(q, "true")) {
      return true;
    }
    // Found the key with a different value - keep scanning in case the body
    // repeats it, but this occurrence is not a success.
  }
  return false;
}

} // namespace

UploadResult classifyBulk(int httpCode, const char* body) {
  // 202 is what the live endpoint returns; 200 is what the docs describe.
  // Accept both and nothing else - a 429 or any 4xx/5xx keeps the records.
  if (httpCode != 200 && httpCode != 202) {
    return UploadResult::FAILED;
  }
  return hasSuccessTrue(body) ? UploadResult::OK : UploadResult::FAILED;
}

UploadResult classifyUpdate(int httpCode, const char* body) {
  if (httpCode != 200 || body == nullptr) {
    return UploadResult::FAILED;
  }

  const char* p = body;
  while (isSpace(*p)) {
    ++p;
  }

  bool sawDigit = false;
  bool nonZero = false;
  for (; *p != '\0' && !isSpace(*p); ++p) {
    if (*p < '0' || *p > '9') {
      return UploadResult::FAILED; // not an entry id
    }
    sawDigit = true;
    if (*p != '0') {
      nonZero = true;
    }
  }
  while (isSpace(*p)) {
    ++p;
  }
  if (*p != '\0') {
    return UploadResult::FAILED; // trailing junk
  }

  // "0" means ThingSpeak rejected the write even though the HTTP call was 200.
  return (sawDigit && nonZero) ? UploadResult::OK : UploadResult::FAILED;
}

} // namespace dg
