#include "dg_filters.h"

namespace dg {
namespace {

// NaN is the only value that compares unequal to itself.
inline bool isNotANumber(float x) { return !(x == x); }

} // namespace

float echoUsToCm(uint32_t us, float soundCmPerUs) {
  if (us == 0u) {
    return -1.0f; // timeout: no echo came back
  }
  // The pulse covers the round trip, so halve it for the one-way distance.
  return static_cast<float>(us) * soundCmPerUs * 0.5f;
}

float levelFromDistance(float mountCm, float distCm) {
  return mountCm - distCm;
}

float medianOf(const float* v, int n) {
  if (v == nullptr || n < 1 || n > kMaxMedianN) {
    return -1.0f;
  }

  // Copy first; the caller's samples are never reordered.
  float a[kMaxMedianN];
  for (int i = 0; i < n; ++i) {
    a[i] = v[i];
  }

  // Insertion sort: at most five elements, no heap, no library calls.
  for (int i = 1; i < n; ++i) {
    const float key = a[i];
    int j = i - 1;
    while (j >= 0 && a[j] > key) {
      a[j + 1] = a[j];
      --j;
    }
    a[j + 1] = key;
  }

  if (n % 2 == 1) {
    return a[n / 2];
  }
  return (a[n / 2 - 1] + a[n / 2]) * 0.5f;
}

DistanceFilter::DistanceFilter(const FilterParams& params)
    : params_(params),
      rawValue_{},
      rawOk_{},
      rawCount_(0),
      rawHead_(0),
      medians_{},
      medianCount_(0),
      medianHead_(0),
      pushesWithoutMedian_(0),
      hasMedian_(false) {}

bool DistanceFilter::sampleValid(float cm) const {
  if (isNotANumber(cm)) {
    return false;
  }
  if (cm < 0.0f) {
    return false;
  }
  return cm >= params_.minCm && cm <= params_.maxCm;
}

void DistanceFilter::push(float rawCm) {
  rawValue_[rawHead_] = rawCm;
  rawOk_[rawHead_] = sampleValid(rawCm);
  rawHead_ = (rawHead_ + 1) % kRawWindow;
  if (rawCount_ < kRawWindow) {
    ++rawCount_;
  }

  // Gather the valid samples currently in the window. Order does not matter to
  // a median, so the ring can be read in slot order.
  float good[kRawWindow];
  int n = 0;
  for (int i = 0; i < rawCount_; ++i) {
    if (rawOk_[i]) {
      good[n++] = rawValue_[i];
    }
  }

  if (n < kMinValidInWindow) {
    // Saturate at staleSamples so a long dropout cannot overflow the counter.
    if (pushesWithoutMedian_ < params_.staleSamples) {
      ++pushesWithoutMedian_;
    }
    return;
  }

  medians_[medianHead_] = medianOf(good, n);
  medianHead_ = (medianHead_ + 1) % kAvgWindow;
  if (medianCount_ < kAvgWindow) {
    ++medianCount_;
  }
  pushesWithoutMedian_ = 0;
  hasMedian_ = true;
}

bool DistanceFilter::valid() const {
  return hasMedian_ && pushesWithoutMedian_ < params_.staleSamples;
}

bool DistanceFilter::ready() const {
  return medianCount_ >= kAvgWindow;
}

float DistanceFilter::value() const {
  if (medianCount_ == 0) {
    return 0.0f;
  }
  float sum = 0.0f;
  for (int i = 0; i < medianCount_; ++i) {
    sum += medians_[i];
  }
  return sum / static_cast<float>(medianCount_);
}

void DistanceFilter::reset() {
  for (int i = 0; i < kRawWindow; ++i) {
    rawValue_[i] = 0.0f;
    rawOk_[i] = false;
  }
  for (int i = 0; i < kAvgWindow; ++i) {
    medians_[i] = 0.0f;
  }
  rawCount_ = 0;
  rawHead_ = 0;
  medianCount_ = 0;
  medianHead_ = 0;
  pushesWithoutMedian_ = 0;
  hasMedian_ = false;
}

} // namespace dg
