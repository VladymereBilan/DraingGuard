// Sensor filter pipeline for DrainGuard.
//
// Nothing in dg_core may include Arduino.h, touch hardware, or allocate on the
// heap, and it is configured through Params structs rather than globals (D2).
// That is what lets the native environment compile and unit-test it on the host,
// and it compiles identically on the ESP32 under gnu++17 (D20).

#ifndef DG_FILTERS_H
#define DG_FILTERS_H

#include <stdint.h>

namespace dg {

// Largest sample count medianOf accepts.
inline constexpr int kMaxMedianN = 5;

// Convert an HC-SR04 echo pulse width into a one-way distance in cm.
// us is the round-trip time; 0 means the echo timed out, and -1 is returned.
float echoUsToCm(uint32_t us, float soundCmPerUs);

// Water level above the channel bed: the measured surface distance subtracted
// from the sensor's mount height.
float levelFromDistance(float mountCm, float distCm);

// Median of n samples, n in 1..kMaxMedianN. An even n returns the mean of the
// two middle values. Sorts an internal copy, so v is left unchanged.
// Returns -1 for a null pointer or an n outside 1..kMaxMedianN.
float medianOf(const float* v, int n);

struct FilterParams {
  float minCm;      // samples below this are rejected
  float maxCm;      // samples above this are rejected
  int staleSamples; // pushes producing no median before the sensor goes invalid
};

// Per-sensor filter: range rejection -> median of the last 5 raw pushes ->
// moving average of the last 5 medians.
//
// Each push keeps its slot in the raw window whether or not the sample was
// valid, so a dropout ages out naturally instead of being papered over.
class DistanceFilter {
public:
  static constexpr int kRawWindow = 5;        // raw pushes kept for the median
  static constexpr int kAvgWindow = 5;        // medians kept for the moving average
  static constexpr int kMinValidInWindow = 3; // valid samples needed to form a median

  explicit DistanceFilter(const FilterParams& params);

  // Feed one raw distance reading in cm. Negative, NaN, or outside
  // [minCm, maxCm] counts as an invalid sample.
  void push(float rawCm);

  // A median has been produced at least once, and fewer than staleSamples
  // no-median pushes have happened since the most recent one. Note that the two
  // pushes right after good data still carry enough valid samples in the window
  // to produce a median, so a total dropout takes
  // (kRawWindow - kMinValidInWindow) + staleSamples pushes to report invalid.
  bool valid() const;

  // The moving average holds a full set of kAvgWindow medians.
  bool ready() const;

  // Current moving average of the medians, in cm. Averages however many medians
  // exist so far, and returns 0 before the first one - check valid() first.
  float value() const;

  void reset();

private:
  bool sampleValid(float cm) const;

  FilterParams params_;

  float rawValue_[kRawWindow];
  bool rawOk_[kRawWindow];
  int rawCount_;
  int rawHead_;

  float medians_[kAvgWindow];
  int medianCount_;
  int medianHead_;

  int pushesWithoutMedian_;
  bool hasMedian_;
};

} // namespace dg

#endif // DG_FILTERS_H
