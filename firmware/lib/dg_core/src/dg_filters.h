// Pure-logic filters for DrainGuard.
//
// Nothing in dg_core may include Arduino.h, touch hardware, or allocate on the
// heap. That is what lets the native environment compile and unit-test it on the
// host machine.

#ifndef DG_FILTERS_H
#define DG_FILTERS_H

namespace dg {

// Median of exactly five samples, used to throw out ultrasonic spikes.
// Sorts a local copy, so the caller's array is left untouched.
float median5(const float v[5]);

} // namespace dg

#endif // DG_FILTERS_H
