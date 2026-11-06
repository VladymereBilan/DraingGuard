#include "dg_filters.h"

namespace dg {

float median5(const float v[5]) {
  // Copy first; the caller's samples are never reordered.
  float a[5] = {v[0], v[1], v[2], v[3], v[4]};

  // Insertion sort: five elements, no heap, no library calls.
  for (int i = 1; i < 5; ++i) {
    const float key = a[i];
    int j = i - 1;
    while (j >= 0 && a[j] > key) {
      a[j + 1] = a[j];
      --j;
    }
    a[j + 1] = key;
  }

  return a[2];
}

} // namespace dg
