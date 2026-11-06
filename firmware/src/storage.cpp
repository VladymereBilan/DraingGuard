#include "storage.h"

#include <Preferences.h>

namespace hal {
namespace {

constexpr char kNamespace[] = "dg";
constexpr char kKeyBaselineValid[] = "bl_valid";
constexpr char kKeyBaseline[] = "baseline";
constexpr char kKeyCycles[] = "cycles";

} // namespace

Storage::Storage() : writes_(0) {}

Saved Storage::load() {
  Saved s{};
  Preferences prefs;
  if (!prefs.begin(kNamespace, /*readOnly=*/true)) {
    // Namespace does not exist yet: first boot on a blank device.
    return s;
  }
  s.baselineValid = prefs.getBool(kKeyBaselineValid, false);
  s.baselineCm = prefs.getFloat(kKeyBaseline, 0.0f);
  s.cyclesSinceEmptied = prefs.getUShort(kKeyCycles, 0);
  prefs.end();
  return s;
}

void Storage::saveBaseline(float cm) {
  Preferences prefs;
  if (!prefs.begin(kNamespace, /*readOnly=*/false)) {
    return;
  }
  prefs.putFloat(kKeyBaseline, cm);
  prefs.putBool(kKeyBaselineValid, true);
  prefs.end();
  ++writes_;
}

void Storage::saveCycles(uint16_t cycles) {
  Preferences prefs;
  if (!prefs.begin(kNamespace, /*readOnly=*/false)) {
    return;
  }
  prefs.putUShort(kKeyCycles, cycles);
  prefs.end();
  ++writes_;
}

} // namespace hal
