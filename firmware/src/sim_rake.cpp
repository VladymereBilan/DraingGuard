#include "sim_rake.h"

#ifdef DG_WOKWI

namespace sim {
namespace {

constexpr float kAtHome = 0.0001f;
constexpr float kAtEnd = 0.9999f;

} // namespace

Rake::Rake() : posFrac_(0.5f) {} // starts at mid-travel (D44)

void Rake::update(dg::MotorCmd cmd, uint32_t dtMs) {
  const float step =
      static_cast<float>(dtMs) / static_cast<float>(kFullStrokeMs);

  if (cmd == dg::MotorCmd::LOWER) {
    posFrac_ += step;
  } else if (cmd == dg::MotorCmd::RAISE) {
    posFrac_ -= step;
  }

  if (posFrac_ < 0.0f) {
    posFrac_ = 0.0f;
  } else if (posFrac_ > 1.0f) {
    posFrac_ = 1.0f;
  }
}

bool Rake::homePressed() const { return posFrac_ <= kAtHome; }
bool Rake::endPressed() const { return posFrac_ >= kAtEnd; }

} // namespace sim

#endif // DG_WOKWI
