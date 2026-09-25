#pragma once

#include <optional>

namespace dairlib {
namespace systems {

/// Decides when a latched pose-tracking cost should be released:  once the
/// object's distance to the goal has stayed above release_distance for
/// hold_seconds.  The timer restarts whenever the distance drops back to
/// release_distance or below, so brief excursions (pose noise, a transient
/// bump) never release the latch.
class PoseLatchReleaseTimer {
 public:
  void Reset() { beyond_since_.reset(); }

  /// Returns true if the latch should be released at time `now`.
  bool Update(double distance, double release_distance, double hold_seconds,
              double now) {
    if (distance <= release_distance) {
      beyond_since_.reset();
      return false;
    }
    if (!beyond_since_.has_value()) beyond_since_ = now;
    return now - *beyond_since_ >= hold_seconds;
  }

 private:
  std::optional<double> beyond_since_;
};

}  // namespace systems
}  // namespace dairlib
