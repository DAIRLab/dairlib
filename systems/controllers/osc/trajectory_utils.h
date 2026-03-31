#pragma once

#include <drake/common/trajectories/piecewise_polynomial.h>
#include <drake/common/trajectories/trajectory.h>

namespace dairlib::systems::controllers {

inline bool HasUsableTrajectory(
    const drake::trajectories::Trajectory<double>& traj) {
  const auto* pp =
      dynamic_cast<const drake::trajectories::PiecewisePolynomial<double>*>(
          &traj);
  return pp != nullptr && pp->get_number_of_segments() > 0;
}

}  // namespace dairlib::systems::controllers
