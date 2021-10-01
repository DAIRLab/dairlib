#pragma once

#include <drake/multibody/plant/multibody_plant.h>

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib::examples::osc_jump {

/// TODO (yangwill) This leaf system just passes through the trajectory.
/// It might be worth making this a general class
class BasicTrajectoryPassthrough : public drake::systems::LeafSystem<double> {
 public:
  BasicTrajectoryPassthrough(
      const drake::trajectories::PiecewisePolynomial<double>& traj,
      const std::string& traj_name, double time_offset = 0.0);

 private:
  void CalcTraj(const drake::systems::Context<double>& context,
                drake::trajectories::Trajectory<double>* traj) const;

  drake::trajectories::PiecewisePolynomial<double> traj_;
};

}  // namespace dairlib::examples::osc_jump
