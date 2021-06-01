#pragma once

#include <drake/multibody/plant/multibody_plant.h>

#include "systems/framework/timestamped_vector.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib::systems {

/// TODO (yangwill) This leaf system just passes through the trajectory.
/// It might be worth making this a general class
class TrajectoryPlayback : public drake::systems::LeafSystem<double> {
 public:
  TrajectoryPlayback(
      const drake::trajectories::PiecewisePolynomial<double>& traj,
      int num_inputs, double time_offset = 0.0);

 private:
  void CalcEffort(const drake::systems::Context<double>& context,
                  BasicVector<double>* control) const;

  drake::trajectories::PiecewisePolynomial<double> traj_;
};

}  // namespace dairlib::systems
