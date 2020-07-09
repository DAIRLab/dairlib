#pragma once

#include <drake/multibody/plant/multibody_plant.h>
#include "examples/Cassie/osc_jump/jumping_event_based_fsm.h"
#include "systems/controllers/control_utils.h"
#include "systems/framework/output_vector.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib::examples::Cassie::osc_jump {

/// TODO (yangwill) This leaf system just passes through the trajectory.
/// It might be worth making this a general class
class PelvisOrientationTrajGenerator : public drake::systems::LeafSystem<double> {
 public:
  PelvisOrientationTrajGenerator(
      const drake::multibody::MultibodyPlant<double>& plant,
      const drake::trajectories::PiecewisePolynomial<double>& orientation_traj,
      std::string traj_name, double time_offset = 0.0);

 private:
  void CalcTraj(const drake::systems::Context<double>& context,
                drake::trajectories::Trajectory<double>* traj) const;

  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::trajectories::PiecewisePolynomial<double> traj_;

  int state_port_;
};

}  // namespace dairlib::examples::Cassie::osc_jump
