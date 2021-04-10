#pragma once

#include "drake/common/trajectories/exponential_plus_piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/leaf_system.h"

#include "dairlib/lcmt_trajectory_block.hpp"
#include "dairlib/lcmt_saved_traj.hpp"
#include "multibody/multibody_utils.h"
#include "systems/framework/output_vector.h"
#include "lcm/lcm_trajectory.h"

namespace dairlib {
namespace goldilocks_models {

class OptimalRoMTrajReceiver : public drake::systems::LeafSystem<double> {
 public:
  OptimalRoMTrajReceiver();

 private:
  void CalcDesiredTraj(const drake::systems::Context<double>& context,
                       drake::trajectories::Trajectory<double>* traj) const;

  int rom_traj_lcm_port_;
};

}  // namespace goldilocks_models
}  // namespace dairlib
