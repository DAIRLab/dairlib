#pragma once

#include "drake/common/trajectories/exponential_plus_piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/leaf_system.h"

#include "dairlib/lcmt_trajectory_block.hpp"
#include "lcm/lcm_trajectory.h"
#include "multibody/multibody_utils.h"
#include "systems/framework/output_vector.h"

namespace dairlib {
namespace goldilocks_models {

class PlannedTrajGuard : public drake::systems::LeafSystem<double> {
 public:
  PlannedTrajGuard(double max_solve_time);

  const drake::systems::InputPort<double>& get_input_port_optimal_rom_traj()
      const {
    return this->get_input_port(optimal_traj_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_lipm_traj() const {
    return this->get_input_port(lipm_port_);
  }

 private:
  void ApplyGuard(const drake::systems::Context<double>& context,
                  drake::trajectories::Trajectory<double>* traj) const;

  int optimal_traj_port_;
  int lipm_port_;

  double max_solve_time_;
};

}  // namespace goldilocks_models
}  // namespace dairlib
