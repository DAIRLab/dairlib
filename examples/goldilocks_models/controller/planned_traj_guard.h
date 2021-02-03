#pragma once

#include "drake/common/trajectories/exponential_plus_piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/leaf_system.h"

#include "lcm/lcm_trajectory.h"
#include "multibody/multibody_utils.h"
#include "systems/framework/output_vector.h"

namespace dairlib {
namespace goldilocks_models {

class PlannedTrajGuard : public drake::systems::LeafSystem<double> {
 public:
  PlannedTrajGuard();

  const drake::systems::InputPort<double>& get_input_port_optimal_rom_traj()
      const {
    return this->get_input_port(optimal_rom_traj_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_lipm_traj() const {
    return this->get_input_port(lipm_port_);
  }

 private:
  void ApplyGuard(const drake::systems::Context<double>& context,
                  drake::trajectories::Trajectory<double>* traj) const;

  int optimal_rom_traj_port_;
  int lipm_port_;

  // Debugging
  mutable int previous_traj_ = 0;
  const int OPTIMAL_ROM_TRAJ_INDEX = 0;
  const int LIPM_TRAJ_INDEX = 1;
};

}  // namespace goldilocks_models
}  // namespace dairlib
