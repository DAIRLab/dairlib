#pragma once

#include "drake/common/trajectories/exponential_plus_piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/leaf_system.h"

#include "multibody/multibody_utils.h"
#include "systems/framework/output_vector.h"

namespace dairlib {
namespace goldilocks_models {

/// Assumption:
/// - we assume that there is no x, y and yaw dependency in the ROM (the mapping
/// function), because we rotate the robot state at touchdown to the origin in
/// MPC

class OptimalRomPlanner : public drake::systems::LeafSystem<double> {
 public:
  OptimalRomPlanner(const drake::multibody::MultibodyPlant<double>& plant,
                    const std::vector<int>& unordered_fsm_states);

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_fsm() const {
    return this->get_input_port(fsm_port_);
  }

 private:
  void SolveMPC(const drake::systems::Context<double>& context,
                drake::trajectories::Trajectory<double>* traj) const;

  // Port indices
  int state_port_;
  int touchdown_state_port_;
  int fsm_port_;

//  const drake::multibody::MultibodyPlant<double>& plant_;
  std::vector<int> unordered_fsm_states_;
};

}  // namespace goldilocks_models
}  // namespace dairlib
