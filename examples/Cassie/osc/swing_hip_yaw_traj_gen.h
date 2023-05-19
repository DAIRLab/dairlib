#pragma once

#include <unordered_map>

#include "systems/controllers/control_utils.h"
#include "systems/framework/output_vector.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace cassie {
namespace osc {

/// Input:
///  - Desired yaw velocity
///
/// Output:
///  - A 1D polynomial trajectory.
class SwingHipYawTrajGenerator : public drake::systems::LeafSystem<double> {
 public:
  SwingHipYawTrajGenerator(
      const drake::multibody::MultibodyPlant<double>& plant,
      const std::unordered_map<int, int>& fsm_to_joint_idx_map,
      double phase_duration);

  // Input/output ports
  const drake::systems::InputPort<double>& get_state_input_port() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_yaw_input_port() const {
    return this->get_input_port(des_yaw_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_fsm() const {
    return this->get_input_port(fsm_port_);
  }

 private:
  void CalcDesiredTraj(const drake::systems::Context<double>& context,
                       drake::trajectories::Trajectory<double>* traj) const;

  const drake::multibody::MultibodyPlant<double>& plant_;
  const std::unordered_map<int, int>& fsm_to_joint_idx_map_;
  double phase_duration_;

  int state_port_;
  int des_yaw_port_;
  int fsm_port_;
};

}  // namespace osc
}  // namespace cassie
}  // namespace dairlib
