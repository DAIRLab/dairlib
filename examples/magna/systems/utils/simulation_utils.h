#pragma once
#include "drake/common/trajectories/trajectory.h"
#include "drake/systems/framework/leaf_system.h"
namespace dairlib {
namespace examples {
namespace magna {
namespace systems {
namespace simulation_utils {
class FrankaHandCommandToTrajectory
    : public drake::systems::LeafSystem<double> {
 public:
  FrankaHandCommandToTrajectory();

  const drake::systems::InputPort<double>& get_input_port_franka_hand_command()
      const {
    return this->get_input_port(input_franka_hand_command_port_);
  }

  const drake::systems::OutputPort<double>&
  get_output_port_hand_position_trajectory() const {
    return this->get_output_port(hand_position_trajectory_port_);
  }

 private:
  void CalcHandPositionTrajectory(
      const drake::systems::Context<double>& context,
      drake::trajectories::Trajectory<double>* traj) const;
  drake::systems::InputPortIndex input_franka_hand_command_port_;
  drake::systems::OutputPortIndex hand_position_trajectory_port_;
};

}  // namespace simulation_utils
}  // namespace systems
}  // namespace magna
}  // namespace examples
}  // namespace dairlib
