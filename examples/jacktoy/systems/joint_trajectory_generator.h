#pragma once

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {

class JointTrajectoryGenerator : public drake::systems::LeafSystem<double> {
 public:
  JointTrajectoryGenerator(
      const drake::multibody::MultibodyPlant<double>& plant,
      const Eigen::VectorXd& target_position);

  const drake::systems::InputPort<double>& get_input_port_robot_state() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_radio() const {
    return this->get_input_port(radio_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_joint(
      int joint_index) const {
    return this->get_output_port(joint_trajectory_ports_[joint_index]);
  }

 private:
  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  void CalcTraj(int joint_index, const drake::systems::Context<double>& context,
                drake::AbstractValue* traj) const;

  drake::systems::InputPortIndex state_port_;
  drake::systems::InputPortIndex radio_port_;
  std::vector<drake::systems::OutputPortIndex> joint_trajectory_ports_;
  drake::systems::DiscreteStateIndex initial_position_index_;
  drake::systems::DiscreteStateIndex initial_time_index_;
  const Eigen::VectorXd target_position_;

  double default_speed = 0.1;  // rad/s
};

}  // namespace dairlib
