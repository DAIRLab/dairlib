#pragma once

#include <drake/common/trajectories/trajectory.h>

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace examples {
namespace magna {
namespace systems {

class CartesianPoseTrajectoryGenerator
    : public drake::systems::LeafSystem<double> {
 public:
  CartesianPoseTrajectoryGenerator(
      const drake::multibody::MultibodyPlant<double>& plant,
      drake::systems::Context<double>* context, std::string end_effector_name);

  const drake::systems::InputPort<double>& get_input_port_robot_state() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>&
  get_input_port_target_cartesian_pose() const {
    return this->get_input_port(target_cartesian_pose_port_);
  }
  const drake::systems::OutputPort<double>&
  get_output_port_translation_trajectory() const {
    return this->get_output_port(translation_trajectory_port_);
  }
  const drake::systems::OutputPort<double>&
  get_output_port_rotation_trajectory() const {
    return this->get_output_port(rotation_trajectory_port_);
  }
  const drake::systems::OutputPort<double>&
  get_output_port_joint_trajectory() const {
    return this->get_output_port(joint_trajectory_port_);
  }

 private:
  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  void CalcTranslationTrajectory(
      const drake::systems::Context<double>& context,
      drake::trajectories::Trajectory<double>* output) const;
  void CalcRotationTrajectory(
      const drake::systems::Context<double>& context,
      drake::trajectories::Trajectory<double>* output) const;
  void CalcJointTrajectory(
      const drake::systems::Context<double>& context,
      drake::trajectories::Trajectory<double>* output) const;

  drake::systems::InputPortIndex state_port_;
  drake::systems::InputPortIndex target_cartesian_pose_port_;
  drake::systems::OutputPortIndex translation_trajectory_port_;
  drake::systems::OutputPortIndex rotation_trajectory_port_;
  drake::systems::OutputPortIndex joint_trajectory_port_;
  drake::systems::DiscreteStateIndex current_joint_position_index_;
  drake::systems::DiscreteStateIndex current_cartesian_pose_index_;
  drake::systems::DiscreteStateIndex current_time_index_;
  drake::systems::DiscreteStateIndex target_cartesian_pose_index_;

  double default_speed = 0.1;  // rad/s

  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* plant_context_;
  std::string end_effector_name_;
};
}  // namespace systems
}  // namespace magna
}  // namespace examples
}  // namespace dairlib
