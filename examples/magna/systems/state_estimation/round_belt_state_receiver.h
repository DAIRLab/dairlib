#pragma once

#include "dairlib/lcmt_round_belt_state.hpp"

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace examples {
namespace magna {
namespace systems {
namespace state_estimation {

class RoundBeltStateReceiver : public drake::systems::LeafSystem<double> {
 public:
  explicit RoundBeltStateReceiver(
      drake::multibody::MultibodyPlant<double>& franka_plant,
      drake::systems::Context<double>* franka_context,
      const std::string& end_effector_name,
      const std::vector<double>& taskboard_position,
      const std::vector<double>& taskboard_orientation);
  const drake::systems::InputPort<double>& get_input_port_franka_state() const {
    return this->get_input_port(franka_state_input_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_round_belt_state()
      const {
    return this->get_input_port(round_belt_state_wrt_taskboard_input_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_keypoint_state()
      const {
    return this->get_output_port(round_belt_state_wrt_robot_output_port_);
  }
  const drake::systems::OutputPort<double>&
  get_output_port_interested_keypoints() const {
    return this->get_output_port(interested_keypoints_output_port_);
  }
  void CopyRoundBeltStateWrtRobotToOutput(
      const drake::systems::Context<double>& context,
      dairlib::lcmt_round_belt_state* output) const;
  void CopyInterestedKeypointsToOutput(
      const drake::systems::Context<double>& context,
      dairlib::lcmt_round_belt_state* output) const;

 private:
  void CopyKeypointsToOutput(const drake::systems::Context<double>& context,
                             const std::vector<Eigen::Vector3d>& positions,
                             dairlib::lcmt_round_belt_state* output) const;
  drake::systems::EventStatus UpdateBeltStateWrtRobot(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;
  drake::multibody::MultibodyPlant<double>& franka_plant_;
  drake::systems::Context<double>* franka_context_;
  const std::string end_effector_name_;
  const std::vector<double> taskboard_position_;
  const std::vector<double> taskboard_orientation_;
  drake::math::RigidTransform<double> taskboard_transform_;
  drake::systems::InputPortIndex round_belt_state_wrt_taskboard_input_port_;
  drake::systems::InputPortIndex franka_state_input_port_;
  drake::systems::OutputPortIndex round_belt_state_wrt_robot_output_port_;
  drake::systems::OutputPortIndex interested_keypoints_output_port_;
  mutable std::vector<Eigen::Vector3d> keypoint_positions_wrt_robot_;
  mutable std::vector<Eigen::Vector3d>
      interested_keypoints_positions_wrt_robot_;
  mutable Eigen::Vector3d end_effector_position_wrt_robot_;
};

}  // namespace state_estimation
}  // namespace systems
}  // namespace magna
}  // namespace examples
}  // namespace dairlib
