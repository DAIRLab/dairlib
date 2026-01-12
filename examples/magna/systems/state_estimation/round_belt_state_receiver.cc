#include "examples/magna/systems/state_estimation/round_belt_state_receiver.h"

#include "systems/framework/output_vector.h"

#include "drake/math/rigid_transform.h"

namespace dairlib {
namespace examples {
namespace magna {
namespace systems {
namespace state_estimation {

using dairlib::systems::OutputVector;

RoundBeltStateReceiver::RoundBeltStateReceiver(
    drake::multibody::MultibodyPlant<double>& franka_plant,
    drake::systems::Context<double>* franka_context,
    const std::string& end_effector_name,
    const std::vector<double>& taskboard_position,
    const std::vector<double>& taskboard_orientation)
    : franka_plant_(franka_plant),
      franka_context_(franka_context),
      end_effector_name_(end_effector_name),
      taskboard_position_(taskboard_position),
      taskboard_orientation_(taskboard_orientation) {
  round_belt_state_wrt_taskboard_input_port_ =
      this->DeclareAbstractInputPort(
              "round_belt_state_wrt_taskboard",
              drake::Value<dairlib::lcmt_round_belt_state>())
          .get_index();
  franka_state_input_port_ =
      this->DeclareVectorInputPort(
              "x_franka", OutputVector<double>(franka_plant.num_positions(),
                                               franka_plant.num_velocities(),
                                               franka_plant.num_actuators()))
          .get_index();

  taskboard_transform_ = drake::math::RigidTransform<double>(
      drake::math::RollPitchYaw<double>(taskboard_orientation_[0],
                                        taskboard_orientation_[1],
                                        taskboard_orientation_[2]),
      Eigen::Vector3d(taskboard_position_[0], taskboard_position_[1],
                      taskboard_position_[2]));
  round_belt_state_wrt_robot_output_port_ =
      this->DeclareAbstractOutputPort(
              "round_belt_state_wrt_robot",
              &RoundBeltStateReceiver::CopyRoundBeltStateWrtRobotToOutput)
          .get_index();
  this->DeclareForcedDiscreteUpdateEvent(
      &RoundBeltStateReceiver::UpdateBeltStateWrtRobot);
}

drake::systems::EventStatus RoundBeltStateReceiver::UpdateBeltStateWrtRobot(
    const drake::systems::Context<double>& context,
    drake::systems::DiscreteValues<double>* discrete_state) const {
  // Perform forward kinematics to get the end effector position in the robot
  // frame
  const OutputVector<double>* franka_state =
      (OutputVector<double>*)this->EvalVectorInput(context,
                                                   franka_state_input_port_);
  franka_plant_.SetPositions(franka_context_, franka_state->GetPositions());
  franka_plant_.SetVelocities(franka_context_, franka_state->GetVelocities());
  auto end_effector_pose = franka_plant_.EvalBodyPoseInWorld(
      *franka_context_, franka_plant_.GetBodyByName(end_effector_name_));
  end_effector_position_wrt_robot_ = end_effector_pose.translation();

  // Convert the positions of the keypoints in the taskboard frame to the robot
  // frame
  const auto round_belt_state_wrt_taskboard_input = this->EvalAbstractInput(
      context, round_belt_state_wrt_taskboard_input_port_);
  const auto& round_belt_state_wrt_taskboard_msg =
      round_belt_state_wrt_taskboard_input
          ->get_value<dairlib::lcmt_round_belt_state>();
  for (int i = 0; i < round_belt_state_wrt_taskboard_msg.num_points; i++) {
    Eigen::Vector3d keypoint_position_wrt_taskboard;
    keypoint_position_wrt_taskboard
        << round_belt_state_wrt_taskboard_msg.point_positions[i][0],
        round_belt_state_wrt_taskboard_msg.point_positions[i][1],
        round_belt_state_wrt_taskboard_msg.point_positions[i][2];
    keypoint_positions_wrt_robot_.push_back(
        taskboard_transform_.rotation() * keypoint_position_wrt_taskboard +
        taskboard_transform_.translation());
  }
  return drake::systems::EventStatus::Succeeded();
}

void RoundBeltStateReceiver::CopyRoundBeltStateWrtRobotToOutput(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_round_belt_state* output) const {
  output->utime = context.get_time() * 1e6;
  output->frame_name = "robot";
  output->num_points = keypoint_positions_wrt_robot_.size();
  output->point_positions.resize(output->num_points);
  for (int i = 0; i < output->num_points; i++) {
    output->point_positions[i].resize(3);
    output->point_positions[i][0] = keypoint_positions_wrt_robot_[i][0];
    output->point_positions[i][1] = keypoint_positions_wrt_robot_[i][1];
    output->point_positions[i][2] = keypoint_positions_wrt_robot_[i][2];
  }
}
}  // namespace state_estimation
}  // namespace systems
}  // namespace magna
}  // namespace examples
}  // namespace dairlib
