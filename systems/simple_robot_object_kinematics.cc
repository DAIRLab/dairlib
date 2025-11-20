#include "systems/simple_robot_object_kinematics.h"

#include <iostream>

#include "common/find_resource.h"

namespace dairlib {
namespace systems {

SimpleRobotObjectKinematics::SimpleRobotObjectKinematics(
    const MultibodyPlant<double>& plant, const ModelInstanceIndex& robot_idx,
    const ModelInstanceIndex& object_idx,
    const bool& enforce_msg_from_all_systems)
    : num_robot_positions_(plant.num_positions(robot_idx)),
      num_object_positions_(plant.num_positions(object_idx)),
      num_robot_velocities_(plant.num_velocities(robot_idx)),
      num_object_velocities_(plant.num_velocities(object_idx)),
      enforce_msg_from_all_systems_(enforce_msg_from_all_systems) {
  this->set_name("simple_robot_object_kinematics");
  robot_state_port_ =
      this->DeclareVectorInputPort(
              "x_robot",
              OutputVector<double>(num_robot_positions_, num_robot_velocities_,
                                   plant.num_actuators(robot_idx)))
          .get_index();

  object_state_port_ =
      this->DeclareVectorInputPort("x_object",
                                   StateVector<double>(num_object_positions_,
                                                       num_object_velocities_))
          .get_index();
  lcs_state_port_ = this->DeclareVectorOutputPort(
                            "x_lcs",
                            TimestampedVector<double>(
                                num_robot_positions_ + num_object_positions_ +
                                num_robot_velocities_ + num_object_velocities_),
                            &SimpleRobotObjectKinematics::ComputeLCSState)
                        .get_index();
}

void SimpleRobotObjectKinematics::ComputeLCSState(
    const drake::systems::Context<double>& context,
    TimestampedVector<double>* lcs_state) const {
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, robot_state_port_);
  const StateVector<double>* object_output =
      (StateVector<double>*)this->EvalVectorInput(context, object_state_port_);

  // Set the timestamp to 0 if either input has not been set yet.
  if (enforce_msg_from_all_systems_ &&
      ((robot_output->get_timestamp() < 1e-5) ||
       (object_output->get_timestamp() < 1e-5))) {
    lcs_state->set_timestamp(0.0);
    std::cout << "Missing either robot or object message..." << std::endl;
  } else {
    lcs_state->set_timestamp(robot_output->get_timestamp());
  }

  VectorXd q_robot = robot_output->GetPositions();
  VectorXd v_robot = robot_output->GetVelocities();
  VectorXd q_object = object_output->GetPositions();
  VectorXd v_object = object_output->GetVelocities();

  lcs_state->get_mutable_data().segment(0, num_robot_positions_) = q_robot;
  lcs_state->get_mutable_data().segment(num_robot_positions_,
                                        num_object_positions_) = q_object;
  lcs_state->get_mutable_data().segment(
      num_robot_positions_ + num_object_positions_, num_robot_velocities_) =
      v_robot;
  lcs_state->get_mutable_data().segment(
      num_robot_positions_ + num_object_positions_ + num_robot_velocities_,
      num_object_velocities_) = v_object;
}

}  // namespace systems
}  // namespace dairlib
