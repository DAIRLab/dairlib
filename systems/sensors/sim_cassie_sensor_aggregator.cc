#include "systems/sensors/sim_cassie_sensor_aggregator.h"

namespace dairlib {
namespace systems {

using std::string;
using drake::systems::Context;
using drake::AbstractValue;
using dairlib::systems::TimestampedVector;
using Eigen::VectorXd;
using drake::systems::Context;
using std::map;
using std::cout;
using std::endl;

SimCassieSensorAggregator::SimCassieSensorAggregator(
  const RigidBodyTree<double>& tree) {
  num_positions_ = tree.get_num_positions();
  num_velocities_ = tree.get_num_velocities();

  positionIndexMap_ = multibody::makeNameToPositionsMap(tree);
  velocityIndexMap_ = multibody::makeNameToVelocitiesMap(tree);
  actuatorIndexMap_ = multibody::makeNameToActuatorsMap(tree);

  input_input_port_ = this->DeclareVectorInputPort(
                        BasicVector<double>(10)).get_index();
  state_input_port_ = this->DeclareVectorInputPort(
            BasicVector<double>(num_positions_ + num_velocities_)).get_index();
  acce_input_port_ = this->DeclareVectorInputPort(
                       BasicVector<double>(3)).get_index();
  gyro_input_port_ = this->DeclareVectorInputPort(
                       BasicVector<double>(3)).get_index();
  this->DeclareAbstractOutputPort(&SimCassieSensorAggregator::Aggregate);
}

void SimCassieSensorAggregator::Aggregate(const Context<double>& context,
    dairlib::lcmt_cassie_out* cassie_out_msg) const {
  const auto input = this->EvalVectorInput(context, input_input_port_);
  const auto state = this->EvalVectorInput(context, state_input_port_);
  const auto accel = this->EvalVectorInput(context, acce_input_port_);
  const auto gyro = this->EvalVectorInput(context, gyro_input_port_);

  // using the time from the context
  cassie_out_msg->utime = context.get_time() * 1e6;

  // Motor position, velocity and torque
  cassie_out_msg->leftLeg.hipRollDrive.position = state->GetAtIndex(
        positionIndexMap_.at("hip_roll_left"));
  cassie_out_msg->rightLeg.hipRollDrive.position = state->GetAtIndex(
        positionIndexMap_.at("hip_roll_right"));
  cassie_out_msg->leftLeg.hipYawDrive.position = state->GetAtIndex(
        positionIndexMap_.at("hip_yaw_left"));
  cassie_out_msg->rightLeg.hipYawDrive.position = state->GetAtIndex(
        positionIndexMap_.at("hip_yaw_right"));
  cassie_out_msg->leftLeg.hipPitchDrive.position = state->GetAtIndex(
        positionIndexMap_.at("hip_pitch_left"));
  cassie_out_msg->rightLeg.hipPitchDrive.position = state->GetAtIndex(
        positionIndexMap_.at("hip_pitch_right"));
  cassie_out_msg->leftLeg.kneeDrive.position = state->GetAtIndex(
        positionIndexMap_.at("knee_left"));
  cassie_out_msg->rightLeg.kneeDrive.position = state->GetAtIndex(
        positionIndexMap_.at("knee_right"));
  cassie_out_msg->leftLeg.footDrive.position = state->GetAtIndex(
        positionIndexMap_.at("toe_left"));
  cassie_out_msg->rightLeg.footDrive.position = state->GetAtIndex(
        positionIndexMap_.at("toe_right"));

  cassie_out_msg->leftLeg.hipRollDrive.velocity = state->GetAtIndex(
        num_positions_ + velocityIndexMap_.at("hip_roll_leftdot"));
  cassie_out_msg->rightLeg.hipRollDrive.velocity = state->GetAtIndex(
        num_positions_ + velocityIndexMap_.at("hip_roll_rightdot"));
  cassie_out_msg->leftLeg.hipYawDrive.velocity = state->GetAtIndex(
        num_positions_ + velocityIndexMap_.at("hip_yaw_leftdot"));
  cassie_out_msg->rightLeg.hipYawDrive.velocity = state->GetAtIndex(
        num_positions_ + velocityIndexMap_.at("hip_yaw_rightdot"));
  cassie_out_msg->leftLeg.hipPitchDrive.velocity = state->GetAtIndex(
        num_positions_ + velocityIndexMap_.at("hip_pitch_leftdot"));
  cassie_out_msg->rightLeg.hipPitchDrive.velocity = state->GetAtIndex(
        num_positions_ + velocityIndexMap_.at("hip_pitch_rightdot"));
  cassie_out_msg->leftLeg.kneeDrive.velocity = state->GetAtIndex(
        num_positions_ + velocityIndexMap_.at("knee_leftdot"));
  cassie_out_msg->rightLeg.kneeDrive.velocity = state->GetAtIndex(
        num_positions_ + velocityIndexMap_.at("knee_rightdot"));
  cassie_out_msg->leftLeg.footDrive.velocity = state->GetAtIndex(
        num_positions_ + velocityIndexMap_.at("toe_leftdot"));
  cassie_out_msg->rightLeg.footDrive.velocity = state->GetAtIndex(
        num_positions_ + velocityIndexMap_.at("toe_rightdot"));

  cassie_out_msg->leftLeg.hipRollDrive.torque = input->GetAtIndex(
        actuatorIndexMap_.at("hip_roll_left_motor"));
  cassie_out_msg->rightLeg.hipRollDrive.torque = input->GetAtIndex(
        actuatorIndexMap_.at("hip_roll_right_motor"));
  cassie_out_msg->leftLeg.hipYawDrive.torque = input->GetAtIndex(
        actuatorIndexMap_.at("hip_yaw_left_motor"));
  cassie_out_msg->rightLeg.hipYawDrive.torque = input->GetAtIndex(
        actuatorIndexMap_.at("hip_yaw_right_motor"));
  cassie_out_msg->leftLeg.hipPitchDrive.torque = input->GetAtIndex(
        actuatorIndexMap_.at("hip_pitch_left_motor"));
  cassie_out_msg->rightLeg.hipPitchDrive.torque = input->GetAtIndex(
        actuatorIndexMap_.at("hip_pitch_right_motor"));
  cassie_out_msg->leftLeg.kneeDrive.torque = input->GetAtIndex(
        actuatorIndexMap_.at("knee_left_motor"));
  cassie_out_msg->rightLeg.kneeDrive.torque = input->GetAtIndex(
        actuatorIndexMap_.at("knee_right_motor"));
  cassie_out_msg->leftLeg.footDrive.torque = input->GetAtIndex(
        actuatorIndexMap_.at("toe_left_motor"));
  cassie_out_msg->rightLeg.footDrive.torque = input->GetAtIndex(
        actuatorIndexMap_.at("toe_right_motor"));

  // Joint position and velocity
  cassie_out_msg->leftLeg.shinJoint.position = state->GetAtIndex(
        positionIndexMap_.at("knee_joint_left"));
  cassie_out_msg->rightLeg.shinJoint.position = state->GetAtIndex(
        positionIndexMap_.at("knee_joint_right"));
  cassie_out_msg->leftLeg.tarsusJoint.position = state->GetAtIndex(
        positionIndexMap_.at("ankle_joint_left"));
  cassie_out_msg->rightLeg.tarsusJoint.position = state->GetAtIndex(
        positionIndexMap_.at("ankle_joint_right"));
  cassie_out_msg->leftLeg.footJoint.position = state->GetAtIndex(
        positionIndexMap_.at("toe_left"));
  cassie_out_msg->rightLeg.footJoint.position = state->GetAtIndex(
        positionIndexMap_.at("toe_right"));
  cassie_out_msg->leftLeg.shinJoint.velocity = state->GetAtIndex(
        num_positions_ + velocityIndexMap_.at("knee_joint_leftdot"));
  cassie_out_msg->rightLeg.shinJoint.velocity = state->GetAtIndex(
        num_positions_ + velocityIndexMap_.at("knee_joint_rightdot"));
  cassie_out_msg->leftLeg.tarsusJoint.velocity = state->GetAtIndex(
        num_positions_ + velocityIndexMap_.at("ankle_joint_leftdot"));
  cassie_out_msg->rightLeg.tarsusJoint.velocity = state->GetAtIndex(
        num_positions_ + velocityIndexMap_.at("ankle_joint_rightdot"));
  cassie_out_msg->leftLeg.footJoint.velocity = state->GetAtIndex(
        num_positions_ + velocityIndexMap_.at("toe_leftdot"));
  cassie_out_msg->rightLeg.footJoint.velocity = state->GetAtIndex(
        num_positions_ + velocityIndexMap_.at("toe_rightdot"));

  // Gyro and acceleration
  for (int i = 0; i < 3; i++) {
    cassie_out_msg->pelvis.vectorNav.linearAcceleration[i] =
      accel->GetAtIndex(i);
    cassie_out_msg->pelvis.vectorNav.angularVelocity[i] = gyro->GetAtIndex(i);
  }
}

}  // namespace systems
}  // namespace dairlib
