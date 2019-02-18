#include "systems/sensors/sim_cassie_sensor_aggregator.h"

namespace dairlib {
namespace systems {

using std::string;
using drake::systems::Context;
using drake::systems::AbstractValue;
using dairlib::systems::TimestampedVector;
using Eigen::VectorXd;
using drake::systems::Context;
using std::map;
using std::cout;
using std::endl;

SimCassieSensorAggregator::SimCassieSensorAggregator(const RigidBodyTree<double>& tree) {
  num_positions_ = tree.get_num_positions();
  num_velocities_ = tree.get_num_velocities();

  positionIndexMap_ = multibody::makeNameToPositionsMap(tree);
  velocityIndexMap_ = multibody::makeNameToVelocitiesMap(tree);
  actuatorIndexMap_ = multibody::makeNameToActuatorsMap(tree);

  input_input_port_ = this->DeclareVectorInputPort(BasicVector<double>(10)).get_index();
  state_input_port_ = this->DeclareVectorInputPort(BasicVector<double>(
                        num_positions_ + num_velocities_)).get_index();
  acce_input_port_ = this->DeclareVectorInputPort(BasicVector<double>(3)).get_index();
  gyro_input_port_ = this->DeclareVectorInputPort(BasicVector<double>(3)).get_index();
  this->DeclareAbstractOutputPort(&SimCassieSensorAggregator::Aggregator);
}

/// read and parse a configuration LCM message
void SimCassieSensorAggregator::Aggregator(const Context<double>& context,
    dairlib::lcmt_cassie_out* cassie_out_msg) const {
  const auto input = this->EvalVectorInput(context, input_input_port_);
  const auto state = this->EvalVectorInput(context, state_input_port_);
  const auto acce = this->EvalVectorInput(context, acce_input_port_);
  const auto gyro = this->EvalVectorInput(context, gyro_input_port_);

  // using the time from the context
  cassie_out_msg->utime = context.get_time() * 1e6;
  cout << cassie_out_msg->utime / 1e6 << endl;

  // Motor position, velocity and torque
  cassie_out_msg->leftLeg.hipRollDrive.position = state->GetAtIndex(positionIndexMap_.at(motor_position_names_[0]));
  cassie_out_msg->rightLeg.hipRollDrive.position = state->GetAtIndex(positionIndexMap_.at(motor_position_names_[1]));
  cassie_out_msg->leftLeg.hipYawDrive.position = state->GetAtIndex(positionIndexMap_.at(motor_position_names_[2]));
  cassie_out_msg->rightLeg.hipYawDrive.position = state->GetAtIndex(positionIndexMap_.at(motor_position_names_[3]));
  cassie_out_msg->leftLeg.hipPitchDrive.position = state->GetAtIndex(positionIndexMap_.at(motor_position_names_[4]));
  cassie_out_msg->rightLeg.hipPitchDrive.position = state->GetAtIndex(positionIndexMap_.at(motor_position_names_[5]));
  cassie_out_msg->leftLeg.kneeDrive.position = state->GetAtIndex(positionIndexMap_.at(motor_position_names_[6]));
  cassie_out_msg->rightLeg.kneeDrive.position = state->GetAtIndex(positionIndexMap_.at(motor_position_names_[7]));
  cassie_out_msg->leftLeg.footDrive.position = state->GetAtIndex(positionIndexMap_.at(motor_position_names_[8]));
  cassie_out_msg->rightLeg.footDrive.position = state->GetAtIndex(positionIndexMap_.at(motor_position_names_[9]));

  cassie_out_msg->leftLeg.hipRollDrive.velocity = state->GetAtIndex(num_positions_ + velocityIndexMap_.at(motor_velocity_names_[0]));
  cassie_out_msg->rightLeg.hipRollDrive.velocity = state->GetAtIndex(num_positions_ + velocityIndexMap_.at(motor_velocity_names_[1]));
  cassie_out_msg->leftLeg.hipYawDrive.velocity = state->GetAtIndex(num_positions_ + velocityIndexMap_.at(motor_velocity_names_[2]));
  cassie_out_msg->rightLeg.hipYawDrive.velocity = state->GetAtIndex(num_positions_ + velocityIndexMap_.at(motor_velocity_names_[3]));
  cassie_out_msg->leftLeg.hipPitchDrive.velocity = state->GetAtIndex(num_positions_ + velocityIndexMap_.at(motor_velocity_names_[4]));
  cassie_out_msg->rightLeg.hipPitchDrive.velocity = state->GetAtIndex(num_positions_ + velocityIndexMap_.at(motor_velocity_names_[5]));
  cassie_out_msg->leftLeg.kneeDrive.velocity = state->GetAtIndex(num_positions_ + velocityIndexMap_.at(motor_velocity_names_[6]));
  cassie_out_msg->rightLeg.kneeDrive.velocity = state->GetAtIndex(num_positions_ + velocityIndexMap_.at(motor_velocity_names_[7]));
  cassie_out_msg->leftLeg.footDrive.velocity = state->GetAtIndex(num_positions_ + velocityIndexMap_.at(motor_velocity_names_[8]));
  cassie_out_msg->rightLeg.footDrive.velocity = state->GetAtIndex(num_positions_ + velocityIndexMap_.at(motor_velocity_names_[9]));

  cassie_out_msg->leftLeg.hipRollDrive.torque = input->GetAtIndex(0);
  cassie_out_msg->rightLeg.hipRollDrive.torque = input->GetAtIndex(1);
  cassie_out_msg->leftLeg.hipYawDrive.torque = input->GetAtIndex(2);
  cassie_out_msg->rightLeg.hipYawDrive.torque = input->GetAtIndex(3);
  cassie_out_msg->leftLeg.hipPitchDrive.torque = input->GetAtIndex(4);
  cassie_out_msg->rightLeg.hipPitchDrive.torque = input->GetAtIndex(5);
  cassie_out_msg->leftLeg.kneeDrive.torque = input->GetAtIndex(6);
  cassie_out_msg->rightLeg.kneeDrive.torque = input->GetAtIndex(7);
  cassie_out_msg->leftLeg.footDrive.torque = input->GetAtIndex(8);
  cassie_out_msg->rightLeg.footDrive.torque = input->GetAtIndex(9);

  // Joint position and velocity
  cassie_out_msg->leftLeg.shinJoint.position = state->GetAtIndex(positionIndexMap_.at(joint_position_names_[0]));
  cassie_out_msg->rightLeg.shinJoint.position = state->GetAtIndex(positionIndexMap_.at(joint_position_names_[1]));
  cassie_out_msg->leftLeg.tarsusJoint.position = state->GetAtIndex(positionIndexMap_.at(joint_position_names_[2]));
  cassie_out_msg->rightLeg.tarsusJoint.position = state->GetAtIndex(positionIndexMap_.at(joint_position_names_[3]));
  cassie_out_msg->leftLeg.footJoint.position = state->GetAtIndex(positionIndexMap_.at(joint_position_names_[4]));
  cassie_out_msg->rightLeg.footJoint.position = state->GetAtIndex(positionIndexMap_.at(joint_position_names_[5]));
  cassie_out_msg->leftLeg.shinJoint.velocity = state->GetAtIndex(num_positions_ + velocityIndexMap_.at(joint_velocity_names_[0]));
  cassie_out_msg->rightLeg.shinJoint.velocity = state->GetAtIndex(num_positions_ + velocityIndexMap_.at(joint_velocity_names_[1]));
  cassie_out_msg->leftLeg.tarsusJoint.velocity = state->GetAtIndex(num_positions_ + velocityIndexMap_.at(joint_velocity_names_[2]));
  cassie_out_msg->rightLeg.tarsusJoint.velocity = state->GetAtIndex(num_positions_ + velocityIndexMap_.at(joint_velocity_names_[3]));
  cassie_out_msg->leftLeg.footJoint.velocity = state->GetAtIndex(num_positions_ + velocityIndexMap_.at(joint_velocity_names_[4]));
  cassie_out_msg->rightLeg.footJoint.velocity = state->GetAtIndex(num_positions_ + velocityIndexMap_.at(joint_velocity_names_[5]));

  // Gyro and acceleration
  for (int i = 0; i < 3; i++) {
    cassie_out_msg->pelvis.vectorNav.linearAcceleration[i] = acce->GetAtIndex(i);
    cassie_out_msg->pelvis.vectorNav.angularVelocity[i] = gyro->GetAtIndex(i);
  }

  // // Printing
  // cout << "leftLeg.hipRollDrive.position = "<< cassie_out_msg->leftLeg.hipRollDrive.position << endl;
  // cout << "rightLeg.hipRollDrive.position = "<< cassie_out_msg->rightLeg.hipRollDrive.position << endl;
  // cout << "leftLeg.hipYawDrive.position = "<< cassie_out_msg->leftLeg.hipYawDrive.position << endl;
  // cout << "rightLeg.hipYawDrive.position = "<< cassie_out_msg->rightLeg.hipYawDrive.position << endl;
  // cout << "leftLeg.hipPitchDrive.position = "<< cassie_out_msg->leftLeg.hipPitchDrive.position << endl;
  // cout << "rightLeg.hipPitchDrive.position = "<< cassie_out_msg->rightLeg.hipPitchDrive.position << endl;
  // cout << "leftLeg.kneeDrive.position = "<< cassie_out_msg->leftLeg.kneeDrive.position << endl;
  // cout << "rightLeg.kneeDrive.position = "<< cassie_out_msg->rightLeg.kneeDrive.position << endl;
  // cout << "leftLeg.footDrive.position = "<< cassie_out_msg->leftLeg.footDrive.position << endl;
  // cout << "rightLeg.footDrive.position = "<< cassie_out_msg->rightLeg.footDrive.position << endl;
  // cout << "leftLeg.hipRollDrive.velocity = "<< cassie_out_msg->leftLeg.hipRollDrive.velocity << endl;
  // cout << "rightLeg.hipRollDrive.velocity = "<< cassie_out_msg->rightLeg.hipRollDrive.velocity << endl;
  // cout << "leftLeg.hipYawDrive.velocity = "<< cassie_out_msg->leftLeg.hipYawDrive.velocity << endl;
  // cout << "rightLeg.hipYawDrive.velocity = "<< cassie_out_msg->rightLeg.hipYawDrive.velocity << endl;
  // cout << "leftLeg.hipPitchDrive.velocity = "<< cassie_out_msg->leftLeg.hipPitchDrive.velocity << endl;
  // cout << "rightLeg.hipPitchDrive.velocity = "<< cassie_out_msg->rightLeg.hipPitchDrive.velocity << endl;
  // cout << "leftLeg.kneeDrive.velocity = "<< cassie_out_msg->leftLeg.kneeDrive.velocity << endl;
  // cout << "rightLeg.kneeDrive.velocity = "<< cassie_out_msg->rightLeg.kneeDrive.velocity << endl;
  // cout << "leftLeg.footDrive.velocity = "<< cassie_out_msg->leftLeg.footDrive.velocity << endl;
  // cout << "rightLeg.footDrive.velocity = "<< cassie_out_msg->rightLeg.footDrive.velocity << endl;

  // cout << "leftLeg.shinJoint.position = "<< cassie_out_msg->leftLeg.shinJoint.position << endl;
  // cout << "rightLeg.shinJoint.position = "<< cassie_out_msg->rightLeg.shinJoint.position << endl;
  // cout << "leftLeg.tarsusJoint.position = "<< cassie_out_msg->leftLeg.tarsusJoint.position << endl;
  // cout << "rightLeg.tarsusJoint.position = "<< cassie_out_msg->rightLeg.tarsusJoint.position << endl;
  // cout << "leftLeg.footJoint.position = "<< cassie_out_msg->leftLeg.footJoint.position << endl;
  // cout << "rightLeg.footJoint.position = "<< cassie_out_msg->rightLeg.footJoint.position << endl;
  // cout << "leftLeg.shinJoint.velocity = "<< cassie_out_msg->leftLeg.shinJoint.velocity << endl;
  // cout << "rightLeg.shinJoint.velocity = "<< cassie_out_msg->rightLeg.shinJoint.velocity << endl;
  // cout << "leftLeg.tarsusJoint.velocity = "<< cassie_out_msg->leftLeg.tarsusJoint.velocity << endl;
  // cout << "rightLeg.tarsusJoint.velocity = "<< cassie_out_msg->rightLeg.tarsusJoint.velocity << endl;
  // cout << "leftLeg.footJoint.velocity = "<< cassie_out_msg->leftLeg.footJoint.velocity << endl;
  // cout << "rightLeg.footJoint.velocity< = "<< cassie_out_msg->rightLeg.footJoint.velocity<< endl;

  // cout << "leftLeg.hipRollDrive.torque = "<< cassie_out_msg->leftLeg.hipRollDrive.torque << endl;
  // cout << "rightLeg.hipRollDrive.torque = "<< cassie_out_msg->rightLeg.hipRollDrive.torque << endl;
  // cout << "leftLeg.hipYawDrive.torque = "<< cassie_out_msg->leftLeg.hipYawDrive.torque << endl;
  // cout << "rightLeg.hipYawDrive.torque = "<< cassie_out_msg->rightLeg.hipYawDrive.torque << endl;
  // cout << "leftLeg.hipPitchDrive.torque = "<< cassie_out_msg->leftLeg.hipPitchDrive.torque << endl;
  // cout << "rightLeg.hipPitchDrive.torque = "<< cassie_out_msg->rightLeg.hipPitchDrive.torque << endl;
  // cout << "leftLeg.kneeDrive.torque = "<< cassie_out_msg->leftLeg.kneeDrive.torque << endl;
  // cout << "rightLeg.kneeDrive.torque = "<< cassie_out_msg->rightLeg.kneeDrive.torque << endl;
  // cout << "leftLeg.footDrive.torque = "<< cassie_out_msg->leftLeg.footDrive.torque << endl;
  // cout << "rightLeg.footDrive.torque < = "<< cassie_out_msg->rightLeg.footDrive.torque << endl;

  // for (int i = 0; i < 3; i++) {
  //   cout << "pelvis.vectorNav.linearAcceleration["<<"i"<<"] = "<< cassie_out_msg->pelvis.vectorNav.linearAcceleration[i] << endl;
  //   cout << "pelvis.vectorNav.angularVelocity["<<"i"<<"] = "<< cassie_out_msg->pelvis.vectorNav.angularVelocity[i] << endl;
  // }


}

}  // namespace systems
}  // namespace dairlib
