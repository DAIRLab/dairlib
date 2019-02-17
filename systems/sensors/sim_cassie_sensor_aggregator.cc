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
                               dairlib::lcmt_cassie_sensor* sensor_msg) const {
  const auto input = this->EvalVectorInput(context, input_input_port_);
  const auto state = this->EvalVectorInput(context, state_input_port_);
  const auto acce = this->EvalVectorInput(context, acce_input_port_);
  const auto gyro = this->EvalVectorInput(context, gyro_input_port_);

  // using the time from the context
  sensor_msg->timestamp = context.get_time() * 1e6;
  sensor_msg->num_motors = 10;
  sensor_msg->num_joints = 6;

  sensor_msg->motor_position_names.resize(sensor_msg->num_motors);
  sensor_msg->motor_velocity_names.resize(sensor_msg->num_motors);
  sensor_msg->joint_position_names.resize(sensor_msg->num_joints);
  sensor_msg->joint_velocity_names.resize(sensor_msg->num_joints);
  sensor_msg->effort_names.resize(sensor_msg->num_motors);
  sensor_msg->motor_position.resize(sensor_msg->num_motors);
  sensor_msg->motor_velocity.resize(sensor_msg->num_motors);
  sensor_msg->joint_position.resize(sensor_msg->num_joints);
  sensor_msg->joint_velocity.resize(sensor_msg->num_joints);
  sensor_msg->effort.resize(sensor_msg->num_motors);

  for (int i = 0; i < sensor_msg->num_motors; i++) {
    sensor_msg->motor_position_names[i] = motor_position_names_[i];
    sensor_msg->motor_position[i] = state->GetAtIndex(positionIndexMap_.at(motor_position_names_[i]));    
    sensor_msg->motor_velocity_names[i] = motor_velocity_names_[i];
    sensor_msg->motor_velocity[i] = state->GetAtIndex(num_positions_ + velocityIndexMap_.at(motor_velocity_names_[i]));
    sensor_msg->effort_names[i] = effort_names_[i];
    sensor_msg->effort[i] = input->GetAtIndex(i);
  }
   
  for (int i = 0; i < sensor_msg->num_joints; i++) {
    sensor_msg->joint_position_names[i] = joint_position_names_[i];
    sensor_msg->joint_position[i] = state->GetAtIndex(positionIndexMap_.at(joint_position_names_[i]));
    sensor_msg->joint_velocity_names[i] = joint_velocity_names_[i];
    sensor_msg->joint_velocity[i] = state->GetAtIndex(num_positions_ + velocityIndexMap_.at(joint_velocity_names_[i]));
  }

  for (int i = 0; i < 3; i++) {
    sensor_msg->imu_linear_acc_names[i] = imu_linear_acc_names_[i];
    sensor_msg->imu_linear_acc[i] = acce->GetAtIndex(i);
    sensor_msg->imu_ang_vel_names[i] = imu_ang_vel_names_[i];
    sensor_msg->imu_ang_vel[i] = gyro->GetAtIndex(i);
  }
  
  // Printing 
  // for (int i = 0; i < sensor_msg->num_motors; i++) 
  //   std::cout<<sensor_msg->motor_position_names[i] <<" ="<< sensor_msg->motor_position[i] << endl;
  // for (int i = 0; i < sensor_msg->num_motors; i++) 
  //   std::cout<<sensor_msg->motor_velocity_names[i] <<" ="<< sensor_msg->motor_velocity[i] << endl;
  // for (int i = 0; i < sensor_msg->num_joints; i++) 
  //   cout<< sensor_msg->joint_position_names[i] <<" = " << sensor_msg->joint_position[i] <<endl;
  // for (int i = 0; i < sensor_msg->num_joints; i++) 
  //   cout<< sensor_msg->joint_velocity_names[i]<<" = " << sensor_msg->joint_velocity[i] <<endl;
  // for (int i = 0; i < sensor_msg->num_motors; i++) 
  //   cout<< sensor_msg->effort_names[i] <<" = " << sensor_msg->effort[i] << endl;
  // for (int i = 0; i < 3; i++) 
  //   cout<< sensor_msg->imu_linear_acc_names[i]<<" = " << sensor_msg->imu_linear_acc[i] <<endl;
  // for (int i = 0; i < 3; i++) 
  //   cout<< sensor_msg->imu_ang_vel_names[i] <<" = " << sensor_msg->imu_ang_vel[i] <<endl;
  // cout<<endl;

}

}  // namespace systems
}  // namespace dairlib
