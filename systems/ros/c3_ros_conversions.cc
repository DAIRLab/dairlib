#include "c3_ros_conversions.h"

namespace dairlib {
namespace systems {

using Eigen::VectorXd;

/***************************************************************************************/
// TimestampedVectorToROS implementation

TimestampedVectorToROS::TimestampedVectorToROS(int num_elements)
  : num_elements_(num_elements) {
  
  this->DeclareVectorInputPort("u, t",
    TimestampedVector<double>(num_elements_));
  this->DeclareAbstractOutputPort("ROS Float64MultiArray",
                                  &TimestampedVectorToROS::ConvertToROS);
};

void TimestampedVectorToROS::ConvertToROS(const drake::systems::Context<double>& context,
                std_msgs::Float64MultiArray* output) const {
  
  const TimestampedVector<double>* input =
    (TimestampedVector<double>*)this->EvalVectorInput(context, 0);
  
  // note: this function currently appends the timestamp to the end of the output
  output->data.clear();
  for (int i = 0; i < num_elements_; i++){
    if (std::isnan(input->GetAtIndex(i))){
      output->data.push_back(0);
    }
    else{
      output->data.push_back(input->GetAtIndex(i));
    }
  }
  output->data.push_back(input->get_timestamp()); // TODO: figure out if 1e6 needed
}

/***************************************************************************************/
// ROSToRobotOuput implementation

ROSToRobotOutputLCM::ROSToRobotOutputLCM(int num_positions, int num_velocities, int num_efforts)
  : num_positions_(num_positions), num_velocities_(num_velocities), num_efforts_(num_efforts) {

  this->DeclareAbstractInputPort("Franka JointState topic", 
    drake::Value<sensor_msgs::JointState>());
  this->DeclareAbstractOutputPort("lcmt_robot_output",
    &ROSToRobotOutputLCM::ConvertToLCM);
}

void ROSToRobotOutputLCM::ConvertToLCM(const drake::systems::Context<double>& context,
                  dairlib::lcmt_robot_output* output) const {

  const drake::AbstractValue* const input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto& msg = input->get_value<sensor_msgs::JointState>();

  output->num_positions = num_positions_;
  output->num_velocities = num_velocities_;
  output->num_efforts = num_efforts_;
  output->position_names.resize(num_positions_);
  output->velocity_names.resize(num_velocities_);
  output->effort_names.resize(num_efforts_);
  output->position_names = position_names_;
  output->velocity_names = velocity_names_;
  output->effort_names = effort_names_;
  output->position.resize(num_positions_);
  output->velocity.resize(num_velocities_);
  output->effort.resize(num_efforts_);
  
  // yet to receive message from rostopic
  if (msg.position.empty()){
    for (int i = 0; i < num_positions_; i++){
      output->position[i] = nan("");
    }
    for (int i = 0; i < num_velocities_; i++){
      output->velocity[i] = nan("");
    }
    for (int i = 0; i < num_efforts_; i++){
      output->effort[i] = nan("");
    }
    for (int i = 0; i < 3; i++){
      output->imu_accel[i] = nan("");
    }
  }
  // input should be order as "positions, velocities, effort, timestamp"
  else{
    for (int i = 0; i < num_franka_joints_; i++){
      output->position[i] = msg.position[i];
    }
    // hard coded to add 7 zeros to the end of position
    for (int i = num_franka_joints_; i < num_positions_; i++){
      output->position[i] = default_ball_position_[i-num_franka_joints_];
    }

    for (int i = 0; i < num_velocities_; i++){
      output->velocity[i] = msg.velocity[i];
    }
    // hard coded to add 6 zeros to the end of velocity
    for (int i = num_franka_joints_; i < num_velocities_; i++){
      output->velocity[i] = 0;
    }
    
    for (int i = 0; i < num_efforts_; i++){
      output->effort[i] = msg.effort[i];
    }
    for (int i = 0; i < 3; i++){
      output->imu_accel[i] = 0;
    }
  }
  // TODO: figure out which time to use
  output->utime = context.get_time() * 1e6;
  //output->utime = msg.head.stamp.secs
  //output->utime = msg.data[num_positions_+num_velocities_+num_efforts_];
}

/***************************************************************************************/
// ROSToC3LCM implementation

ROSToC3LCM::ROSToC3LCM(int num_positions, int num_velocities,
    int lambda_size, int misc_size)
  : num_positions_(num_positions), num_velocities_(num_velocities),
    lambda_size_(lambda_size), misc_size_(misc_size) {

  this->DeclareAbstractInputPort("ROS Float64MultiArray", 
    drake::Value<std_msgs::Float64MultiArray>());
  this->DeclareAbstractOutputPort("lcmt_c3",
    &ROSToC3LCM::ConvertToLCM);
  
  data_size_ = num_positions_+num_velocities_+lambda_size_+misc_size_;
}

void ROSToC3LCM::ConvertToLCM(const drake::systems::Context<double>& context,
                  dairlib::lcmt_c3* c3_msg) const {

  const drake::AbstractValue* input = this->EvalAbstractInput(context, 0);
  const auto& msg = input->get_value<std_msgs::Float64MultiArray>();

  c3_msg->data_size = data_size_;
  c3_msg->data.resize(data_size_);
  if (msg.data.empty()){
    for (int i = 0; i < data_size_; i++){
      c3_msg->data[i] = nan("");
    }
  }
  else{
    for (int i = 0; i < data_size_; i++){
      c3_msg->data[i] = msg.data[i];
    }
  }
  // TODO: figure out which time to use
  c3_msg->utime = context.get_time() * 1e6;
  // c3_msg->utime = msg.data[data_size_];
}

}  // namespace systems
}  // namespace dairlib
