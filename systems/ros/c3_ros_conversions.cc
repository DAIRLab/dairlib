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

  this->DeclareAbstractInputPort("ROS Float64MultiArray", 
    drake::Value<std_msgs::Float64MultiArray>());
  this->DeclareAbstractOutputPort("lcmt_robot_output",
    &ROSToRobotOutputLCM::ConvertToLCM);
}

void ROSToRobotOutputLCM::ConvertToLCM(const drake::systems::Context<double>& context,
                  dairlib::lcmt_robot_output* output) const {

  const drake::AbstractValue* input = this->EvalAbstractInput(context, 0);
  const auto& msg = input->get_value<std_msgs::Float64MultiArray>();

  output->num_positions = num_positions_;
  output->num_velocities = num_velocities_;
  output->position_names.resize(num_positions_);
  output->velocity_names.resize(num_velocities_);
  output->position.resize(num_positions_);
  output->velocity.resize(num_velocities_);

  // input should be order as "positions, velocities, effort, timestamp"
  for (int i = 0; i < num_positions_; i++){
    output->position[i] = msg.data[i];
  }
  for (int i = 0; i < num_velocities_; i++){
    output->velocity[i] = msg.data[i+num_positions_];
  }
  for (int i = 0; i < num_efforts_; i++){
    output->effort[i] = msg.data[i+num_positions_+num_velocities_];
  }
  for (int i = 0; i < 3; i++){
    output->imu_accel[i] = 0;
  }
  // TODO: figure out which time to use
  output->utime = context.get_time() * 1e6;
  output->utime = msg.data[num_positions_+num_velocities_+num_efforts_];
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
  for (int i = 0; i < data_size_; i++){
    // TODO: not sure how to handle nan case since 
    // position estimate can send nan
    c3_msg->data[i] = msg.data[i];
  }
    // TODO: figure out which time to use
  c3_msg->utime = context.get_time() * 1e6;
  c3_msg->utime = msg.data[data_size_];
}

}  // namespace systems
}  // namespace dairlib
