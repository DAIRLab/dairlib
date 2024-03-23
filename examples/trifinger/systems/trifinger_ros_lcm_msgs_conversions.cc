#include "trifinger_ros_lcm_msgs_conversions.h"

#include <sstream>

#include "multibody/multibody_utils.h"
#include "nav_msgs/msg/odometry.hpp"
#include "trifinger_ros2_msgs/msg/trifinger_state.hpp"

namespace dairlib {
namespace systems {

using Eigen::VectorXd;

// LcmToRosTimestampedVector implementation
LcmToRosTimestampedVector::LcmToRosTimestampedVector(int vector_size)
    : vector_size_(vector_size) {
  this->DeclareVectorInputPort("u, t", TimestampedVector<double>(vector_size_));
  this->DeclareAbstractOutputPort("ROS Float64MultiArray",
                                  &LcmToRosTimestampedVector::ConvertToROS);
};

void LcmToRosTimestampedVector::ConvertToROS(
    const drake::systems::Context<double>& context,
    std_msgs::msg::Float64MultiArray* output) const {
  const TimestampedVector<double>* input =
      (TimestampedVector<double>*)this->EvalVectorInput(context, 0);

  output->data.clear();
  for (int i = 0; i < vector_size_; i++) {
    if (std::isnan(input->GetAtIndex(i))) {
      output->data.push_back(0);
    } else {
      output->data.push_back(input->GetAtIndex(i));
    }
  }
  output->data.push_back(input->get_timestamp());
}

/***************************************************************************************/
RosToLcmRobotState::RosToLcmRobotState(int num_positions, int num_velocities,
                                       int num_efforts)
    : num_positions_(num_positions),
      num_velocities_(num_velocities),
      num_efforts_(num_efforts) {
  this->DeclareAbstractInputPort(
      "Franka JointState topic",
      drake::Value<trifinger_ros2_msgs::msg::TrifingerState>());
  this->DeclareAbstractOutputPort("lcmt_robot_output",
                                  &RosToLcmRobotState::ConvertToLCM);
}

void RosToLcmRobotState::ConvertToLCM(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_robot_output* output) const {
  const drake::AbstractValue* const input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto& msg =
      input->get_value<trifinger_ros2_msgs::msg::TrifingerState>();

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
  if (msg.position.empty()) {
    for (int i = 0; i < num_positions_; i++) {
      output->position[i] = nan("");
    }
    for (int i = 0; i < num_velocities_; i++) {
      output->velocity[i] = nan("");
    }
    for (int i = 0; i < num_efforts_; i++) {
      output->effort[i] = nan("");
    }
    for (int i = 0; i < 3; i++) {
      output->imu_accel[i] = nan("");
    }
  } else {
    for (int i = 0; i < num_positions_; i++) {
      output->position[i] = msg.position[i];
    }
    for (int i = 0; i < num_velocities_; i++) {
      output->velocity[i] = msg.velocity[i];
    }
    for (int i = 0; i < num_efforts_; i++) {
      output->effort[i] = msg.torque[i];
    }
    for (int i = 0; i < 3; i++) {
      output->imu_accel[i] = 0;
    }
  }
  output->utime = context.get_time() * 1e6;
}

/***************************************************************************************/
RosToLcmObjectState::RosToLcmObjectState(
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::multibody::ModelInstanceIndex model_instance,
    const std::string& object_name) {
  auto positions_start_idx =
      plant.get_joint(plant.GetJointIndices(model_instance).front())
          .position_start();
  auto velocities_start_idx =
      plant.get_joint(plant.GetJointIndices(model_instance).front())
          .velocity_start();

  dairlib::lcmt_object_state object_state = dairlib::lcmt_object_state();
  object_state.num_positions = plant.num_positions(model_instance);
  object_state.num_velocities = plant.num_velocities(model_instance);
  object_state.object_name = object_name;
  object_state.position = std::vector<double>(object_state.num_positions);
  object_state.position_names = multibody::ExtractOrderedNamesFromMap(
      multibody::MakeNameToPositionsMap(plant, model_instance),
      positions_start_idx);
  object_state.velocity = std::vector<double>(object_state.num_velocities);
  object_state.velocity_names = multibody::ExtractOrderedNamesFromMap(
      multibody::MakeNameToVelocitiesMap(plant, model_instance),
      velocities_start_idx);
  object_state.position[0] = 1;
  this->DeclareAbstractOutputPort(object_name + "_state", object_state,
                                  &RosToLcmObjectState::ConvertToLCM);
  this->DeclareAbstractInputPort("ROS PoseWithCovariance",
                                 drake::Value<nav_msgs::msg::Odometry>());
}

void RosToLcmObjectState::ConvertToLCM(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_object_state* object_state) const {
  const drake::AbstractValue* input = this->EvalAbstractInput(context, 0);
  const auto& msg = input->get_value<nav_msgs::msg::Odometry>();

  if (msg.pose.pose.position.x == 0) {
    // do nothing when there is no message, just keep the most recent message1
  } else {
    object_state->position[0] = msg.pose.pose.orientation.w;
    object_state->position[1] = msg.pose.pose.orientation.x;
    object_state->position[2] = msg.pose.pose.orientation.y;
    object_state->position[3] = msg.pose.pose.orientation.z;
    object_state->position[4] = msg.pose.pose.position.x;
    object_state->position[5] = msg.pose.pose.position.y;
    object_state->position[6] = msg.pose.pose.position.z;
  }
  object_state->utime = context.get_time() * 1e6;
}

/***************************************************************************************/
RosToLcmFingertipsDeltaPosition::RosToLcmFingertipsDeltaPosition() {
  dairlib::lcmt_fingertips_delta_position delta_pos =
      dairlib::lcmt_fingertips_delta_position();
  for (int i = 0; i < n_delta_pos_; i++) {
    delta_pos.deltaPos[i] = 0.0;
  }
  this->DeclareAbstractOutputPort(
      "fingertips delta position lcm", delta_pos,
      &RosToLcmFingertipsDeltaPosition::ConvertToLCM);
  this->DeclareAbstractInputPort(
      "fingertips delta position ros",
      *drake::AbstractValue::Make<std_msgs::msg::Float64MultiArray>());
}

void RosToLcmFingertipsDeltaPosition::ConvertToLCM(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_fingertips_delta_position* output) const {
  const auto& msg = this->get_input_port(0)
      .Eval<std_msgs::msg::Float64MultiArray>(context);
  if (msg.data.size() == 0) {
    // do nothing when there is no message, just keep the most recent message
  } else {
    for (int i = 0; i < n_delta_pos_; i++) {
      output->deltaPos[i] = msg.data[i];
    }
  }
  output->utime = context.get_time() * 1e6;
}
}  // namespace systems
}  // namespace dairlib