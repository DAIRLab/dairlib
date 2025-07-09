#include "robot_input_systems.h"

#include "multibody/multibody_utils.h"

using Eigen::VectorXd;

using drake::multibody::JointActuatorIndex;
using drake::systems::Context;

namespace dairlib {
namespace systems {
namespace lcmt_systems {

// Receives lcmt_robot_input messages and outputs actuator efforts.
RobotInputConsumer::RobotInputConsumer(
    const drake::multibody::MultibodyPlant<double>& plant) {
  num_actuators_ = plant.num_actuators();
  actuator_index_map_ = multibody::MakeNameToActuatorsMap(plant);
  // Declare input port for lcmt_robot_input message.
  this->DeclareAbstractInputPort("lcmt_robot_input",
                                 drake::Value<dairlib::lcmt_robot_input>{});
  // Declare output port for actuator efforts vector.
  this->DeclareVectorOutputPort(
      "u, t", TimestampedVector<double>(num_actuators_),
      &RobotInputConsumer::CopyInputOut, {all_sources_ticket()});
}

// Converts lcmt_robot_input message to actuator effort vector.
void RobotInputConsumer::CopyInputOut(const Context<double>& context,
                                      TimestampedVector<double>* output) const {
  const drake::AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto& input_msg = input->get_value<dairlib::lcmt_robot_input>();

  VectorXd input_vector = VectorXd::Zero(num_actuators_);

  // Map efforts from message to correct actuator indices.
  for (int i = 0; i < input_msg.num_efforts; i++) {
    int j = actuator_index_map_.at(input_msg.effort_names[i]);
    input_vector(j) = input_msg.efforts[i];
  }
  output->SetDataVector(input_vector);
  output->set_timestamp(input_msg.utime * 1.0e-6);
}

RobotInputGenerator::RobotInputGenerator(
    const drake::multibody::MultibodyPlant<double>& plant) {
  num_actuators_ = plant.num_actuators();
  actuator_index_map_ = multibody::MakeNameToActuatorsMap(plant);

  // Store actuator names in order.
  for (JointActuatorIndex i(0); i < plant.num_actuators(); ++i) {
    ordered_actuator_names_.push_back(plant.get_joint_actuator(i).name());
  }

  // Declare input port for actuator efforts.
  this->DeclareVectorInputPort("u, t",
                               TimestampedVector<double>(num_actuators_));
  // Declare output port for lcmt_robot_input message.
  this->DeclareAbstractOutputPort("lcmt_robot_input",
                                  &RobotInputGenerator::OutputCommand);
}

// Converts actuator effort vector to lcmt_robot_input message.
void RobotInputGenerator::OutputCommand(
    const Context<double>& context,
    dairlib::lcmt_robot_input* input_msg) const {
  const TimestampedVector<double>* command =
      (TimestampedVector<double>*)this->EvalVectorInput(context, 0);

  input_msg->utime = command->get_timestamp() * 1e6;
  input_msg->num_efforts = num_actuators_;
  input_msg->effort_names.resize(num_actuators_);
  input_msg->efforts.resize(num_actuators_);
  // Fill message with actuator names and efforts.
  for (int i = 0; i < num_actuators_; i++) {
    input_msg->effort_names[i] = ordered_actuator_names_[i];
    if (std::isnan(command->GetAtIndex(i))) {
      input_msg->efforts[i] = 0;
    } else {
      input_msg->efforts[i] = command->GetAtIndex(i);
    }
  }
}
}  // namespace lcmt_systems
}  // namespace systems
}  // namespace dairlib