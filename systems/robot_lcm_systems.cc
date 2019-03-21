#include "robot_lcm_systems.h"
#include "multibody/multibody_utils.h"


namespace dairlib {
namespace systems {

using std::string;
using Eigen::VectorXd;
using drake::systems::Context;
using drake::systems::LeafSystem;
using drake::multibody::JointActuatorIndex;
using drake::multibody::JointIndex;
using systems::OutputVector;


/*--------------------------------------------------------------------------*/
// methods implementation for RobotOutputReceiver.

RobotOutputReceiver::RobotOutputReceiver(
    const drake::multibody::MultibodyPlant<double>& plant) {
  num_positions_ = plant.num_positions();
  num_velocities_ = plant.num_velocities();
  positionIndexMap_ = multibody::makeNameToPositionsMap(plant);
  velocityIndexMap_ = multibody::makeNameToVelocitiesMap(plant);
  this->DeclareAbstractInputPort("lcmt_robot_output",
    drake::Value<dairlib::lcmt_robot_output>{});
  this->DeclareVectorOutputPort(OutputVector<double>(
    plant.num_positions(), plant.num_velocities(),
    plant.num_actuators()),
    &RobotOutputReceiver::CopyOutput);
}

// RBT constructor--to be deprecated when move to MBP is complete
RobotOutputReceiver::RobotOutputReceiver(
    const RigidBodyTree<double>& tree) {
  num_positions_ = tree.get_num_positions();
  num_velocities_ = tree.get_num_velocities();
  positionIndexMap_ = multibody::makeNameToPositionsMap(tree);
  velocityIndexMap_ = multibody::makeNameToVelocitiesMap(tree);
  this->DeclareAbstractInputPort("lcmt_robot_output",
    drake::Value<dairlib::lcmt_robot_output>{});
  this->DeclareVectorOutputPort(OutputVector<double>(
    tree.get_num_positions(), tree.get_num_velocities(),
    tree.get_num_actuators()),
    &RobotOutputReceiver::CopyOutput);
}

void RobotOutputReceiver::CopyOutput(
    const Context<double>& context, OutputVector<double>* output) const {
  const drake::AbstractValue* input =
      this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto& state_msg = input->get_value<dairlib::lcmt_robot_output>();

  VectorXd positions = VectorXd::Zero(num_positions_);
  for (int i = 0; i < state_msg.num_positions; i++) {
    int j = positionIndexMap_.at(state_msg.position_names[i]);
    positions(j) = state_msg.position[i];
  }
  VectorXd velocities = VectorXd::Zero(num_velocities_);
  for (int i = 0; i < state_msg.num_velocities; i++) {
    int j = velocityIndexMap_.at(state_msg.velocity_names[i]);
    velocities(j) = state_msg.velocity[i];
  }
  output->SetPositions(positions);
  output->SetVelocities(velocities);
  output->set_timestamp(state_msg.utime * 1.0e-6);
}

/*--------------------------------------------------------------------------*/
// methods implementation for RobotOutputSender.

// RBT constructor--to be deprecated when move to MBP is complete
RobotOutputSender::RobotOutputSender(const RigidBodyTree<double>& tree) {
  num_positions_ = tree.get_num_positions();
  num_velocities_ = tree.get_num_velocities();

  for (int i = 0; i < num_positions_; i++) {
    ordered_position_names_.push_back(tree.get_position_name(i));
  }

  for (int i = 0; i < num_velocities_; i++) {
    ordered_velocity_names_.push_back(tree.get_velocity_name(i));
  }

  positionIndexMap_ = multibody::makeNameToPositionsMap(tree);
  velocityIndexMap_ = multibody::makeNameToVelocitiesMap(tree);

  state_input_port_ = this->DeclareVectorInputPort(BasicVector<double>(
      num_positions_ + num_velocities_)).get_index();
  this->DeclareAbstractOutputPort(&RobotOutputSender::Output);
}

RobotOutputSender::RobotOutputSender(
    const drake::multibody::MultibodyPlant<double>& plant) {
  num_positions_ = plant.num_positions();
  num_velocities_ = plant.num_velocities();

  positionIndexMap_ = multibody::makeNameToPositionsMap(plant);
  velocityIndexMap_ = multibody::makeNameToVelocitiesMap(plant);

  // Loop through the maps to extract ordered names
  for (int i = 0; i < num_positions_; ++i) {
    for (auto& x : positionIndexMap_) {
      if (x.second == i) {
        ordered_position_names_.push_back(x.first);
        break;
      }
    }
  }

  for (int i = 0; i < num_velocities_; ++i) {
    for (auto& x : velocityIndexMap_) {
      if (x.second == i) {
        ordered_velocity_names_.push_back(x.first);
        break;
      }
    }
  }

  state_input_port_ = this->DeclareVectorInputPort(BasicVector<double>(
      num_positions_ + num_velocities_)).get_index();
  this->DeclareAbstractOutputPort(&RobotOutputSender::Output);
}

/// Populate a state message with all states
void RobotOutputSender::Output(const Context<double>& context,
                               dairlib::lcmt_robot_output* state_msg) const {
  const auto state = this->EvalVectorInput(context, state_input_port_);

  // using the time from the context
  state_msg->utime = context.get_time() * 1e6;

  state_msg->num_positions = num_positions_;
  state_msg->num_velocities = num_velocities_;
  state_msg->position_names.resize(num_positions_);
  state_msg->velocity_names.resize(num_velocities_);
  state_msg->position.resize(num_positions_);
  state_msg->velocity.resize(num_velocities_);

  for (int i = 0; i < num_positions_; i++) {
    state_msg->position[i] = state->GetAtIndex(i);
    state_msg->position_names[i] = ordered_position_names_[i];
  }
  for (int i = 0; i < num_velocities_; i++) {
    state_msg->velocity[i] = state->GetAtIndex(num_positions_ + i);
    state_msg->velocity_names[i] = ordered_velocity_names_[i];
  }
}

/*--------------------------------------------------------------------------*/
// methods implementation for RobotInputReceiver.
RobotInputReceiver::RobotInputReceiver(const RigidBodyTree<double>& tree) {
  num_actuators_ = tree.get_num_actuators();
  actuatorIndexMap_ = multibody::makeNameToActuatorsMap(tree);
  this->DeclareAbstractInputPort("lcmt_robot_input",
    drake::Value<dairlib::lcmt_robot_input>{});
  this->DeclareVectorOutputPort(TimestampedVector<double>(num_actuators_),
                                &RobotInputReceiver::CopyInputOut);
}

RobotInputReceiver::RobotInputReceiver(
      const drake::multibody::MultibodyPlant<double>& plant) {
  num_actuators_ = plant.num_actuators();
  actuatorIndexMap_ = multibody::makeNameToActuatorsMap(plant);
  this->DeclareAbstractInputPort("lcmt_robot_input",
    drake::Value<dairlib::lcmt_robot_input>{});
  this->DeclareVectorOutputPort(TimestampedVector<double>(num_actuators_),
                                &RobotInputReceiver::CopyInputOut);
}

void RobotInputReceiver::CopyInputOut(const Context<double>& context,
                                      TimestampedVector<double>* output) const {
  const drake::AbstractValue* input =
      this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto& input_msg = input->get_value<dairlib::lcmt_robot_input>();

  VectorXd input_vector = VectorXd::Zero(num_actuators_);

  for (int i = 0; i < input_msg.num_efforts; i++) {
    int j = actuatorIndexMap_.at(input_msg.effort_names[i]);
    input_vector(j) = input_msg.efforts[i];
  }
  output->SetDataVector(input_vector);
  output->set_timestamp(input_msg.utime * 1.0e-6);
}

/*--------------------------------------------------------------------------*/
// methods implementation for RobotCommandSender.

RobotCommandSender::RobotCommandSender(const RigidBodyTree<double>& tree) {
  num_actuators_ = tree.get_num_actuators();
  actuatorIndexMap_ = multibody::makeNameToActuatorsMap(tree);

for (int i = 0; i < num_actuators_; i++) {
    ordered_actuator_names_.push_back(tree.actuators[i].name_);
  }

  this->DeclareVectorInputPort(TimestampedVector<double>(num_actuators_));
  this->DeclareAbstractOutputPort(&RobotCommandSender::OutputCommand);
}

RobotCommandSender::RobotCommandSender(
    const drake::multibody::MultibodyPlant<double>& plant) {
  num_actuators_ = plant.num_actuators();
  actuatorIndexMap_ = multibody::makeNameToActuatorsMap(plant);

for (JointActuatorIndex i(0); i < plant.num_actuators(); ++i) {
    ordered_actuator_names_.push_back(
      plant.get_joint_actuator(i).name());
  }

  this->DeclareVectorInputPort(TimestampedVector<double>(num_actuators_));
  this->DeclareAbstractOutputPort(&RobotCommandSender::OutputCommand);
}

void RobotCommandSender::OutputCommand(const Context<double>& context,
    dairlib::lcmt_robot_input* input_msg) const {
  const TimestampedVector<double>* command = (TimestampedVector<double>*)
      this->EvalVectorInput(context, 0);


  input_msg->utime = command->get_timestamp() * 1e6;
  input_msg->num_efforts = num_actuators_;
  input_msg->effort_names.resize(num_actuators_);
  input_msg->efforts.resize(num_actuators_);
  for (int i = 0; i < num_actuators_; i++) {
    input_msg->effort_names[i] = ordered_actuator_names_[i];
    input_msg->efforts[i] = command->GetAtIndex(i);
  }
}

}  // namespace systems
}  // namespace dairlib
