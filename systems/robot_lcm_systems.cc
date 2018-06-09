#include "robot_lcm_systems.h"

namespace dairlib{
namespace systems{

using std::string;
using drake::systems::Context;
using systems::OutputVector;
using Eigen::VectorXd;
using drake::systems::LeafSystem;

/*--------------------------------------------------------------------------*/
// methods implementation for RobotOutputReceiver.
RobotOutputReceiver::RobotOutputReceiver(const RigidBodyTree<double>& tree) {
  tree_ = &tree;
  positionIndexMap_ = multibody::utils::makeNameToPositionsMap(tree);
  velocityIndexMap_ = multibody::utils::makeNameToVelocitiesMap(tree);
  this->DeclareAbstractInputPort();
  this->DeclareVectorOutputPort(OutputVector<double>(
    tree.get_num_positions(), tree.get_num_velocities(),
    tree.get_num_actuators()),
    &RobotOutputReceiver::CopyOutput);
}

void RobotOutputReceiver::CopyOutput(
    const Context<double>& context, OutputVector<double>* output) const {
  const drake::systems::AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto& state_msg = input->GetValue<dairlib::lcmt_robot_output>();

  VectorXd positions = VectorXd::Zero(tree_->get_num_positions());
  for (int i = 0; i < state_msg.num_positions; i++) {
    int j = positionIndexMap_.at(state_msg.position_names[i]);
    positions(j) = state_msg.position[i];
  }
  VectorXd velocities = VectorXd::Zero(tree_->get_num_velocities());
  for (int i = 0; i < state_msg.num_velocities; i++) {
    int j = velocityIndexMap_.at(state_msg.velocity_names[i]);
    velocities(j) = state_msg.velocity[i];
  }
  output->SetPositions(positions);
  output->SetVelocities(velocities);
  output->set_timestamp(state_msg.timestamp);
}


/*--------------------------------------------------------------------------*/
// methods implementation for RobotOutputSender.

RobotOutputSender::RobotOutputSender(const RigidBodyTree<double>& tree) {
  tree_ = &tree;
  positionIndexMap_ = multibody::utils::makeNameToPositionsMap(tree);
  velocityIndexMap_ = multibody::utils::makeNameToVelocitiesMap(tree);

  state_input_port_ = this->DeclareVectorInputPort(BasicVector<double>(
      tree.get_num_positions() + tree.get_num_velocities())).get_index();
  this->DeclareAbstractOutputPort(&RobotOutputSender::Output);
}

/// Populate a state message with all states
void RobotOutputSender::Output(const Context<double>& context,
                                         dairlib::lcmt_robot_output* state_msg) const {
  const auto state = this->EvalVectorInput(context, state_input_port_);

  //using the time from the context
  state_msg->timestamp = context.get_time()*1e6;

  int nq = tree_->get_num_positions();
  int nv = tree_->get_num_positions();

  state_msg->num_positions = nq;
  state_msg->num_velocities = nv;
  state_msg->position_names.resize(nq);
  state_msg->velocity_names.resize(nv);
  state_msg->position.resize(nq);
  state_msg->velocity.resize(nv);

  for (int i = 0; i < nq; i++) {
    state_msg->position[i] = state->GetAtIndex(i);
    state_msg->position_names[i] = tree_->get_position_name(i);
  }
  for (int i = 0; i < nv; i++) {
    state_msg->velocity[i] = state->GetAtIndex(nq + i);
    state_msg->velocity_names[i] = tree_->get_velocity_name(i);
  }
}

/*--------------------------------------------------------------------------*/
// methods implementation for RobotInputReceiver.
RobotInputReceiver::RobotInputReceiver(const RigidBodyTree<double>& tree) {
  tree_ = &tree;
  actuatorIndexMap_ = multibody::utils::makeNameToActuatorsMap(tree);
  this->DeclareAbstractInputPort();
  this->DeclareVectorOutputPort(TimestampedVector<double>(
      tree.get_num_actuators()), &RobotInputReceiver::CopyInputOut);
}

void RobotInputReceiver::CopyInputOut(
    const Context<double>& context, TimestampedVector<double>* output) const {
  const drake::systems::AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto& input_msg = input->GetValue<dairlib::lcmt_robot_input>();

  VectorXd input_vector = VectorXd::Zero(tree_->get_num_actuators());

  for (int i = 0; i < input_msg.num_efforts; i++) {
    int j = actuatorIndexMap_.at(input_msg.effort_names[i]);
    input_vector(j) = input_msg.efforts[i];
  }
  output->SetDataVector(input_vector);
  output->set_timestamp(input_msg.timestamp);
}


/*--------------------------------------------------------------------------*/
// methods implementation for RobotCommandSender.

RobotCommandSender::RobotCommandSender(const RigidBodyTree<double>& tree) {
  tree_ = &tree;
  actuatorIndexMap_ = multibody::utils::makeNameToActuatorsMap(tree);

  this->DeclareVectorInputPort(TimestampedVector<double>(tree.get_num_actuators()));
  this->DeclareAbstractOutputPort(&RobotCommandSender::OutputCommand);
}

void RobotCommandSender::OutputCommand(const Context<double>& context,
                                         dairlib::lcmt_robot_input* input_msg) const {
  const TimestampedVector<double>* command = (TimestampedVector<double>*)
      this->EvalVectorInput(context, 0);

  int nu = tree_->get_num_actuators();

  input_msg->timestamp = command->get_timestamp(); //context.get_time()*1e6;
  input_msg->num_efforts = nu;
  input_msg->effort_names.resize(nu);
  input_msg->efforts.resize(nu);
  for (int i = 0; i < nu; i++) {
    input_msg->effort_names[i] = tree_->actuators[i].name_;
    input_msg->efforts[i] = command->GetAtIndex(i);
  }
}


}
}