#include "datatypes/cassie_names.h"
#include "cassie_controller_lcm.h"

namespace drake{

using std::string;
using systems::Context;
using systems::BasicVector;
using Eigen::VectorXd;

const int CASSIE_NUM_COMMANDS = cassieJointNames.size();
const int CASSIE_NUM_JOINTS = cassieJointNames.size();

/// Given a string name, finds the index related to that joint
/// looks into cassieJointNames list
/// This is a slow implementation and should be fixed. It should also probably
/// be move to the cassie_names header itself
int getJointIndex(string joint) {
  //should probably use std::find
  for (int i = 0; i < CASSIE_NUM_JOINTS; i++) {
    if (cassieJointNames[i].compare(joint) == 0) {
      return i;
    }
  }
  return -1;
}

/*--------------------------------------------------------------------------*/
// methods implementation for CassieStateReceiver.
CassieStateReceiver::CassieStateReceiver() {
  this->DeclareAbstractInputPort();
  this->DeclareVectorOutputPort(BasicVector<double>(CASSIE_NUM_JOINTS*2),
   &CassieStateReceiver::CopyStateOut);
}

void CassieStateReceiver::CopyStateOut(
    const Context<double>& context, BasicVector<double>* output) const {
  const systems::AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto& state_msg = input->GetValue<dairlib::lcmt_cassie_state>();

  VectorXd state = VectorXd::Zero(2*CASSIE_NUM_JOINTS);
  for (int i = 0; i < state_msg.num_joints; i++) {
    int j = getJointIndex(state_msg.joint_names[i]);
    state(j) = state_msg.position[i];
    state(j + CASSIE_NUM_JOINTS) = state_msg.velocity[i];
  }
  output->SetFromVector(state);
}

/*--------------------------------------------------------------------------*/
// methods implementation for CassieCommandSender.

CassieCommandSender::CassieCommandSender() {
  this->DeclareInputPort(systems::kVectorValued, CASSIE_NUM_COMMANDS); //number of inputs
  this->DeclareAbstractOutputPort(&CassieCommandSender::OutputCommand);
}

void CassieCommandSender::OutputCommand(const Context<double>& context,
                                         dairlib::lcmt_cassie_input* input_msg) const {
  const systems::BasicVector<double>* command =
      this->EvalVectorInput(context, 0);

  input_msg->timestamp = context.get_time()*1e6;
  input_msg->num_joints = CASSIE_NUM_JOINTS;
  input_msg->joint_names.resize(CASSIE_NUM_JOINTS);
  input_msg->inputs.resize(CASSIE_NUM_JOINTS);
  for (int i = 0; i < CASSIE_NUM_JOINTS; i++) {
    input_msg->joint_names[i] = cassieJointNames[i];
    input_msg->inputs[i] = command->GetAtIndex(i);
  }
}

/*--------------------------------------------------------------------------*/
// methods implementation for CassieStateSender.

CassieStateSender::CassieStateSender() {
  this->DeclareInputPort(systems::kVectorValued, 2*(CASSIE_NUM_JOINTS));
  this->DeclareAbstractOutputPort(&CassieStateSender::OutputState);
}

/// Populate a state message with all states
/// Assumes: number of positions == number of velocities
///          state is a vector ordered [positions; velocities]
void CassieStateSender::OutputState(const Context<double>& context,
                                         dairlib::lcmt_cassie_state* state_msg) const {
  const systems::BasicVector<double>* state =
      this->EvalVectorInput(context, 0);

  state_msg->timestamp = context.get_time()*1e6;
  state_msg->num_joints = CASSIE_NUM_JOINTS;
  state_msg->joint_names.resize(CASSIE_NUM_JOINTS);
  state_msg->position.resize(CASSIE_NUM_JOINTS);
  state_msg->velocity.resize(CASSIE_NUM_JOINTS);
  for (int i = 0; i < CASSIE_NUM_JOINTS; i++) {
    state_msg->joint_names[i] = cassieJointNames[i];
    state_msg->position[i] = state->GetAtIndex(i);
    state_msg->velocity[i] = state->GetAtIndex(i + CASSIE_NUM_JOINTS);
  }
}


// methods implementation for CassieStateReceiver.
CassiePDConfigReceiver::CassiePDConfigReceiver() {
  this->DeclareAbstractInputPort();
  this->DeclareVectorOutputPort(CassiePDConfig(CASSIE_NUM_JOINTS),
                                &CassiePDConfigReceiver::CopyConfig);
}

/// read and parse a configuration LCM message
void CassiePDConfigReceiver::CopyConfig(
    const Context<double>& context, CassiePDConfig* output) const {
  const systems::AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto& config_msg = input->GetValue<dairlib::lcmt_cassie_pd_config>();

  for (int i = 0; i < config_msg.num_joints; i++) {
    int j = getJointIndex(config_msg.joint_names[i]);
    output->setDesiredPosition(j, config_msg.desired_position[i]);
    output->setDesiredVelocity(j, config_msg.desired_velocity[i]);
    output->setKp(j, config_msg.kp[i]);
    output->setKd(j, config_msg.kd[i]);
  }
}

CassiePDConfig::CassiePDConfig(int num_joints) : BasicVector<double>(4*num_joints) {
  num_joints_ = num_joints;
  desired_position_ = VectorXd::Zero(num_joints);
  desired_velocity_ = VectorXd::Zero(num_joints);
  kp_ = VectorXd::Zero(num_joints);
  kd_ = VectorXd::Zero(num_joints);
}

void CassiePDConfig::setDesiredPosition(VectorXd desired_position) {
  desired_position_ = desired_position;
}

void CassiePDConfig::setDesiredVelocity(VectorXd desired_velocity) {
  desired_velocity_ = desired_velocity;
}

void CassiePDConfig::setKp(VectorXd kp) {
  kp_ = kp;
}

void CassiePDConfig::setKd(VectorXd kd) {
  kd_ = kd;
}

void CassiePDConfig::setDesiredPosition(int index, double position) {
  desired_position_(index) = position;
}
void CassiePDConfig::setDesiredVelocity(int index, double velocity) {
  desired_velocity_(index) = velocity;
}
void CassiePDConfig::setKp(int index, double kp) {
  kp_(index) = kp;
}
void CassiePDConfig::setKd(int index, double kd) {
  kd_(index) = kd;
}

/// Clone a configuration
/// This function must be implmemented for Drake
CassiePDConfig* CassiePDConfig::DoClone() const {
  auto config = new CassiePDConfig(num_joints_);

  config-> setDesiredPosition(getDesiredPosition());
  config-> setDesiredVelocity(getDesiredVelocity());
  config-> setKp(getKp());
  config-> setKd(getKd());

  return config;
}


}