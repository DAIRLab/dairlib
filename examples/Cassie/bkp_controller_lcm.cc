
namespace drake {
const int CASSIE_NUM_COMMANDS = cassieJointNames.size();
const int CASSIE_NUM_JOINTS = cassieJointNames.size();

using std::string;
using drake::systems::Context;
using dairlib::systems::TimestampedVector;
using Eigen::VectorXd;

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


// methods implementation for CassiePDConfigReceiver.
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

CassiePDConfig::CassiePDConfig(int num_joints) : TimestampedVector<double>(4*num_joints) {
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