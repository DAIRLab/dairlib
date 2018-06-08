#include "systems/controllers/pd_config_lcm.h"

namespace dairlib {
namespace systems {

using std::string;
using drake::systems::Context;
using drake::systems::AbstractValue;
using dairlib::systems::TimestampedVector;
using Eigen::VectorXd;

// methods implementation for CassiePDConfigReceiver.
PDConfigReceiver::PDConfigReceiver(RigidBodyTree<double>& tree) {
  tree_ = &tree;
  actuatorIndexMap_ = multibody::utils::makeNameToActuatorsMap(tree);
  positionIndexMap_ = multibody::utils::makeNameToPositionsMap(tree);
  velocityIndexMap_ = multibody::utils::makeNameToVelocitiesMap(tree);

  this->DeclareAbstractInputPort();
  this->DeclareVectorOutputPort(LinearConfig(tree.get_num_positions() +
                                             tree.get_num_velocities(),
                                             tree.get_num_actuators()),
                                &PDConfigReceiver::CopyConfig);
}

/// read and parse a configuration LCM message
void PDConfigReceiver::CopyConfig(
    const Context<double>& context, LinearConfig* output) const {
  const AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto& config_msg = input->GetValue<dairlib::lcmt_pd_config>();

  int nq = tree_->get_num_positions();
  int nv = tree_->get_num_velocities();
  int nu = tree_->get_num_actuators();

  MatrixXd K = MatrixXd::Zero(nu, nq + nv);
  VectorXd desired_state = VectorXd::Zero(nq + nv);

  for (int i = 0; i < config_msg.num_joints; i++) {
    int u_ind = actuatorIndexMap_.at(config_msg.joint_names[i]);
    int q_ind = positionIndexMap_.at(config_msg.joint_names[i]);
    int v_ind = velocityIndexMap_.at(config_msg.joint_names[i]);

    K(u_ind, q_ind) = config_msg.kp[i];
    K(u_ind, nq + v_ind) = config_msg.kd[i];
    desired_state(q_ind) = config_msg.desired_position[i];
    desired_state(nq + q_ind) = config_msg.desired_velocity[i];
  }
}

}
}