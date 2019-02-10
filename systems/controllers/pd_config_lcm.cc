#include "systems/controllers/pd_config_lcm.h"
#include "multibody/multibody_utils.h"

namespace dairlib {
namespace systems {

using std::string;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using drake::systems::AbstractValue;
using dairlib::systems::TimestampedVector;
using Eigen::VectorXd;
using drake::systems::Context;
using std::map;

// methods implementation for CassiePDConfigReceiver.
PDConfigReceiver::PDConfigReceiver(const RigidBodyTree<double>& tree) {
  actuatorIndexMap_ = multibody::makeNameToActuatorsMap(tree);

  num_positions_ = tree.get_num_positions();
  num_velocities_ = tree.get_num_velocities();
  num_actuators_ = tree.get_num_actuators();

  MatrixXd B = tree.B;
  // using all one's because zeros will cause quaternions to NaN
  // Don't really care about the answer here, just that it has the correct
  // sparsity pattern
  VectorXd q_zeros = VectorXd::Constant(tree.get_num_positions(), 1);
  KinematicsCache<double> cache = tree.doKinematics(q_zeros);
  MatrixXd vToqdot = tree.GetVelocityToQDotMapping(cache);
  // Use the tree to build a map from actuators to positions and velocities
  // Specific to systems where each actuator maps uniquely to a joint with
  // position and velocity
  //
  // Expect the jth column of RBT.B to be all zero except for one positive
  // element
  // If B(i,j) > 0, then the jth' actuator maps to the ith velocity coordinate
  for (int j = 0; j < B.cols(); j++) {
    int index = -1;
    for (int i = 0; i < B.rows(); i++) {
      DRAKE_THROW_UNLESS(B(i, j) >= 0);
      if (B(i, j) > 0) {
        DRAKE_THROW_UNLESS(index == -1);
        index = i;
      }
    }
    DRAKE_THROW_UNLESS(index != -1);
    actuatorToVelocityIndexMap_[j] = index;
    // Next use the GetVelocityToQDotMapping to map this velocity to a position
    // coordinate. The unit vector e_index should map to a single positive
    // element in qdot. This is not true for some joints (like quaternions),
    // so these joints will not be added to the map
    VectorXd e_index = VectorXd::Zero(tree.get_num_velocities());
    e_index(index) = 1;
    VectorXd qdot = vToqdot * e_index;
    int index_q = -1;
    for (int k = 0; k < qdot.size(); k++) {
      if (qdot(k) == 1) {
        DRAKE_THROW_UNLESS(index_q == -1);
        index_q = k;
      }
    }
    if (index_q != -1)
      actuatorToPositionIndexMap_[j] = index_q;
    std::cout << "Map u_ind:" << j << " q_ind: " << index_q << " v_ind: " <<
                  index << std::endl;
  }

  // Velocity map:
  this->DeclareAbstractInputPort(
      "lcmt_pd_config", drake::systems::Value<dairlib::lcmt_pd_config>{});
  this->DeclareVectorOutputPort(
      LinearConfig(tree.get_num_positions() + tree.get_num_velocities(),
                   tree.get_num_actuators()),
      &PDConfigReceiver::CopyConfig);
}

// methods implementation for CassiePDConfigReceiver.
PDConfigReceiver::PDConfigReceiver(const MultibodyPlant<double>& plant) {
  actuatorIndexMap_ = multibody::makeNameToActuatorsMap(plant);

  num_positions_ = plant.num_positions();
  num_velocities_ = plant.num_velocities();
  num_actuators_ = plant.num_actuators();


  MatrixXd B = plant.MakeActuationMatrix();
  // using all one's because zeros will cause quaternions to NaN
  // Don't really care about the answer here, just that it has the correct
  // sparsity pattern
  VectorXd q_zeros = VectorXd::Constant(plant.num_positions(), 1);

  // Use the plant to build a map from actuators to positions and velocities
  // Specific to systems where each actuator maps uniquely to a joint with
  // position and velocity
  //
  // Expect the jth column of RBT.B to be all zero except for one positive
  // element
  // If B(i,j) > 0, then the jth' actuator maps to the ith velocity coordinate
  for (int j = 0; j < B.cols(); j++) {
    int index = -1;
    for (int i = 0; i < B.rows(); i++) {
      DRAKE_THROW_UNLESS(B(i, j) >= 0);
      if (B(i, j) > 0) {
        DRAKE_THROW_UNLESS(index == -1);
        index = i;
      }
    }
    DRAKE_THROW_UNLESS(index != -1);
    actuatorToVelocityIndexMap_[j] = index;
    // Next use the velocity to qdot map to convert this velocity to a position
    // coordinate. The unit vector e_index should map to a single positive
    // element in qdot. This is not true for some joints (like quaternions),
    // so these joints will not be added to the map
    VectorXd e_index = VectorXd::Zero(plant.num_velocities());
    e_index(index) = 1;
    VectorXd qdot(plant.num_positions());
    auto context = plant.CreateDefaultContext();
    plant.MapVelocityToQDot(*context, e_index, &qdot);
    int index_q = -1;
    for (int k = 0; k < qdot.size(); k++) {
      if (qdot(k) == 1) {
        DRAKE_THROW_UNLESS(index_q == -1);
        index_q = k;
      }
    }
    if (index_q != -1)
      actuatorToPositionIndexMap_[j] = index_q;
    std::cout << "Map u_ind:" << j << " q_ind: " << index_q << " v_ind: " <<
                  index << std::endl;
  }

  // Velocity map:
  this->DeclareAbstractInputPort(
      "lcmt_pd_config", drake::systems::Value<dairlib::lcmt_pd_config>{});
  this->DeclareVectorOutputPort(
      LinearConfig(plant.num_positions() + plant.num_velocities(),
                   plant.num_actuators()),
      &PDConfigReceiver::CopyConfig);
}
/// read and parse a configuration LCM message
void PDConfigReceiver::CopyConfig(const Context<double>& context,
                                  LinearConfig* output) const {
  const AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_THROW_UNLESS(input != nullptr);
  const auto& config_msg = input->GetValue<dairlib::lcmt_pd_config>();


  MatrixXd K = MatrixXd::Zero(num_actuators_, num_positions_ + num_velocities_);
  VectorXd desired_state = VectorXd::Zero(num_positions_ + num_velocities_);

  for (int i = 0; i < config_msg.num_joints; i++) {
    int u_ind = actuatorIndexMap_.at(config_msg.joint_names[i]);
    // Need to generate position and velocity indices
    int q_ind = actuatorToPositionIndexMap_.at(u_ind);
    int v_ind = actuatorToVelocityIndexMap_.at(u_ind);

    K(u_ind, q_ind) = config_msg.kp[i];
    K(u_ind, num_positions_ + v_ind) = config_msg.kd[i];
    desired_state(q_ind) = config_msg.desired_position[i];
    desired_state(num_positions_ + v_ind) = config_msg.desired_velocity[i];
  }

  output->SetK(K);
  output->SetDesiredState(desired_state);
}

}  // namespace systems
}  // namespace dairlib
