#include "joint_space_tracking_data.h"

#include "multibody/multibody_utils.h"

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using std::string;
using std::vector;

using drake::multibody::JacobianWrtVariable;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;

namespace dairlib::systems::controllers {

using multibody::MakeNameToPositionsMap;
using multibody::MakeNameToVelocitiesMap;

/**** JointSpaceTrackingData ****/
JointSpaceTrackingData::JointSpaceTrackingData(
    const string& name, const MatrixXd& K_p, const MatrixXd& K_d,
    const MatrixXd& W, const MultibodyPlant<double>& plant)
    : OptionsTrackingData(name, K_p.rows(), K_d.rows(), K_p, K_d, W, plant) {}

void JointSpaceTrackingData::AddJointToTrack(
    const std::string& joint_pos_name,
    const std::string& joint_vel_name) {
  AddStateAndJointToTrack(-1, joint_pos_name, joint_vel_name);
}

void JointSpaceTrackingData::AddStateAndJointToTrack(
    int fsm_state, const std::string& joint_pos_name,
    const std::string& joint_vel_name) {
  AddFiniteStateToTrack(fsm_state);
  joint_pos_idx_[fsm_state] =
      {
          MakeNameToPositionsMap(plant_).at(joint_pos_name)};
  joint_vel_idx_[fsm_state] =
      {
      MakeNameToVelocitiesMap(plant_).at(joint_vel_name)};

}

void JointSpaceTrackingData::AddJointsToTrack(
    const std::vector<std::string>& joint_pos_names,
    const std::vector<std::string>& joint_vel_names) {
  AddStateAndJointsToTrack(-1, joint_pos_names, joint_vel_names);
}

void JointSpaceTrackingData::AddStateAndJointsToTrack(
    int fsm_state, const std::vector<std::string>& joint_pos_names,
    const std::vector<std::string>& joint_vel_names) {
  AddFiniteStateToTrack(fsm_state);
  std::vector<int> ordered_index_set;
  for (const auto& mem : joint_pos_names) {
    ordered_index_set.push_back(MakeNameToPositionsMap(plant_).at(mem));
  }
  joint_pos_idx_[fsm_state] = ordered_index_set;
  ordered_index_set.clear();
  for (const auto& mem : joint_vel_names) {
    ordered_index_set.push_back(MakeNameToVelocitiesMap(plant_).at(mem));
  }
  joint_vel_idx_[fsm_state] = ordered_index_set;
}

void JointSpaceTrackingData::UpdateY(const VectorXd& x,
                                     const Context<double>& context) {
  VectorXd y(GetYDim());
  for (int i = 0; i < GetYDim(); i++) {
    y(i) = x(joint_pos_idx_.at(fsm_state_).at(i));
  }
  y_ = y;
}

void JointSpaceTrackingData::UpdateYdot(const VectorXd& x,
                                        const Context<double>& context) {
  VectorXd ydot(GetYdotDim());
  for (int i = 0; i < GetYdotDim(); i++) {
    ydot(i) = x(plant_.num_positions() +
                      joint_vel_idx_.at(fsm_state_).at(i));
  }
  ydot_ = ydot;
}

void JointSpaceTrackingData::UpdateJ(const VectorXd& x,
                                     const Context<double>& context) {
  MatrixXd J = MatrixXd::Zero(GetYdotDim(), plant_.num_velocities());
  for (int i = 0; i < GetYdotDim(); i++) {
    J(i, joint_vel_idx_.at(fsm_state_).at(i)) = 1;
  }
  J_ = J;
}

void JointSpaceTrackingData::UpdateJdotV(
    const VectorXd& x, const Context<double>& context) {
  JdotV_ = VectorXd::Zero(GetYdotDim());
}

void JointSpaceTrackingData::CheckDerivedOscTrackingData() {
  for (auto fsm_joint_pair : joint_pos_idx_) {
    DRAKE_DEMAND(joint_pos_idx_.at(fsm_joint_pair.first).size() == GetYDim());
    DRAKE_DEMAND(joint_vel_idx_.at(fsm_joint_pair.first).size() == GetYdotDim());
  }
  if (active_fsm_states_.empty()) {
    DRAKE_DEMAND(joint_pos_idx_.size() == 1);
    DRAKE_DEMAND(joint_vel_idx_.size() == 1);
  } else {
    DRAKE_DEMAND(joint_pos_idx_.size() == active_fsm_states_.size());
    DRAKE_DEMAND(joint_vel_idx_.size() == active_fsm_states_.size());
  }
}

}  // namespace dairlib::systems::controllers