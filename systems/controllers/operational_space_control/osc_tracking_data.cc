#pragma once
#include "systems/controllers/operational_space_control/osc_tracking_data.h"
#include <Eigen/Dense>
#include <string>
#include <vector>

using std::string;
using Eigen::VectorXd;
using Eigen::MatrixXd;

namespace dairlib {
namespace systems {
namespace controllers {


// OscTrackingData /////////////////////////////////////////////////////////////

// Updater
void OscTrackingData::Update(VectorXd x,
                             RigidBodyTree<double>* tree,
                             const drake::trajectories::Trajectory<double>* traj, double t,
                             int finite_state_machine_state,
                             double time_since_last_state_switch) {
  TrackOrNot(finite_state_machine_state, time_since_last_state_switch)
  if (!track_at_current_step_) return;

  // Update Feedback Output (Calling virtual methods)
  UpdateOutput(x, tree);
  UpdateJ(x, tree);
  UpdateJdotTimesV(x, tree);

  // Update Desired Output
  y_des_ = traj->value(t);
  dy_des_ = traj->MakeDerivative(1)->value(t);
  ddy_des_ = traj->MakeDerivative(2)->value(t);
}

// Getters
VectorXd OscTrackingData::GetDesiredOutputWithPdControl(
  VectorXd q) {
  return ddy_des_ + K_p * (y_des_ - y_) + K_d * (dy_des_ - J_ * q);
}
MatrixXd OscTrackingData::GetWeight(int finite_state_machine_state) {
  if (track_at_current_step_) {
    return W_;
  } else {
    return MatrixXd::Zero(n_r_, n_r_);
  }
}

// Set constant trajectory
void OscTrackingData::SetConstantTraj(VectorXd v) {
  fixed_position_ = v;
}

void OscTrackingData::TrackOrNot(int finite_state_machine_state,
                                 double time_since_last_state_switch) {
  it = find (state_to_do_tracking_.begin(), state_to_do_tracking_.end(),
             finite_state_machine_state);
  // Disable tracking by setting the weight to be zero when the current state
  // is not found in `state_to_do_tracking_`
  if (it != state_to_do_tracking_.end() &&
      time_since_last_state_switch >= period_of_no_control_) {
    track_at_current_step_ = true;
  } else {
    track_at_current_step_ = false;
  }
}

// Run this function in OSC constructor to make sure that users constructed
// OscTrackingData correctly.
bool OscTrackingData::CheckOscTrackingData() {
  DRAKE_DEMAND(!traj_is_const || traj_is_const && (fixed_position_.size() != 0));
}

// TaskSpaceTrackingData ///////////////////////////////////////////////////////
void TaskSpaceTrackingData::AddPointToTrack(int body_index,
    VectorXd pt_on_body,
    int state) {
  body_index_.push_back(body_index);
  pt_on_body_.push_back(pt_on_body);
  state_to_do_tracking_.push_back(state);
}
void TaskSpaceTrackingData::AddPointToTrack(std::vector<int> body_index,
    std::vector<VectorXd> pt_on_body,
    std::vector<int> state) {
  body_index_ = body_index;
  pt_on_body_ = pt_on_body;
  state_to_do_tracking_ = state;
}
// TransTaskSpaceTrackingData //////////////////////////////////////////
TransTaskSpaceTrackingData::TransTaskSpaceTrackingData(string traj_name,
    int n_r,
    Eigen::MatrixXd K_p,
    Eigen::MatrixXd K_d,
    Eigen::MatrixXd W,
    bool traj_is_const,
    bool traj_has_exp) :
  traj_name_(traj_name),
  n_r_(n_r),
  K_p_(K_p),
  K_d_(K_d),
  W_(W),
  traj_is_const_(traj_is_const),
  traj_has_exp_(traj_has_exp) {

}
void TransTaskSpaceTrackingData::UpdateOutput(const VectorXd& x,
    RigidBodyTree<double>* tree);
void TransTaskSpaceTrackingData::UpdateJ(const VectorXd& x,
    RigidBodyTree<double>* tree);
void TransTaskSpaceTrackingData::UpdateJdotTimesV(const VectorXd& x,
    RigidBodyTree<double>* tree);


// RotTaskSpaceTrackingData /////////////////////////////////////////////
RotTaskSpaceTrackingData::RotTaskSpaceTrackingData(string traj_name,
    int n_r,
    Eigen::MatrixXd K_p,
    Eigen::MatrixXd K_d,
    Eigen::MatrixXd W,
    bool traj_is_const,
    bool traj_has_exp) :
  traj_name_(traj_name),
  n_r_(n_r),
  K_p_(K_p),
  K_d_(K_d),
  W_(W),
  traj_is_const_(traj_is_const),
  traj_has_exp_(traj_has_exp)  {

}
void RotTaskSpaceTrackingData::UpdateOutput(const VectorXd& x,
    RigidBodyTree<double>* tree);
void RotTaskSpaceTrackingData::UpdateJ(const VectorXd& x,
                                       RigidBodyTree<double>* tree);
void RotTaskSpaceTrackingData::UpdateJdotTimesV(const VectorXd& x,
    RigidBodyTree<double>* tree);


// JointSpaceTrackingData //////////////////////////////////////////////////////
JointSpaceTrackingData::JointSpaceTrackingData(string traj_name,
    int n_r,
    Eigen::MatrixXd K_p,
    Eigen::MatrixXd K_d,
    Eigen::MatrixXd W,
    bool traj_is_const,
    bool traj_has_exp) :
  traj_name_(traj_name),
  n_r_(n_r),
  K_p_(K_p),
  K_d_(K_d),
  W_(W),
  traj_is_const_(traj_is_const),
  traj_has_exp_(traj_has_exp)  {
}

void JointSpaceTrackingData::UpdateOutput(const VectorXd& x,
    RigidBodyTree<double>* tree);
void JointSpaceTrackingData::UpdateJ(const VectorXd& x,
                                     RigidBodyTree<double>* tree);
void JointSpaceTrackingData::UpdateJdotTimesV(const VectorXd& x,
    RigidBodyTree<double>* tree);


void AddJointToTrack(int joint_position_index,
                     int joint_velocity_index,
                     int state) {
  joint_position_index_.push_back(joint_position_index);
  joint_velocity_index_.push_back(joint_velocity_index);
  state_to_do_tracking_.push_back(state);
}
void AddJointToTrack(std::vector<int> joint_position_index,
                     std::vector<VectorXd> joint_velocity_index,
                     std::vector<int> state) {
  joint_position_index_ = joint_position_index;
  joint_velocity_index_ = joint_velocity_index;
  state_to_do_tracking_ = state;
}

// AbstractTrackingData ////////////////////////////////////////////////////////
AbstractTrackingData::AbstractTrackingData(string traj_name,
    int n_r,
    Eigen::MatrixXd K_p,
    Eigen::MatrixXd K_d,
    Eigen::MatrixXd W,
    bool traj_is_const,
    bool traj_has_exp) :
  traj_name_(traj_name),
  n_r_(n_r),
  K_p_(K_p),
  K_d_(K_d),
  W_(W),
  traj_is_const_(traj_is_const),
  traj_has_exp_(traj_has_exp)  {

}
void AbstractTrackingData::UpdateOutput(const VectorXd& x,
                                        RigidBodyTree<double>* tree);
void AbstractTrackingData::UpdateJ(const VectorXd& x,
                                   RigidBodyTree<double>* tree);
void AbstractTrackingData::UpdateJdotTimesV(const VectorXd& x,
    RigidBodyTree<double>* tree);



}  // namespace controllers
}  // namespace systems
}  // namespace dairlib
