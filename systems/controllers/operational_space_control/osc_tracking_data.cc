#pragma once
#include "systems/controllers/operational_space_control/osc_tracking_data.h"
#include <Eigen/Dense>
#include <string>
#include <vector>

namespace dairlib {
namespace systems {
namespace controllers {


// OscTrackingData /////////////////////////////////////////////////////////////
OscTrackingData::OscTrackingData(std::string traj_name, int n_r,
                                 bool traj_is_const, bool traj_has_exp):
  traj_name_(traj_name),
  n_r_(n_r),
  K_p_(MatrixXd::Zero(n_r, n_r)),
  K_d_(MatrixXd::Zero(n_r, n_r)),
  W_(MatrixXd::Zero(n_r, n_r)),
  traj_is_const_(traj_is_const),
  traj_has_exp_(traj_has_exp) {
}

void OscTrackingData::UpdateFeedbackOutput(Eigen::VectorXd x,
    RigidBodyTree<double>* tree) {
  // Calling virtual methods
  UpdateOutput(x, tree);
  UpdateJ(x, tree);
  UpdateJdotTimesV(x, tree);
}
void OscTrackingData::UpdateDesiredOutput(
  const drake::trajectories::Trajectory<double>* traj, double t) {
  y_des_ = traj->value(t);
  dy_des_ = traj->MakeDerivative(1)->value(t);
  ddy_des_ = traj->MakeDerivative(2)->value(t);
}
Eigen::VectorXd OscTrackingData::GetDesiredOutputWithPdControl(
  Eigen::VectorXd q) {
  return ddy_des_ + K_p * (y_des_ - y_) + K_d * (dy_des_ - J_ * q);
}

// Getters
Eigen::MatrixXd OscTrackingData::GetWeight(int finite_state_machine_state) {
  it = find (state_to_do_tracking_.begin(), state_to_do_tracking_.end(),
             finite_state_machine_state);
  // Disable tracking by setting the weight to be zero when the current state
  // is not found in `state_to_do_tracking_`
  if (it != state_to_do_tracking_.end()) {
    return W_;
  } else {
    return MatrixXd::Zero(n_r_, n_r_);
  }
}

// Setters

// Set constant trajectory
void OscTrackingData::SetConstantTraj(Eigen::VectorXd v) {
  fixed_position_ = v;
}

// Run this function in OSC constructor to make sure that users constructed
// OscTrackingData correctly.
bool OscTrackingData::CheckOscTrackingData() {
  DRAKE_DEMAND(!traj_is_const || traj_is_const && (fixed_position_.size() != 0));

}




// TranslationalTaskSpaceTrackingData //////////////////////////////////////////
void TranslationalTaskSpaceTrackingData::UpdateOutput(
  const Eigen::VectorXd& x, RigidBodyTree<double>* tree);
void TranslationalTaskSpaceTrackingData::UpdateJ(
  Eigen::VectorXd x);
void TranslationalTaskSpaceTrackingData::UpdateJdotTimesV(
  Eigen::VectorXd x);


// RotationalTaskSpaceTrackingData /////////////////////////////////////////////
void RotationalTaskSpaceTrackingData::UpdateOutput(
  const Eigen::VectorXd& x,
  RigidBodyTree<double>* tree);
void RotationalTaskSpaceTrackingData::UpdateJ(
  Eigen::VectorXd x);
void RotationalTaskSpaceTrackingData::UpdateJdotTimesV(
  Eigen::VectorXd x);


// JointSpaceTrackingData //////////////////////////////////////////////////////
void JointSpaceTrackingData::UpdateOutput(const Eigen::VectorXd& x,
    RigidBodyTree<double>* tree);
void JointSpaceTrackingData::UpdateJ(Eigen::VectorXd x);
void JointSpaceTrackingData::UpdateJdotTimesV(Eigen::VectorXd x);


void AbstractTrackingData::UpdateOutput(const Eigen::VectorXd& x,
                                        RigidBodyTree<double>* tree);
void AbstractTrackingData::UpdateJ(Eigen::VectorXd x);
void AbstractTrackingData::UpdateJdotTimesV(Eigen::VectorXd x);



}  // namespace controllers
}  // namespace systems
}  // namespace dairlib
