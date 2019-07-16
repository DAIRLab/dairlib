#include "systems/controllers/operational_space_control/osc_tracking_data.h"
#include <Eigen/Dense>
#include <string>
#include <vector>

using std::vector;
using std::string;
using Eigen::VectorXd;
using Eigen::MatrixXd;

namespace dairlib {
namespace systems {
namespace controllers {


// OscTrackingData /////////////////////////////////////////////////////////////
// In each loop, run Update() before calling any getters.
OscTrackingData::OscTrackingData(string name,
                                 int n_r,
                                 Eigen::MatrixXd K_p,
                                 Eigen::MatrixXd K_d,
                                 Eigen::MatrixXd W,
                                 bool traj_is_const,
                                 bool traj_has_exp) :
  name_(name),
  n_r_(n_r),
  K_p_(K_p),
  K_d_(K_d),
  W_(W),
  traj_is_const_(traj_is_const),
  traj_has_exp_(traj_has_exp) {
}

// Updater
bool OscTrackingData::Update(VectorXd x,
                             const KinematicsCache<double>& cache,
                             RigidBodyTree<double>* tree,
                             const drake::trajectories::Trajectory<double>& traj,
                             double t,
                             int finite_state_machine_state,
                             double time_since_last_state_switch) {
  TrackOrNot(finite_state_machine_state, time_since_last_state_switch);
  if (!track_at_current_step_) return track_at_current_step_;

  // Update Feedback Output (Calling virtual methods)
  UpdateOutput(x, cache, tree);
  UpdateJ(x, cache, tree);
  UpdateJdotV(x, cache, tree);

  // Update Desired Output
  y_des_ = traj.value(t);
  dy_des_ = traj.MakeDerivative(1)->value(t);
  ddy_des_ = traj.MakeDerivative(2)->value(t);

  return track_at_current_step_;
}

// Getters
VectorXd OscTrackingData::GetDesiredOutputWithPdControl(
  VectorXd v) {
  return ddy_des_ + K_p_ * (y_des_ - y_) + K_d_ * (dy_des_ - J_ * v);
}
MatrixXd OscTrackingData::GetWeight() {
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
  vector<int>::iterator it = find (state_to_do_tracking_.begin(),
                                   state_to_do_tracking_.end(),
                                   finite_state_machine_state);
  // TODO: test the following line
  state_to_do_tracking_idx_ = std::distance(state_to_do_tracking_.begin(), it);
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
void OscTrackingData::CheckOscTrackingData() {
  DRAKE_DEMAND(!traj_is_const_ ||
               (traj_is_const_ && (fixed_position_.size() != 0)));
}

// TaskSpaceTrackingData ///////////////////////////////////////////////////////
TaskSpaceTrackingData::TaskSpaceTrackingData(string name,
    int n_r,
    Eigen::MatrixXd K_p,
    Eigen::MatrixXd K_d,
    Eigen::MatrixXd W,
    bool traj_is_const,
    bool traj_has_exp) : OscTrackingData(name, n_r,
          K_p, K_d, W, traj_is_const, traj_has_exp) {
}

void TaskSpaceTrackingData::AddPointToTrack(int body_index,
    VectorXd pt_on_body,
    int state) {
  body_index_.push_back(body_index);
  pt_on_body_.push_back(pt_on_body);
  state_to_do_tracking_.push_back(state);
}
void TaskSpaceTrackingData::AddPointToTrack(vector<int> body_index,
    vector<VectorXd> pt_on_body,
    vector<int> state) {
  body_index_ = body_index;
  pt_on_body_ = pt_on_body;
  state_to_do_tracking_ = state;
}
// TransTaskSpaceTrackingData //////////////////////////////////////////
TransTaskSpaceTrackingData::TransTaskSpaceTrackingData(string name,
    int n_r,
    Eigen::MatrixXd K_p,
    Eigen::MatrixXd K_d,
    Eigen::MatrixXd W,
    bool traj_is_const,
    bool traj_has_exp,
    bool track_center_of_mass) : TaskSpaceTrackingData(name, n_r,
          K_p, K_d, W, traj_is_const, traj_has_exp),
  track_center_of_mass_(track_center_of_mass) {
}

void TransTaskSpaceTrackingData::UpdateOutput(const VectorXd& x,
    const KinematicsCache<double>& cache, RigidBodyTree<double>* tree) {
  if (track_center_of_mass_) {
    y_ = tree->centerOfMass(cache);
  } else {
    y_ = tree->CalcBodyPoseInWorldFrame(
           cache, tree->get_body(body_index_.at(state_to_do_tracking_idx_))).translation();
  }
}
void TransTaskSpaceTrackingData::UpdateJ(const VectorXd& x,
    const KinematicsCache<double>& cache, RigidBodyTree<double>* tree) {
  if (track_center_of_mass_) {
    J_ = tree->centerOfMassJacobian(cache);
  } else {
    J_ = tree_->transformPointsJacobian(cache,
                                        pt_on_body_.at(state_to_do_tracking_idx_),
                                        body_index_.at(state_to_do_tracking_idx_), 0, false);
  }
}
void TransTaskSpaceTrackingData::UpdateJdotV(const VectorXd& x,
    const KinematicsCache<double>& cache, RigidBodyTree<double>* tree) {
  if (track_center_of_mass_) {
    JdotV_ = tree->centerOfMassJacobianDotTimesV(cache);
  } else {
    JdotV_ = tree_->transformPointsJacobianDotTimesV(cache,
             pt_on_body_.at(state_to_do_tracking_idx_),
             body_index_.at(state_to_do_tracking_idx_), 0);
  }
}


// RotTaskSpaceTrackingData /////////////////////////////////////////////
RotTaskSpaceTrackingData::RotTaskSpaceTrackingData(string name,
    int n_r,
    Eigen::MatrixXd K_p,
    Eigen::MatrixXd K_d,
    Eigen::MatrixXd W,
    bool traj_is_const,
    bool traj_has_exp) : TaskSpaceTrackingData(name, n_r,
          K_p, K_d, W, traj_is_const, traj_has_exp) {
}

void RotTaskSpaceTrackingData::UpdateOutput(const VectorXd& x,
    const KinematicsCache<double>& cache, RigidBodyTree<double>* tree) {

}
void RotTaskSpaceTrackingData::UpdateJ(const VectorXd& x,
                                       const KinematicsCache<double>& cache,
                                       RigidBodyTree<double>* tree) {

}
void RotTaskSpaceTrackingData::UpdateJdotV(const VectorXd& x,
    const KinematicsCache<double>& cache, RigidBodyTree<double>* tree) {

}


// JointSpaceTrackingData //////////////////////////////////////////////////////
JointSpaceTrackingData::JointSpaceTrackingData(string name,
    int n_r,
    Eigen::MatrixXd K_p,
    Eigen::MatrixXd K_d,
    Eigen::MatrixXd W,
    bool traj_is_const,
    bool traj_has_exp) : OscTrackingData(name, n_r,
          K_p, K_d, W, traj_is_const, traj_has_exp) {
}

void JointSpaceTrackingData::UpdateOutput(const VectorXd& x,
    const KinematicsCache<double>& cache, RigidBodyTree<double>* tree) {

}
void JointSpaceTrackingData::UpdateJ(const VectorXd& x,
                                     const KinematicsCache<double>& cache,
                                     RigidBodyTree<double>* tree) {

}
void JointSpaceTrackingData::UpdateJdotV(const VectorXd& x,
    const KinematicsCache<double>& cache, RigidBodyTree<double>* tree) {

}


void JointSpaceTrackingData::AddJointToTrack(int joint_position_index,
    int joint_velocity_index,
    int state) {
  joint_position_index_.push_back(joint_position_index);
  joint_velocity_index_.push_back(joint_velocity_index);
  state_to_do_tracking_.push_back(state);
}
void JointSpaceTrackingData::AddJointToTrack(vector<int> joint_position_index,
    vector<int> joint_velocity_index,
    vector<int> state) {
  joint_position_index_ = joint_position_index;
  joint_velocity_index_ = joint_velocity_index;
  state_to_do_tracking_ = state;
}

// AbstractTrackingData ////////////////////////////////////////////////////////
AbstractTrackingData::AbstractTrackingData(string name,
    int n_r,
    Eigen::MatrixXd K_p,
    Eigen::MatrixXd K_d,
    Eigen::MatrixXd W,
    bool traj_is_const,
    bool traj_has_exp) : OscTrackingData(name, n_r,
          K_p, K_d, W, traj_is_const, traj_has_exp) {
}

void AbstractTrackingData::UpdateOutput(const VectorXd& x,
                                        const KinematicsCache<double>& cache,
                                        RigidBodyTree<double>* tree) {

}
void AbstractTrackingData::UpdateJ(const VectorXd& x,
                                   const KinematicsCache<double>& cache,
                                   RigidBodyTree<double>* tree) {

}
void AbstractTrackingData::UpdateJdotV(const VectorXd& x,
                                       const KinematicsCache<double>& cache,
                                       RigidBodyTree<double>* tree) {

}



}  // namespace controllers
}  // namespace systems
}  // namespace dairlib
