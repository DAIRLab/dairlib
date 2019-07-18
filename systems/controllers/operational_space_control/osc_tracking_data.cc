#include "systems/controllers/operational_space_control/osc_tracking_data.h"

#include <Eigen/Dense>
#include <string>
#include <vector>

#include "common/math_utils.h"

using std::cout;
using std::endl;

using std::vector;
using std::string;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Quaterniond;

namespace dairlib {
namespace systems {
namespace controllers {


// In each loop, run Update() before calling any getters.
// OscTrackingData /////////////////////////////////////////////////////////////
OscTrackingData::OscTrackingData(string name,
                                 int n_r,
                                 MatrixXd K_p,
                                 MatrixXd K_d,
                                 MatrixXd W,
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
bool OscTrackingData::Update(VectorXd x_w_spr,
                             KinematicsCache<double>& cache_w_spr,
                             RigidBodyTree<double>* tree_w_spr,
                             VectorXd x_wo_spr,
                             KinematicsCache<double>& cache_wo_spr,
                             RigidBodyTree<double>* tree_wo_spr,
                             const drake::trajectories::Trajectory<double>& traj,
                             double t,
                             int finite_state_machine_state,
                             double time_since_last_state_switch) {
  // Update track_at_current_step_
  if (state_.empty()) {
    track_at_current_step_ = true;
  } else {
    TrackOrNot(finite_state_machine_state, time_since_last_state_switch);
  }

  // Proceed based on the result of track_at_current_step_
  if (!track_at_current_step_) {
    return track_at_current_step_;
  } else {
    // Careful: must update y_des_ before calling UpdateYAndError()

    // Update Desired Output
    y_des_ = traj.value(t);
    dy_des_ = traj.MakeDerivative(1)->value(t);
    ddy_des_ = traj.MakeDerivative(2)->value(t);

    // Update Feedback Output (Calling virtual methods)
    UpdateYAndError(x_w_spr, cache_w_spr, tree_w_spr);
    UpdateYdot(x_w_spr, cache_w_spr, tree_w_spr);
    UpdateJ(x_wo_spr, cache_wo_spr, tree_wo_spr);
    UpdateJdotV(x_wo_spr, cache_wo_spr, tree_wo_spr);

    return track_at_current_step_;
  }
}

// Getters
VectorXd OscTrackingData::GetDesiredOutputWithPdControl() {
  return ddy_des_ + K_p_ * (error_y_) + K_d_ * (dy_des_ - ydot_);
}
MatrixXd OscTrackingData::GetWeight() {
  if (track_at_current_step_) {
    return W_;
  } else {
    return MatrixXd::Zero(n_r_, n_r_);
  }
}

void OscTrackingData::TrackOrNot(int finite_state_machine_state,
                                 double time_since_last_state_switch) {
  if (state_.empty()) {
    if (time_since_last_state_switch >= period_of_no_control_) {
      track_at_current_step_ = true;
    } else {
      track_at_current_step_ = false;
    }
  }

  vector<int>::iterator it = find(state_.begin(),
                                  state_.end(),
                                  finite_state_machine_state);
  // TODO: test the following line
  state_idx_ = std::distance(state_.begin(), it);
  // Disable tracking by setting the weight to be zero when the current state
  // is not found in `state_`
  if (it != state_.end() &&
      time_since_last_state_switch >= period_of_no_control_) {
    track_at_current_step_ = true;
  } else {
    track_at_current_step_ = false;
  }
}

// Run this function in OSC constructor to make sure that users constructed
// OscTrackingData correctly.
void OscTrackingData::CheckOscTrackingData() {
  CheckDerivedOscTrackingData();

  DRAKE_DEMAND(!traj_is_const_ ||
               (traj_is_const_ && (fixed_position_.size() != 0)));
  DRAKE_DEMAND((K_p_.rows() == n_r_) && (K_p_.cols() == n_r_));
  DRAKE_DEMAND((K_d_.rows() == n_r_) && (K_d_.cols() == n_r_));
  DRAKE_DEMAND((W_.rows() == n_r_) && (W_.cols() == n_r_));
}

// TaskSpaceTrackingData ///////////////////////////////////////////////////////
TaskSpaceTrackingData::TaskSpaceTrackingData(string name,
    int n_r,
    MatrixXd K_p,
    MatrixXd K_d,
    MatrixXd W,
    bool traj_is_const,
    bool traj_has_exp) : OscTrackingData(name, n_r,
          K_p, K_d, W, traj_is_const, traj_has_exp) {
}

void TaskSpaceTrackingData::AddPointToTrack(int body_index_wo_spr,
    VectorXd pt_on_body) {
  body_index_wo_spr_.push_back(body_index_wo_spr);
  pt_on_body_.push_back(pt_on_body);
}
void TaskSpaceTrackingData::AddPointToTrack(int body_index_wo_spr,
    VectorXd pt_on_body, int state) {
  state_.push_back(state);
  AddPointToTrack(body_index_wo_spr, pt_on_body);
}
void TaskSpaceTrackingData::AddPointToTrack(int body_index_w_spr,
    int body_index_wo_spr,
    VectorXd pt_on_body) {
  body_index_w_spr_.push_back(body_index_w_spr);
  AddPointToTrack(body_index_wo_spr, pt_on_body);
}
void TaskSpaceTrackingData::AddPointToTrack(int body_index_w_spr,
    int body_index_wo_spr,
    VectorXd pt_on_body, int state) {
  state_.push_back(state);
  AddPointToTrack(body_index_w_spr, body_index_wo_spr, pt_on_body);
}

void TaskSpaceTrackingData::AddPointToTrack(vector<int> body_index_wo_spr,
    vector<VectorXd> pt_on_body) {
  body_index_wo_spr_.insert(body_index_wo_spr_.end(),
                            body_index_wo_spr.begin(), body_index_wo_spr.end());
  pt_on_body_.insert(pt_on_body_.end(), pt_on_body.begin(), pt_on_body.end());
}
void TaskSpaceTrackingData::AddPointToTrack(vector<int> body_index_wo_spr,
    vector<VectorXd> pt_on_body, vector<int> state) {
  state_.insert(state_.end(), state.begin(), state.end());
  AddPointToTrack(body_index_wo_spr, pt_on_body);
}
void TaskSpaceTrackingData::AddPointToTrack(vector<int> body_index_w_spr,
    vector<int> body_index_wo_spr,
    vector<VectorXd> pt_on_body) {
  body_index_w_spr_.insert(body_index_w_spr_.end(),
                           body_index_w_spr.begin(), body_index_w_spr.end());
  AddPointToTrack(body_index_wo_spr, pt_on_body);
}
void TaskSpaceTrackingData::AddPointToTrack(vector<int> body_index_w_spr,
    vector<int> body_index_wo_spr,
    vector<VectorXd> pt_on_body, vector<int> state) {
  body_index_w_spr_.insert(body_index_w_spr_.end(),
                           body_index_w_spr.begin(), body_index_w_spr.end());
  AddPointToTrack(body_index_wo_spr, body_index_wo_spr, pt_on_body);
}

void TaskSpaceTrackingData::CheckDerivedOscTrackingData() {
  if (body_index_w_spr_.empty()) {
    body_index_w_spr_ = body_index_wo_spr_;
  }
  DRAKE_DEMAND(body_index_w_spr_.size() == body_index_wo_spr_.size());
  DRAKE_DEMAND(pt_on_body_.size() == body_index_wo_spr_.size());
  DRAKE_DEMAND(pt_on_body_.size() == body_index_w_spr_.size());
  DRAKE_DEMAND(state_.empty() || (state_.size() == pt_on_body_.size()));
}

// TransTaskSpaceTrackingData //////////////////////////////////////////
TransTaskSpaceTrackingData::TransTaskSpaceTrackingData(string name,
    int n_r,
    MatrixXd K_p,
    MatrixXd K_d,
    MatrixXd W,
    bool traj_is_const,
    bool traj_has_exp,
    bool track_center_of_mass) : TaskSpaceTrackingData(name, n_r,
          K_p, K_d, W, traj_is_const, traj_has_exp),
  track_center_of_mass_(track_center_of_mass) {
}

void TransTaskSpaceTrackingData::UpdateYAndError(const VectorXd& x_w_spr,
    KinematicsCache<double>& cache_w_spr, RigidBodyTree<double>* tree_w_spr) {
  if (track_center_of_mass_) {
    y_ = tree_w_spr->centerOfMass(cache_w_spr);
  } else {
    y_ = tree_w_spr->CalcBodyPoseInWorldFrame(
           cache_w_spr, tree_w_spr->get_body(body_index_w_spr_.at(
                 GetStateIdx()))).translation();
  }
  error_y_ = y_des_ - y_;
}
void TransTaskSpaceTrackingData::UpdateYdot(const VectorXd& x_w_spr,
    KinematicsCache<double>& cache_w_spr, RigidBodyTree<double>* tree_w_spr) {
  if (track_center_of_mass_) {
    ydot_ = tree_w_spr->centerOfMassJacobian(cache_w_spr) *
            x_w_spr.tail(tree_w_spr->get_num_velocities());
  } else {
    ydot_ = tree_w_spr->transformPointsJacobian(cache_w_spr,
            pt_on_body_.at(GetStateIdx()),
            body_index_w_spr_.at(GetStateIdx()), 0, false) *
            x_w_spr.tail(tree_w_spr->get_num_velocities());
  }
}
void TransTaskSpaceTrackingData::UpdateJ(const VectorXd& x_wo_spr,
    KinematicsCache<double>& cache_wo_spr,
    RigidBodyTree<double>* tree_wo_spr) {
  if (track_center_of_mass_) {
    J_ = tree_wo_spr->centerOfMassJacobian(cache_wo_spr);
  } else {
    J_ = tree_wo_spr->transformPointsJacobian(cache_wo_spr,
         pt_on_body_.at(GetStateIdx()),
         body_index_wo_spr_.at(GetStateIdx()), 0, false);
  }
}
void TransTaskSpaceTrackingData::UpdateJdotV(const VectorXd& x_wo_spr,
    KinematicsCache<double>& cache_wo_spr, RigidBodyTree<double>* tree_wo_spr) {
  if (track_center_of_mass_) {
    JdotV_ = tree_wo_spr->centerOfMassJacobianDotTimesV(cache_wo_spr);
  } else {
    JdotV_ = tree_wo_spr->transformPointsJacobianDotTimesV(cache_wo_spr,
             pt_on_body_.at(GetStateIdx()),
             body_index_wo_spr_.at(GetStateIdx()), 0);
  }
}

// RotTaskSpaceTrackingData /////////////////////////////////////////////
RotTaskSpaceTrackingData::RotTaskSpaceTrackingData(string name,
    int n_r,
    MatrixXd K_p,
    MatrixXd K_d,
    MatrixXd W,
    bool traj_is_const,
    bool traj_has_exp,
    Eigen::Isometry3d isometry) : TaskSpaceTrackingData(name, n_r,
          K_p, K_d, W, traj_is_const, traj_has_exp),
  isometry_(isometry) {
}

void RotTaskSpaceTrackingData::UpdateYAndError(const VectorXd& x_w_spr,
    KinematicsCache<double>& cache_w_spr, RigidBodyTree<double>* tree_w_spr) {
  // Get the quaternion in the form of VectorXd
  Eigen::Matrix3d rot_mat = tree_w_spr->CalcBodyPoseInWorldFrame(
                              cache_w_spr, tree_w_spr->get_body(body_index_w_spr_.at(
                                    GetStateIdx()))).linear();
  Quaterniond y_quat(rot_mat);
  y_ << y_quat.w(), y_quat.vec();
  cout << "RotTaskSpaceTrackingData::UpdateYAndError(): y_ = " << y_.transpose()
       << endl;
  Eigen::Vector4d y_des_4d = y_des_;
  Quaterniond y_quat_des(y_des_4d);

  // Get relative quaternion (from current to desired)
  Quaterniond relative_qaut = NormalizeQuaternion(y_quat_des * y_quat.inverse());
  double theta = 2 * acos(relative_qaut.w());
  Vector3d rot_axis = relative_qaut.vec() / sin(theta / 2);

  error_y_ = theta * rot_axis;
}
void RotTaskSpaceTrackingData::UpdateYdot(const VectorXd& x_w_spr,
    KinematicsCache<double>& cache_w_spr, RigidBodyTree<double>* tree_w_spr) {
  MatrixXd J_spatial = tree_w_spr->CalcFrameSpatialVelocityJacobianInWorldFrame (
                         cache_w_spr,
                         tree_w_spr->get_body(body_index_w_spr_.at(GetStateIdx())), isometry_);
  ydot_ = J_spatial.block(0, 0, 3, J_spatial.cols()) *
          x_w_spr.tail(tree_w_spr->get_num_velocities());
}
void RotTaskSpaceTrackingData::UpdateJ(const VectorXd& x_wo_spr,
                                       KinematicsCache<double>& cache_wo_spr,
                                       RigidBodyTree<double>* tree_wo_spr) {
  MatrixXd J_spatial = tree_wo_spr->CalcFrameSpatialVelocityJacobianInWorldFrame (
                         cache_wo_spr,
                         tree_wo_spr->get_body(body_index_wo_spr_.at(GetStateIdx())), isometry_);
  J_ = J_spatial.block(0, 0, 3, J_spatial.cols());
}
void RotTaskSpaceTrackingData::UpdateJdotV(const VectorXd& x_wo_spr,
    KinematicsCache<double>& cache_wo_spr, RigidBodyTree<double>* tree_wo_spr) {
  JdotV_ = tree_wo_spr->CalcFrameSpatialVelocityJacobianDotTimesVInWorldFrame (
             cache_wo_spr,
             tree_wo_spr->get_body(body_index_wo_spr_.at(GetStateIdx())), isometry_).head(3);
}

// JointSpaceTrackingData //////////////////////////////////////////////////////
JointSpaceTrackingData::JointSpaceTrackingData(string name,
    int n_r,
    MatrixXd K_p,
    MatrixXd K_d,
    MatrixXd W,
    bool traj_is_const,
    bool traj_has_exp) : OscTrackingData(name, n_r,
          K_p, K_d, W, traj_is_const, traj_has_exp) {
  DRAKE_DEMAND(n_r == 1);  // one joint at a time
}

void JointSpaceTrackingData::AddJointToTrack(int joint_pos_idx_wo_spr,
    int joint_vel_idx_wo_spr) {
  joint_pos_idx_wo_spr_.push_back(joint_pos_idx_wo_spr);
  joint_vel_idx_wo_spr_.push_back(joint_vel_idx_wo_spr);
}
void JointSpaceTrackingData::AddJointToTrack(int joint_pos_idx_wo_spr,
    int joint_vel_idx_wo_spr, int state) {
  state_.push_back(state);
  AddJointToTrack(joint_pos_idx_wo_spr, joint_vel_idx_wo_spr);
}
void JointSpaceTrackingData::AddJointToTrack(int joint_pos_idx_w_spr,
    int joint_vel_idx_w_spr,
    int joint_pos_idx_wo_spr,
    int joint_vel_idx_wo_spr) {
  joint_pos_idx_w_spr_.push_back(joint_pos_idx_w_spr);
  joint_vel_idx_w_spr_.push_back(joint_vel_idx_w_spr);
  AddJointToTrack(joint_pos_idx_wo_spr, joint_vel_idx_wo_spr);
}
void JointSpaceTrackingData::AddJointToTrack(int joint_pos_idx_w_spr,
    int joint_vel_idx_w_spr,
    int joint_pos_idx_wo_spr,
    int joint_vel_idx_wo_spr,
    int state) {
  state_.push_back(state);
  AddJointToTrack(joint_pos_idx_w_spr, joint_vel_idx_w_spr,
                  joint_pos_idx_wo_spr, joint_vel_idx_wo_spr);
}

void JointSpaceTrackingData::AddJointToTrack(vector<int> joint_pos_idx_wo_spr,
    vector<int> joint_vel_idx_wo_spr) {
  joint_pos_idx_wo_spr_.insert(joint_pos_idx_wo_spr_.end(),
                               joint_pos_idx_wo_spr.begin(), joint_pos_idx_wo_spr.end());
  joint_vel_idx_wo_spr_.insert(joint_vel_idx_wo_spr_.end(),
                               joint_vel_idx_wo_spr.begin(), joint_vel_idx_wo_spr.end());
}
void JointSpaceTrackingData::AddJointToTrack(vector<int> joint_pos_idx_wo_spr,
    vector<int> joint_vel_idx_wo_spr, vector<int> state) {
  state_.insert(state_.end(), state.begin(), state.end());
  AddJointToTrack(joint_pos_idx_wo_spr, joint_vel_idx_wo_spr);
}
void JointSpaceTrackingData::AddJointToTrack(vector<int> joint_pos_idx_w_spr,
    vector<int> joint_vel_idx_w_spr,
    vector<int> joint_pos_idx_wo_spr,
    vector<int> joint_vel_idx_wo_spr) {
  joint_pos_idx_w_spr_.insert(joint_pos_idx_w_spr_.end(),
                              joint_pos_idx_w_spr.begin(), joint_pos_idx_w_spr.end());
  joint_vel_idx_w_spr_.insert(joint_vel_idx_w_spr_.end(),
                              joint_vel_idx_w_spr.begin(), joint_vel_idx_w_spr.end());
  AddJointToTrack(joint_pos_idx_wo_spr, joint_vel_idx_wo_spr);
}
void JointSpaceTrackingData::AddJointToTrack(vector<int> joint_pos_idx_w_spr,
    vector<int> joint_vel_idx_w_spr,
    vector<int> joint_pos_idx_wo_spr,
    vector<int> joint_vel_idx_wo_spr,
    vector<int> state) {
  state_.insert(state_.end(), state.begin(), state.end());
  AddJointToTrack(joint_pos_idx_w_spr, joint_vel_idx_w_spr,
                  joint_pos_idx_wo_spr, joint_vel_idx_wo_spr);
}

void JointSpaceTrackingData::UpdateYAndError(const VectorXd& x_w_spr,
    KinematicsCache<double>& cache_w_spr, RigidBodyTree<double>* tree_w_spr) {
  y_ = x_w_spr.segment(joint_pos_idx_wo_spr_.at(GetStateIdx()), 1);
  error_y_ = y_des_ - y_;
}
void JointSpaceTrackingData::UpdateYdot(const VectorXd& x_w_spr,
                                        KinematicsCache<double>& cache_w_spr, RigidBodyTree<double>* tree_w_spr) {
  J_ = MatrixXd::Zero(1, tree_w_spr->get_num_velocities());
  J_(0, joint_vel_idx_wo_spr_.at(GetStateIdx())) = 1;
}
void JointSpaceTrackingData::UpdateJ(const VectorXd& x_wo_spr,
                                     KinematicsCache<double>& cache_wo_spr,
                                     RigidBodyTree<double>* tree_wo_spr) {
  J_ = MatrixXd::Zero(1, tree_wo_spr->get_num_velocities());
  J_(0, joint_vel_idx_wo_spr_.at(GetStateIdx())) = 1;
}
void JointSpaceTrackingData::UpdateJdotV(const VectorXd& x_wo_spr,
    KinematicsCache<double>& cache_wo_spr, RigidBodyTree<double>* tree_wo_spr) {
  JdotV_ = VectorXd::Zero(GetTrajDim());
}

void JointSpaceTrackingData::CheckDerivedOscTrackingData() {
  if (joint_pos_idx_w_spr_.empty()) {
    joint_pos_idx_w_spr_ = joint_pos_idx_wo_spr_;
  }
  if (joint_vel_idx_w_spr_.empty()) {
    joint_vel_idx_w_spr_ = joint_vel_idx_wo_spr_;
  }
  DRAKE_DEMAND(joint_pos_idx_w_spr_.size() == joint_pos_idx_wo_spr_.size());
  DRAKE_DEMAND(joint_vel_idx_w_spr_.size() == joint_vel_idx_wo_spr_.size());
  DRAKE_DEMAND(state_.empty() ||
               ((state_.size() == joint_pos_idx_wo_spr_.size()) &&
                (state_.size() == joint_vel_idx_wo_spr_.size())));
}

// AbstractTrackingData ////////////////////////////////////////////////////////
AbstractTrackingData::AbstractTrackingData(string name,
    int n_r,
    MatrixXd K_p,
    MatrixXd K_d,
    MatrixXd W,
    bool traj_is_const,
    bool traj_has_exp) : OscTrackingData(name, n_r,
          K_p, K_d, W, traj_is_const, traj_has_exp) {
}

void AbstractTrackingData::UpdateYAndError(const VectorXd& x_w_spr,
    KinematicsCache<double>& cache_w_spr,
    RigidBodyTree<double>* tree_w_spr) {
  // Not implemented yet
}
void AbstractTrackingData::UpdateYdot(const VectorXd& x_w_spr,
                                      KinematicsCache<double>& cache_w_spr,
                                      RigidBodyTree<double>* tree_w_spr) {
  // Not implemented yet
}
void AbstractTrackingData::UpdateJ(const VectorXd& x_wo_spr,
                                   KinematicsCache<double>& cache_wo_spr,
                                   RigidBodyTree<double>* tree_wo_spr) {
  // Not implemented yet
}
void AbstractTrackingData::UpdateJdotV(const VectorXd& x_wo_spr,
                                       KinematicsCache<double>& cache_wo_spr,
                                       RigidBodyTree<double>* tree_wo_spr) {
  // Not implemented yet
}
void AbstractTrackingData::CheckDerivedOscTrackingData() {
  // Not implemented yet
}



}  // namespace controllers
}  // namespace systems
}  // namespace dairlib
