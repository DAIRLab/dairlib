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
using Eigen::Isometry3d;

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
  cout << "Update data for " << name_ << endl;
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

    // Update desired output
    y_des_ = traj.value(t);
    if (!traj_is_const_) {
      dy_des_ = traj.MakeDerivative(1)->value(t);
      ddy_des_ = traj.MakeDerivative(2)->value(t);
    } else {
      dy_des_ = VectorXd::Zero(n_r_);
      ddy_des_ = VectorXd::Zero(n_r_);
    }
    cout << "y_des_ = " << y_des_.transpose() << endl;
    cout << "dy_des_ = " << dy_des_.transpose() << endl;
    cout << "ddy_des_ = " << ddy_des_.transpose() << endl;

    // Update feedback output (Calling virtual methods)
    cout << "UpdateYAndError...\n";
    UpdateYAndError(x_w_spr, cache_w_spr, tree_w_spr);
    cout << "UpdateYdot...\n";
    UpdateYdot(x_w_spr, cache_w_spr, tree_w_spr);
    cout << "UpdateJ...\n";
    UpdateJ(x_wo_spr, cache_wo_spr, tree_wo_spr);
    cout << "UpdateJdotV...\n";
    UpdateJdotV(x_wo_spr, cache_wo_spr, tree_wo_spr);
    cout << "Finished updating\n";

    return track_at_current_step_;
  }
}

// Getters
VectorXd OscTrackingData::GetDesiredOutputWithPdControl() {
  cout << "ddy_des_ = " << ddy_des_ << endl;
  cout << "dy_des_ = " << dy_des_ << endl;
  cout << "y_des_ = " << y_des_ << endl;
  cout << "ydot_ = " << ydot_ << endl;
  cout << "y_ = " << y_ << endl;
  cout << "K_d_ = " << K_d_ << endl;
  cout << "K_p_ = " << K_p_ << endl;
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
    return ;
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
  cout << "Checking " << name_ << endl;
  CheckDerivedOscTrackingData();

  DRAKE_DEMAND(!(traj_is_const_ && traj_has_exp_));
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

void TransTaskSpaceTrackingData::AddPointToTrack(int body_index_wo_spr,
    Vector3d pt_on_body) {
  body_index_wo_spr_.push_back(body_index_wo_spr);
  pt_on_body_.push_back(pt_on_body);
}
void TransTaskSpaceTrackingData::AddStateAndPointToTrack(int state,
    int body_index_wo_spr, Vector3d pt_on_body) {
  state_.push_back(state);
  AddPointToTrack(body_index_wo_spr, pt_on_body);
}
void TransTaskSpaceTrackingData::AddPointToTrack(int body_index_w_spr,
    int body_index_wo_spr,
    Vector3d pt_on_body) {
  body_index_w_spr_.push_back(body_index_w_spr);
  AddPointToTrack(body_index_wo_spr, pt_on_body);
}
void TransTaskSpaceTrackingData::AddStateAndPointToTrack(int state,
    int body_index_w_spr, int body_index_wo_spr, Vector3d pt_on_body) {
  state_.push_back(state);
  AddPointToTrack(body_index_w_spr, body_index_wo_spr, pt_on_body);
}

void TransTaskSpaceTrackingData::AddPointToTrack(vector<int> body_index_wo_spr,
    vector<Vector3d> pt_on_body) {
  body_index_wo_spr_.insert(body_index_wo_spr_.end(),
                            body_index_wo_spr.begin(), body_index_wo_spr.end());
  pt_on_body_.insert(pt_on_body_.end(), pt_on_body.begin(), pt_on_body.end());
}
void TransTaskSpaceTrackingData::AddStateAndPointToTrack(vector<int> state,
    vector<int> body_index_wo_spr, vector<Vector3d> pt_on_body) {
  state_.insert(state_.end(), state.begin(), state.end());
  AddPointToTrack(body_index_wo_spr, pt_on_body);
}
void TransTaskSpaceTrackingData::AddPointToTrack(vector<int> body_index_w_spr,
    vector<int> body_index_wo_spr,
    vector<Vector3d> pt_on_body) {
  body_index_w_spr_.insert(body_index_w_spr_.end(),
                           body_index_w_spr.begin(), body_index_w_spr.end());
  AddPointToTrack(body_index_wo_spr, pt_on_body);
}
void TransTaskSpaceTrackingData::AddStateAndPointToTrack(vector<int> state,
    vector<int> body_index_w_spr, vector<int> body_index_wo_spr,
    vector<Vector3d> pt_on_body) {
  state_.insert(state_.end(), state.begin(), state.end());
  AddPointToTrack(body_index_w_spr, body_index_wo_spr, pt_on_body);
}

void TransTaskSpaceTrackingData::UpdateYAndError(const VectorXd& x_w_spr,
    KinematicsCache<double>& cache_w_spr, RigidBodyTree<double>* tree_w_spr) {
  if (track_center_of_mass_) {
    y_ = tree_w_spr->centerOfMass(cache_w_spr);
  } else {
    Isometry3d body_pose = tree_w_spr->CalcBodyPoseInWorldFrame(cache_w_spr,
                           tree_w_spr->get_body(
                             body_index_w_spr_.at(GetStateIdx())));
    y_ = body_pose.translation() +
         body_pose.linear() * pt_on_body_.at(GetStateIdx());
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

void TransTaskSpaceTrackingData::CheckDerivedOscTrackingData() {
  if (track_center_of_mass_) {
    DRAKE_DEMAND(body_index_w_spr_.size() == 0);
    DRAKE_DEMAND(body_index_wo_spr_.size() == 0);
    DRAKE_DEMAND(pt_on_body_.size() == 0);
  } else {
    if (body_index_w_spr_.empty()) {
      body_index_w_spr_ = body_index_wo_spr_;
    }
    DRAKE_DEMAND(body_index_w_spr_.size() == pt_on_body_.size());
    DRAKE_DEMAND(body_index_wo_spr_.size() == pt_on_body_.size());
    DRAKE_DEMAND(state_.empty() || (state_.size() == pt_on_body_.size()));
    if (state_.empty()) {
      DRAKE_DEMAND(body_index_w_spr_.size() == 1);
      DRAKE_DEMAND(body_index_wo_spr_.size() == 1);
      DRAKE_DEMAND(pt_on_body_.size() == 1);
    }
  }
}

// RotTaskSpaceTrackingData /////////////////////////////////////////////
RotTaskSpaceTrackingData::RotTaskSpaceTrackingData(string name,
    int n_r,
    MatrixXd K_p,
    MatrixXd K_d,
    MatrixXd W,
    bool traj_is_const,
    bool traj_has_exp) : TaskSpaceTrackingData(name, n_r,
          K_p, K_d, W, traj_is_const, traj_has_exp) {
}

void RotTaskSpaceTrackingData::AddFrameToTrack(int body_index_wo_spr,
    Isometry3d frame_pose) {
  body_index_wo_spr_.push_back(body_index_wo_spr);
  frame_pose_.push_back(frame_pose);
}
void RotTaskSpaceTrackingData::AddStateAndFrameToTrack(int state,
    int body_index_wo_spr, Isometry3d frame_pose) {
  state_.push_back(state);
  AddFrameToTrack(body_index_wo_spr, frame_pose);
}
void RotTaskSpaceTrackingData::AddFrameToTrack(int body_index_w_spr,
    int body_index_wo_spr,
    Isometry3d frame_pose) {
  body_index_w_spr_.push_back(body_index_w_spr);
  AddFrameToTrack(body_index_wo_spr, frame_pose);
}
void RotTaskSpaceTrackingData::AddStateAndFrameToTrack(int state,
    int body_index_w_spr, int body_index_wo_spr, Isometry3d frame_pose) {
  state_.push_back(state);
  AddFrameToTrack(body_index_w_spr, body_index_wo_spr, frame_pose);
}

void RotTaskSpaceTrackingData::AddFrameToTrack(vector<int> body_index_wo_spr,
    vector<Isometry3d> frame_pose) {
  body_index_wo_spr_.insert(body_index_wo_spr_.end(),
                            body_index_wo_spr.begin(), body_index_wo_spr.end());
  frame_pose_.insert(frame_pose_.end(), frame_pose.begin(), frame_pose.end());
}
void RotTaskSpaceTrackingData::AddStateAndFrameToTrack(vector<int> state,
    vector<int> body_index_wo_spr, vector<Isometry3d> frame_pose) {
  state_.insert(state_.end(), state.begin(), state.end());
  AddFrameToTrack(body_index_wo_spr, frame_pose);
}
void RotTaskSpaceTrackingData::AddFrameToTrack(vector<int> body_index_w_spr,
    vector<int> body_index_wo_spr,
    vector<Isometry3d> frame_pose) {
  body_index_w_spr_.insert(body_index_w_spr_.end(),
                           body_index_w_spr.begin(), body_index_w_spr.end());
  AddFrameToTrack(body_index_wo_spr, frame_pose);
}
void RotTaskSpaceTrackingData::AddStateAndFrameToTrack(vector<int> state,
    vector<int> body_index_w_spr, vector<int> body_index_wo_spr,
    vector<Isometry3d> frame_pose) {
  state_.insert(state_.end(), state.begin(), state.end());
  AddFrameToTrack(body_index_w_spr, body_index_wo_spr, frame_pose);
}

void RotTaskSpaceTrackingData::UpdateYAndError(const VectorXd& x_w_spr,
    KinematicsCache<double>& cache_w_spr, RigidBodyTree<double>* tree_w_spr) {
  // Get the quaternion in the form of VectorXd
  Eigen::Matrix3d rot_mat = tree_w_spr->CalcBodyPoseInWorldFrame(
                              cache_w_spr, tree_w_spr->get_body(
                                body_index_w_spr_.at(GetStateIdx()))).linear();
  Quaterniond y_quat(rot_mat * frame_pose_.at(GetStateIdx()).linear());
  Eigen::Vector4d y_4d;
  y_4d << y_quat.w(), y_quat.vec();
  y_ = y_4d;
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
                         tree_w_spr->get_body(body_index_w_spr_.at(GetStateIdx())),
                         frame_pose_.at(GetStateIdx()));
  ydot_ = J_spatial.block(0, 0, 3, J_spatial.cols()) *
          x_w_spr.tail(tree_w_spr->get_num_velocities());
}
void RotTaskSpaceTrackingData::UpdateJ(const VectorXd& x_wo_spr,
                                       KinematicsCache<double>& cache_wo_spr,
                                       RigidBodyTree<double>* tree_wo_spr) {
  MatrixXd J_spatial = tree_wo_spr->CalcFrameSpatialVelocityJacobianInWorldFrame (
                         cache_wo_spr,
                         tree_wo_spr->get_body(body_index_wo_spr_.at(GetStateIdx())),
                         frame_pose_.at(GetStateIdx()));
  J_ = J_spatial.block(0, 0, 3, J_spatial.cols());
}
void RotTaskSpaceTrackingData::UpdateJdotV(const VectorXd& x_wo_spr,
    KinematicsCache<double>& cache_wo_spr, RigidBodyTree<double>* tree_wo_spr) {
  JdotV_ = tree_wo_spr->CalcFrameSpatialVelocityJacobianDotTimesVInWorldFrame (
             cache_wo_spr,
             tree_wo_spr->get_body(body_index_wo_spr_.at(GetStateIdx())),
             frame_pose_.at(GetStateIdx())).head(3);
}

void RotTaskSpaceTrackingData::CheckDerivedOscTrackingData() {
  if (body_index_w_spr_.empty()) {
    body_index_w_spr_ = body_index_wo_spr_;
  }
  DRAKE_DEMAND(body_index_w_spr_.size() == frame_pose_.size());
  DRAKE_DEMAND(body_index_wo_spr_.size() == frame_pose_.size());
  DRAKE_DEMAND(state_.empty() || (state_.size() == frame_pose_.size()));
  if (state_.empty()) {
    DRAKE_DEMAND(body_index_w_spr_.size() == 1);
    DRAKE_DEMAND(body_index_wo_spr_.size() == 1);
    DRAKE_DEMAND(frame_pose_.size() == 1);
  }
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
void JointSpaceTrackingData::AddStateAndJointToTrack(int state,
    int joint_pos_idx_wo_spr, int joint_vel_idx_wo_spr) {
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
void JointSpaceTrackingData::AddStateAndJointToTrack(int state,
    int joint_pos_idx_w_spr,
    int joint_vel_idx_w_spr,
    int joint_pos_idx_wo_spr,
    int joint_vel_idx_wo_spr) {
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
void JointSpaceTrackingData::AddStateAndJointToTrack(vector<int> state,
    vector<int> joint_pos_idx_wo_spr,
    vector<int> joint_vel_idx_wo_spr) {
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
void JointSpaceTrackingData::AddStateAndJointToTrack(vector<int> state,
    vector<int> joint_pos_idx_w_spr,
    vector<int> joint_vel_idx_w_spr,
    vector<int> joint_pos_idx_wo_spr,
    vector<int> joint_vel_idx_wo_spr) {
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
  MatrixXd J = MatrixXd::Zero(1, tree_w_spr->get_num_velocities());
  J(0, joint_vel_idx_wo_spr_.at(GetStateIdx())) = 1;
  ydot_ = J * x_w_spr.tail(tree_w_spr->get_num_velocities());
}
void JointSpaceTrackingData::UpdateJ(const VectorXd& x_wo_spr,
                                     KinematicsCache<double>& cache_wo_spr,
                                     RigidBodyTree<double>* tree_wo_spr) {
  J_ = MatrixXd::Zero(1, tree_wo_spr->get_num_velocities());
  J_(0, joint_vel_idx_wo_spr_.at(GetStateIdx())) = 1;
}
void JointSpaceTrackingData::UpdateJdotV(const VectorXd& x_wo_spr,
    KinematicsCache<double>& cache_wo_spr, RigidBodyTree<double>* tree_wo_spr) {
  JdotV_ = VectorXd::Zero(1);
}

void JointSpaceTrackingData::CheckDerivedOscTrackingData() {
  if (joint_pos_idx_w_spr_.empty()) {
    joint_pos_idx_w_spr_ = joint_pos_idx_wo_spr_;
    joint_vel_idx_w_spr_ = joint_vel_idx_wo_spr_;
  }
  DRAKE_DEMAND(joint_pos_idx_w_spr_.size() == joint_pos_idx_wo_spr_.size());
  DRAKE_DEMAND(joint_vel_idx_w_spr_.size() == joint_vel_idx_wo_spr_.size());
  DRAKE_DEMAND(state_.empty() ||
               ((state_.size() == joint_pos_idx_wo_spr_.size()) &&
                (state_.size() == joint_vel_idx_wo_spr_.size())));
  if (state_.empty()) {
    DRAKE_DEMAND(joint_pos_idx_w_spr_.size() == 1);
    DRAKE_DEMAND(joint_vel_idx_w_spr_.size() == 1);
    DRAKE_DEMAND(joint_pos_idx_wo_spr_.size() == 1);
    DRAKE_DEMAND(joint_vel_idx_wo_spr_.size() == 1);
  }
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
