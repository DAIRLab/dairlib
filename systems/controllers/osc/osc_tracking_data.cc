#include "systems/controllers/osc/osc_tracking_data.h"

#include <math.h>
#include <algorithm>

#include "attic/multibody/rigidbody_utils.h"

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

using multibody::GetBodyIndexFromName;
using multibody::makeNameToPositionsMap;
using multibody::makeNameToVelocitiesMap;

// OscTrackingData /////////////////////////////////////////////////////////////
OscTrackingData::OscTrackingData(string name,
                                 int n_r,
                                 MatrixXd K_p,
                                 MatrixXd K_d,
                                 MatrixXd W,
                                 const RigidBodyTree<double>* tree_w_spr,
                                 const RigidBodyTree<double>* tree_wo_spr) :
    tree_w_spr_(tree_w_spr),
    tree_wo_spr_(tree_wo_spr),
    name_(name),
    n_r_(n_r),
    K_p_(K_p),
    K_d_(K_d),
    W_(W) {
}

// Updater
bool OscTrackingData::Update(VectorXd x_w_spr,
    KinematicsCache<double>& cache_w_spr,
    VectorXd x_wo_spr,
    KinematicsCache<double>& cache_wo_spr,
    const drake::trajectories::Trajectory<double>& traj, double t,
    int finite_state_machine_state) {
  // Update track_at_current_step_
  UpdateTrackingFlag(finite_state_machine_state);

  // Proceed based on the result of track_at_current_step_
  if (!track_at_current_step_) {
    return track_at_current_step_;
  } else {
    // Careful: must update y_des_ before calling UpdateYAndError()

    // Update desired output
    y_des_ = traj.value(t);
    dy_des_ = traj.MakeDerivative(1)->value(t);
    ddy_des_ = traj.MakeDerivative(2)->value(t);

    // Update feedback output (Calling virtual methods)
    UpdateYAndError(x_w_spr, cache_w_spr);
    UpdateYdot(x_w_spr, cache_w_spr);
    UpdateJ(x_wo_spr, cache_wo_spr);
    UpdateJdotV(x_wo_spr, cache_wo_spr);

    // Update command output (desired output with pd control)
    ddy_command_ = ddy_des_ + K_p_ * (error_y_) + K_d_ * (dy_des_ - dy_);

    return track_at_current_step_;
  }
}

void OscTrackingData::UpdateTrackingFlag(int finite_state_machine_state) {
  if (state_.empty()) {
    track_at_current_step_ = true;
    state_idx_ = 0;
    return;
  }

  vector<int>::iterator it = find(state_.begin(),
                                  state_.end(),
                                  finite_state_machine_state);
  state_idx_ = std::distance(state_.begin(), it);
  if (it != state_.end()) {
    track_at_current_step_ = true;
  } else {
    track_at_current_step_ = false;
  }
}

void OscTrackingData::PrintFeedbackAndDesiredValues(VectorXd dv) {
  cout << name_ << ":\n";
  cout << "  y = " << y_.transpose() << endl;
  cout << "  y_des = " << y_des_.transpose() << endl;
  cout << "  error_y = " << error_y_.transpose() << endl;
  cout << "  dy = " << dy_.transpose() << endl;
  cout << "  dy_des = " << dy_des_.transpose() << endl;
  cout << "  ddy_des = " << ddy_des_.transpose() << endl;
  cout << "  ddy_command = " << ddy_command_.transpose() << endl;
  cout << "  ddy_command_sol = " << (J_ * dv + JdotV_).transpose() << endl;
}

void OscTrackingData::AddState(int state) {
  // Avoid repeated states
  for (auto const & element : state_) {
    DRAKE_DEMAND(element != state);
  }
  state_.push_back(state);
}

// Run this function in OSC constructor to make sure that users constructed
// OscTrackingData correctly.
void OscTrackingData::CheckOscTrackingData() {
  cout << "Checking " << name_ << endl;
  CheckDerivedOscTrackingData();

  DRAKE_DEMAND((K_p_.rows() == n_r_) && (K_p_.cols() == n_r_));
  DRAKE_DEMAND((K_d_.rows() == n_r_) && (K_d_.cols() == n_r_));
  DRAKE_DEMAND((W_.rows() == n_r_) && (W_.cols() == n_r_));

  // State_ cannot have repeated state
  vector<int>::iterator it = std::unique(state_.begin(), state_.end());
  bool all_state_are_different = (it == state_.end() );
  DRAKE_DEMAND(all_state_are_different);
}


// ComTrackingData /////////////////////////////////////////////////////////////
ComTrackingData::ComTrackingData(string name,
    int n_r,
    MatrixXd K_p,
    MatrixXd K_d,
    MatrixXd W,
    const RigidBodyTree<double>* tree_w_spr,
    const RigidBodyTree<double>* tree_wo_spr) : OscTrackingData(name, n_r,
          K_p, K_d, W, tree_w_spr, tree_wo_spr) {
}

void ComTrackingData::AddStateToTrack(int state){
    AddState(state);
}

void ComTrackingData::UpdateYAndError(const VectorXd& x_w_spr,
    KinematicsCache<double>& cache_w_spr) {
  y_ = tree_w_spr_->centerOfMass(cache_w_spr);
  error_y_ = y_des_ - y_;
}
void ComTrackingData::UpdateYdot(const VectorXd& x_w_spr,
    KinematicsCache<double>& cache_w_spr) {
  dy_ = tree_w_spr_->centerOfMassJacobian(cache_w_spr) *
        x_w_spr.tail(tree_w_spr_->get_num_velocities());
}
void ComTrackingData::UpdateJ(const VectorXd& x_wo_spr,
    KinematicsCache<double>& cache_wo_spr) {
  J_ = tree_wo_spr_->centerOfMassJacobian(cache_wo_spr);
}
void ComTrackingData::UpdateJdotV(const VectorXd& x_wo_spr,
    KinematicsCache<double>& cache_wo_spr) {
  JdotV_ = tree_wo_spr_->centerOfMassJacobianDotTimesV(cache_wo_spr);
}

void ComTrackingData::CheckDerivedOscTrackingData() {
}


// TaskSpaceTrackingData ///////////////////////////////////////////////////////
TaskSpaceTrackingData::TaskSpaceTrackingData(string name,
    int n_r,
    MatrixXd K_p,
    MatrixXd K_d,
    MatrixXd W,
    const RigidBodyTree<double>* tree_w_spr,
    const RigidBodyTree<double>* tree_wo_spr) : OscTrackingData(name, n_r,
          K_p, K_d, W, tree_w_spr, tree_wo_spr) {
}

// TransTaskSpaceTrackingData //////////////////////////////////////////////////
TransTaskSpaceTrackingData::TransTaskSpaceTrackingData(string name,
    int n_r,
    MatrixXd K_p,
    MatrixXd K_d,
    MatrixXd W,
    const RigidBodyTree<double>* tree_w_spr,
    const RigidBodyTree<double>* tree_wo_spr) : TaskSpaceTrackingData(name, n_r,
          K_p, K_d, W, tree_w_spr, tree_wo_spr) {
}

void TransTaskSpaceTrackingData::AddPointToTrack(std::string body_name,
    Vector3d pt_on_body) {
  DRAKE_DEMAND(GetBodyIndexFromName(*tree_w_spr_, body_name) >= 0);
  DRAKE_DEMAND(GetBodyIndexFromName(*tree_wo_spr_, body_name) >= 0);
  body_index_w_spr_.push_back(GetBodyIndexFromName(*tree_w_spr_, body_name));
  body_index_wo_spr_.push_back(GetBodyIndexFromName(*tree_wo_spr_, body_name));
  pt_on_body_.push_back(pt_on_body);
}
void TransTaskSpaceTrackingData::AddStateAndPointToTrack(int state,
    std::string body_name, Vector3d pt_on_body) {
  AddState(state);
  AddPointToTrack(body_name, pt_on_body);
}

void TransTaskSpaceTrackingData::UpdateYAndError(const VectorXd& x_w_spr,
    KinematicsCache<double>& cache_w_spr) {
  y_ = tree_w_spr_->transformPoints(cache_w_spr, pt_on_body_.at(GetStateIdx()),
      body_index_w_spr_.at(GetStateIdx()), 0);
  error_y_ = y_des_ - y_;
}
void TransTaskSpaceTrackingData::UpdateYdot(const VectorXd& x_w_spr,
    KinematicsCache<double>& cache_w_spr) {
  dy_ = tree_w_spr_->transformPointsJacobian(cache_w_spr,
        pt_on_body_.at(GetStateIdx()),
        body_index_w_spr_.at(GetStateIdx()), 0, false) *
        x_w_spr.tail(tree_w_spr_->get_num_velocities());
}
void TransTaskSpaceTrackingData::UpdateJ(const VectorXd& x_wo_spr,
    KinematicsCache<double>& cache_wo_spr) {
  J_ = tree_wo_spr_->transformPointsJacobian(cache_wo_spr,
       pt_on_body_.at(GetStateIdx()),
       body_index_wo_spr_.at(GetStateIdx()), 0, false);
}
void TransTaskSpaceTrackingData::UpdateJdotV(const VectorXd& x_wo_spr,
    KinematicsCache<double>& cache_wo_spr) {
  JdotV_ = tree_wo_spr_->transformPointsJacobianDotTimesV(cache_wo_spr,
           pt_on_body_.at(GetStateIdx()),
           body_index_wo_spr_.at(GetStateIdx()), 0);
}

void TransTaskSpaceTrackingData::CheckDerivedOscTrackingData() {
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

// RotTaskSpaceTrackingData ////////////////////////////////////////////////////
RotTaskSpaceTrackingData::RotTaskSpaceTrackingData(string name,
    int n_r,
    MatrixXd K_p,
    MatrixXd K_d,
    MatrixXd W,
    const RigidBodyTree<double>* tree_w_spr,
    const RigidBodyTree<double>* tree_wo_spr) : TaskSpaceTrackingData(name, n_r,
          K_p, K_d, W, tree_w_spr, tree_wo_spr) {
}

void RotTaskSpaceTrackingData::AddFrameToTrack(std::string body_name,
    Isometry3d frame_pose) {
  DRAKE_DEMAND(GetBodyIndexFromName(*tree_w_spr_, body_name) >= 0);
  DRAKE_DEMAND(GetBodyIndexFromName(*tree_wo_spr_, body_name) >= 0);
  body_index_w_spr_.push_back(GetBodyIndexFromName(*tree_w_spr_, body_name));
  body_index_wo_spr_.push_back(GetBodyIndexFromName(*tree_wo_spr_, body_name));
  frame_pose_.push_back(frame_pose);
}
void RotTaskSpaceTrackingData::AddStateAndFrameToTrack(int state,
    std::string body_name, Isometry3d frame_pose) {
  AddState(state);
  AddFrameToTrack(body_name, frame_pose);
}

void RotTaskSpaceTrackingData::UpdateYAndError(const VectorXd& x_w_spr,
    KinematicsCache<double>& cache_w_spr) {
  // Get the quaternion in the form of VectorXd
  Eigen::Matrix3d rot_mat = tree_w_spr_->CalcBodyPoseInWorldFrame(
                              cache_w_spr, tree_w_spr_->get_body(
                                body_index_w_spr_.at(GetStateIdx()))).linear();
  Quaterniond y_quat(rot_mat * frame_pose_.at(GetStateIdx()).linear());
  Eigen::Vector4d y_4d;
  y_4d << y_quat.w(), y_quat.vec();
  y_ = y_4d;
  DRAKE_DEMAND(y_des_.size() == 4);
  Quaterniond y_quat_des(y_des_(0), y_des_(1), y_des_(2), y_des_(3));

  // Get relative quaternion (from current to desired)
  Quaterniond relative_qaut = (y_quat_des * y_quat.inverse()).normalized();
  double theta = 2 * acos(relative_qaut.w());
  Vector3d rot_axis = relative_qaut.vec().normalized();

  error_y_ = theta * rot_axis;
}
void RotTaskSpaceTrackingData::UpdateYdot(const VectorXd& x_w_spr,
    KinematicsCache<double>& cache_w_spr) {
  MatrixXd J_spatial = tree_w_spr_->CalcFrameSpatialVelocityJacobianInWorldFrame(
      cache_w_spr, tree_w_spr_->get_body(body_index_w_spr_.at(GetStateIdx())),
      frame_pose_.at(GetStateIdx()));
  dy_ = J_spatial.block(0, 0, 3, J_spatial.cols()) *
        x_w_spr.tail(tree_w_spr_->get_num_velocities());
}
void RotTaskSpaceTrackingData::UpdateJ(const VectorXd& x_wo_spr,
    KinematicsCache<double>& cache_wo_spr) {
  MatrixXd J_spatial = tree_wo_spr_->CalcFrameSpatialVelocityJacobianInWorldFrame(
      cache_wo_spr, tree_wo_spr_->get_body(body_index_wo_spr_.at(GetStateIdx())),
      frame_pose_.at(GetStateIdx()));
  J_ = J_spatial.block(0, 0, 3, J_spatial.cols());
}
void RotTaskSpaceTrackingData::UpdateJdotV(const VectorXd& x_wo_spr,
    KinematicsCache<double>& cache_wo_spr) {
  JdotV_ = tree_wo_spr_->CalcFrameSpatialVelocityJacobianDotTimesVInWorldFrame (
             cache_wo_spr,
             tree_wo_spr_->get_body(body_index_wo_spr_.at(GetStateIdx())),
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
    MatrixXd K_p,
    MatrixXd K_d,
    MatrixXd W,
    const RigidBodyTree<double>* tree_w_spr,
    const RigidBodyTree<double>* tree_wo_spr) : OscTrackingData(name, 1,
          K_p, K_d, W, tree_w_spr, tree_wo_spr) {
          // n_r = 1, one joint at a time
}

void JointSpaceTrackingData::AddJointToTrack(std::string joint_pos_name,
    std::string joint_vel_name) {
  joint_pos_idx_w_spr_.push_back(
      makeNameToPositionsMap(*tree_w_spr_).at(joint_pos_name));
  joint_vel_idx_w_spr_.push_back(
      makeNameToVelocitiesMap(*tree_w_spr_).at(joint_vel_name));
  joint_pos_idx_wo_spr_.push_back(
      makeNameToPositionsMap(*tree_wo_spr_).at(joint_pos_name));
  joint_vel_idx_wo_spr_.push_back(
      makeNameToVelocitiesMap(*tree_wo_spr_).at(joint_vel_name));
}
void JointSpaceTrackingData::AddStateAndJointToTrack(int state,
    std::string joint_pos_name, std::string joint_vel_name) {
  AddState(state);
  AddJointToTrack(joint_pos_name, joint_vel_name);
}

void JointSpaceTrackingData::UpdateYAndError(const VectorXd& x_w_spr,
    KinematicsCache<double>& cache_w_spr) {
  y_ = x_w_spr.segment(joint_pos_idx_w_spr_.at(GetStateIdx()), 1);
  error_y_ = y_des_ - y_;
}
void JointSpaceTrackingData::UpdateYdot(const VectorXd& x_w_spr,
    KinematicsCache<double>& cache_w_spr) {
  MatrixXd J = MatrixXd::Zero(1, tree_w_spr_->get_num_velocities());
  J(0, joint_vel_idx_w_spr_.at(GetStateIdx())) = 1;
  dy_ = J * x_w_spr.tail(tree_w_spr_->get_num_velocities());
}
void JointSpaceTrackingData::UpdateJ(const VectorXd& x_wo_spr,
                                     KinematicsCache<double>& cache_wo_spr) {
  J_ = MatrixXd::Zero(1, tree_wo_spr_->get_num_velocities());
  J_(0, joint_vel_idx_wo_spr_.at(GetStateIdx())) = 1;
}
void JointSpaceTrackingData::UpdateJdotV(const VectorXd& x_wo_spr,
    KinematicsCache<double>& cache_wo_spr) {
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


}  // namespace controllers
}  // namespace systems
}  // namespace dairlib
