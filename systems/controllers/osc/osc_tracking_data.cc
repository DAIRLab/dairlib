#include "systems/controllers/osc/osc_tracking_data.h"

#include <math.h>

#include <algorithm>

#include <drake/multibody/plant/multibody_plant.h>

#include "multibody/multibody_utils.h"

using std::cout;
using std::endl;

using drake::multibody::JacobianWrtVariable;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using Eigen::Isometry3d;
using Eigen::MatrixXd;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::VectorXd;
using std::string;
using std::vector;

namespace dairlib::systems::controllers {

using multibody::makeNameToPositionsMap;
using multibody::makeNameToVelocitiesMap;

/**** OscTrackingData ****/
OscTrackingData::OscTrackingData(const string& name, int n_y, int n_ydot,
                                 const MatrixXd& K_p, const MatrixXd& K_d,
                                 const MatrixXd& W,
                                 const MultibodyPlant<double>& plant_w_spr,
                                 const MultibodyPlant<double>& plant_wo_spr)
    : plant_w_spr_(plant_w_spr),
      plant_wo_spr_(plant_wo_spr),
      world_w_spr_(plant_w_spr_.world_frame()),
      world_wo_spr_(plant_wo_spr_.world_frame()),
      name_(name),
      n_y_(n_y),
      n_ydot_(n_ydot),
      K_p_(K_p),
      K_d_(K_d),
      W_(W) {}

// Update
bool OscTrackingData::Update(
    const VectorXd& x_w_spr, const Context<double>& context_w_spr,
    const VectorXd& x_wo_spr, const Context<double>& context_wo_spr,
    const drake::trajectories::Trajectory<double>& traj, double t,
    int finite_state_machine_state) {
  // Update track_at_current_state_
  UpdateTrackingFlag(finite_state_machine_state);

  // Proceed based on the result of track_at_current_state_
  if (track_at_current_state_) {
    // Careful: must update y_des_ before calling UpdateYAndError()
    // Update desired output
    y_des_ = traj.value(t);
    if (traj.has_derivative()) {
      ydot_des_ = traj.EvalDerivative(t, 1);
      yddot_des_ = traj.EvalDerivative(t, 2);
    }
    // TODO (yangwill): Remove this edge case after EvalDerivative has been
    // implemented for ExponentialPlusPiecewisePolynomial
    else {
      ydot_des_ = traj.MakeDerivative(1)->value(t);
      yddot_des_ = traj.MakeDerivative(2)->value(t);
    }

    // Update feedback output (Calling virtual methods)
    UpdateYAndError(x_w_spr, context_w_spr);
    UpdateYdotAndError(x_w_spr, context_w_spr);
    UpdateYddotDes();
    UpdateJ(x_wo_spr, context_wo_spr);
    UpdateJdotV(x_wo_spr, context_wo_spr);

    // Update command output (desired output with pd control)
    yddot_command_ =
        yddot_des_converted_ + K_p_ * (error_y_) + K_d_ * (error_ydot_);
  }
  return track_at_current_state_;
}

void OscTrackingData::UpdateTrackingFlag(int finite_state_machine_state) {
  if (state_.empty()) {
    track_at_current_state_ = true;
    state_idx_ = 0;
    return;
  }

  auto it = find(state_.begin(), state_.end(), finite_state_machine_state);
  state_idx_ = std::distance(state_.begin(), it);
  track_at_current_state_ = it != state_.end();
}

void OscTrackingData::PrintFeedbackAndDesiredValues(const VectorXd& dv) {
  DRAKE_ASSERT(track_at_current_state_);
  cout << name_ << ":\n";
  cout << "  y = " << y_.transpose() << endl;
  cout << "  y_des = " << y_des_.transpose() << endl;
  cout << "  error_y = " << error_y_.transpose() << endl;
  cout << "  ydot = " << ydot_.transpose() << endl;
  cout << "  ydot_des = " << ydot_des_.transpose() << endl;
  cout << "  error_ydot_ = " << error_ydot_.transpose() << endl;
  cout << "  yddot_des_converted = " << yddot_des_converted_.transpose()
       << endl;
  cout << "  yddot_command = " << yddot_command_.transpose() << endl;
  cout << "  yddot_command_sol = " << (J_ * dv + JdotV_).transpose() << endl;
}

void OscTrackingData::SaveYddotCommandSol(const VectorXd& dv) {
  DRAKE_ASSERT(track_at_current_state_);
  yddot_command_sol_ = J_ * dv + JdotV_;
}

void OscTrackingData::AddState(int state) {
  // Avoid repeated states
  for (auto const& element : state_) {
    DRAKE_DEMAND(element != state);
  }
  state_.push_back(state);
}

// Run this function in OSC constructor to make sure that users constructed
// OscTrackingData correctly.
void OscTrackingData::CheckOscTrackingData() {
  cout << "Checking " << name_ << endl;
  CheckDerivedOscTrackingData();

  DRAKE_DEMAND((K_p_.rows() == n_ydot_) && (K_p_.cols() == n_ydot_));
  DRAKE_DEMAND((K_d_.rows() == n_ydot_) && (K_d_.cols() == n_ydot_));
  DRAKE_DEMAND((W_.rows() == n_ydot_) && (W_.cols() == n_ydot_));

  // State_ cannot have repeated state
  auto it = std::unique(state_.begin(), state_.end());
  bool all_state_are_different = (it == state_.end());
  DRAKE_DEMAND(all_state_are_different);
}

/**** ComTrackingData ****/
ComTrackingData::ComTrackingData(const string& name, const MatrixXd& K_p,
                                 const MatrixXd& K_d, const MatrixXd& W,
                                 const MultibodyPlant<double>& plant_w_spr,
                                 const MultibodyPlant<double>& plant_wo_spr)
    : OscTrackingData(name, kSpaceDim, kSpaceDim, K_p, K_d, W, plant_w_spr,
                      plant_wo_spr) {}

void ComTrackingData::AddStateToTrack(int state) { AddState(state); }

void ComTrackingData::UpdateYAndError(const VectorXd& x_w_spr,
                                      const Context<double>& context_w_spr) {
  y_ = plant_w_spr_.CalcCenterOfMassPosition(context_w_spr);
  error_y_ = y_des_ - y_;
}

void ComTrackingData::UpdateYdotAndError(const VectorXd& x_w_spr,
                                         const Context<double>& context_w_spr) {
  MatrixXd J_w_spr(kSpaceDim, plant_w_spr_.num_velocities());
  plant_w_spr_.CalcJacobianCenterOfMassTranslationalVelocity(
      context_w_spr, JacobianWrtVariable::kV, world_w_spr_, world_w_spr_,
      &J_w_spr);
  ydot_ = J_w_spr * x_w_spr.tail(plant_w_spr_.num_velocities());
  error_ydot_ = ydot_des_ - ydot_;
}

void ComTrackingData::UpdateYddotDes() { yddot_des_converted_ = yddot_des_; }

void ComTrackingData::UpdateJ(const VectorXd& x_wo_spr,
                              const Context<double>& context_wo_spr) {
  J_ = MatrixXd::Zero(kSpaceDim, plant_wo_spr_.num_velocities());
  plant_wo_spr_.CalcJacobianCenterOfMassTranslationalVelocity(
      context_wo_spr, JacobianWrtVariable::kV, world_w_spr_, world_w_spr_, &J_);
}

void ComTrackingData::UpdateJdotV(const VectorXd& x_wo_spr,
                                  const Context<double>& context_wo_spr) {
  JdotV_ = plant_wo_spr_.CalcBiasCenterOfMassTranslationalAcceleration(
      context_wo_spr, JacobianWrtVariable::kV, world_wo_spr_, world_wo_spr_);
}

void ComTrackingData::CheckDerivedOscTrackingData() {}

/**** TaskSpaceTrackingData ****/
TaskSpaceTrackingData::TaskSpaceTrackingData(
    const string& name, int n_y, int n_ydot, const MatrixXd& K_p,
    const MatrixXd& K_d, const MatrixXd& W,
    const MultibodyPlant<double>& plant_w_spr,
    const MultibodyPlant<double>& plant_wo_spr)
    : OscTrackingData(name, n_y, n_ydot, K_p, K_d, W, plant_w_spr,
                      plant_wo_spr) {}

/**** TransTaskSpaceTrackingData ****/
TransTaskSpaceTrackingData::TransTaskSpaceTrackingData(
    const string& name, const MatrixXd& K_p, const MatrixXd& K_d,
    const MatrixXd& W, const MultibodyPlant<double>& plant_w_spr,
    const MultibodyPlant<double>& plant_wo_spr)
    : TaskSpaceTrackingData(name, kSpaceDim, kSpaceDim, K_p, K_d, W,
                            plant_w_spr, plant_wo_spr) {}

void TransTaskSpaceTrackingData::AddPointToTrack(const std::string& body_name,
                                                 const Vector3d& pt_on_body) {
  DRAKE_DEMAND(plant_w_spr_.HasBodyNamed(body_name));
  DRAKE_DEMAND(plant_wo_spr_.HasBodyNamed(body_name));
  body_index_w_spr_.push_back(plant_w_spr_.GetBodyByName(body_name).index());
  body_index_wo_spr_.push_back(plant_wo_spr_.GetBodyByName(body_name).index());
  body_frames_w_spr_.push_back(
      &plant_w_spr_.GetBodyByName(body_name).body_frame());
  body_frames_wo_spr_.push_back(
      &plant_wo_spr_.GetBodyByName(body_name).body_frame());
  pts_on_body_.push_back(pt_on_body);
}
void TransTaskSpaceTrackingData::AddStateAndPointToTrack(
    int state, const std::string& body_name, const Vector3d& pt_on_body) {
  AddState(state);
  AddPointToTrack(body_name, pt_on_body);
}

void TransTaskSpaceTrackingData::UpdateYAndError(
    const VectorXd& x_w_spr, const Context<double>& context_w_spr) {
  y_ = Vector3d::Zero();
  plant_w_spr_.CalcPointsPositions(
      context_w_spr, *body_frames_wo_spr_[GetStateIdx()],
      pts_on_body_[GetStateIdx()], world_w_spr_, &y_);
  error_y_ = y_des_ - y_;
}

void TransTaskSpaceTrackingData::UpdateYdotAndError(
    const VectorXd& x_w_spr, const Context<double>& context_w_spr) {
  MatrixXd J(kSpaceDim, plant_w_spr_.num_velocities());
  plant_w_spr_.CalcJacobianTranslationalVelocity(
      context_w_spr, JacobianWrtVariable::kV,
      *body_frames_w_spr_.at(GetStateIdx()), pts_on_body_.at(GetStateIdx()),
      world_w_spr_, world_w_spr_, &J);
  ydot_ = J * x_w_spr.tail(plant_w_spr_.num_velocities());
  error_ydot_ = ydot_des_ - ydot_;
}

void TransTaskSpaceTrackingData::UpdateYddotDes() {
  yddot_des_converted_ = yddot_des_;
}

void TransTaskSpaceTrackingData::UpdateJ(
    const VectorXd& x_wo_spr, const Context<double>& context_wo_spr) {
  J_ = MatrixXd::Zero(kSpaceDim, plant_wo_spr_.num_velocities());
  plant_wo_spr_.CalcJacobianTranslationalVelocity(
      context_wo_spr, JacobianWrtVariable::kV,
      *body_frames_wo_spr_.at(GetStateIdx()), pts_on_body_.at(GetStateIdx()),
      world_wo_spr_, world_wo_spr_, &J_);
}

void TransTaskSpaceTrackingData::UpdateJdotV(
    const VectorXd& x_wo_spr, const Context<double>& context_wo_spr) {
  JdotV_ = plant_wo_spr_.CalcBiasTranslationalAcceleration(
      context_wo_spr, drake::multibody::JacobianWrtVariable::kV,
      *body_frames_wo_spr_.at(GetStateIdx()), pts_on_body_.at(GetStateIdx()),
      world_wo_spr_, world_wo_spr_);
}

void TransTaskSpaceTrackingData::CheckDerivedOscTrackingData() {
  if (body_index_w_spr_.empty()) {
    body_index_w_spr_ = body_index_wo_spr_;
  }
  DRAKE_DEMAND(body_index_w_spr_.size() == pts_on_body_.size());
  DRAKE_DEMAND(body_index_wo_spr_.size() == pts_on_body_.size());
  DRAKE_DEMAND(state_.empty() || (state_.size() == pts_on_body_.size()));
  if (state_.empty()) {
    DRAKE_DEMAND(body_index_w_spr_.size() == 1);
    DRAKE_DEMAND(body_index_wo_spr_.size() == 1);
    DRAKE_DEMAND(pts_on_body_.size() == 1);
  }
}

/**** RotTaskSpaceTrackingData ****/
RotTaskSpaceTrackingData::RotTaskSpaceTrackingData(
    const string& name, const MatrixXd& K_p, const MatrixXd& K_d,
    const MatrixXd& W, const MultibodyPlant<double>& plant_w_spr,
    const MultibodyPlant<double>& plant_wo_spr)
    : TaskSpaceTrackingData(name, kQuaternionDim, kSpaceDim, K_p, K_d, W,
                            plant_w_spr, plant_wo_spr) {}

void RotTaskSpaceTrackingData::AddFrameToTrack(const std::string& body_name,
                                               const Isometry3d& frame_pose) {
  DRAKE_DEMAND(plant_w_spr_.HasBodyNamed(body_name));
  DRAKE_DEMAND(plant_wo_spr_.HasBodyNamed(body_name));
  body_frames_w_spr_.push_back(
      &plant_w_spr_.GetBodyByName(body_name).body_frame());
  body_frames_wo_spr_.push_back(
      &plant_wo_spr_.GetBodyByName(body_name).body_frame());
  body_index_w_spr_.push_back(plant_w_spr_.GetBodyByName(body_name).index());
  body_index_wo_spr_.push_back(plant_wo_spr_.GetBodyByName(body_name).index());
  frame_pose_.push_back(frame_pose);
}

void RotTaskSpaceTrackingData::AddStateAndFrameToTrack(
    int state, const std::string& body_name, const Isometry3d& frame_pose) {
  AddState(state);
  AddFrameToTrack(body_name, frame_pose);
}

void RotTaskSpaceTrackingData::UpdateYAndError(
    const VectorXd& x_w_spr, const Context<double>& context_w_spr) {
  auto transform_mat = plant_w_spr_.EvalBodyPoseInWorld(
      context_w_spr,
      plant_w_spr_.get_body(body_index_w_spr_.at(GetStateIdx())));
  Quaterniond y_quat(transform_mat.rotation() *
                     frame_pose_.at(GetStateIdx()).linear());
  Eigen::Vector4d y_4d;
  y_4d << y_quat.w(), y_quat.vec();
  y_ = y_4d;
  DRAKE_DEMAND(y_des_.size() == 4);
  Quaterniond y_quat_des(y_des_(0), y_des_(1), y_des_(2), y_des_(3));
  y_quat_des.normalize();

  // Get relative quaternion (from current to desired)
  Quaterniond relative_qaut = (y_quat_des * y_quat.inverse()).normalized();
  double theta = 2 * acos(relative_qaut.w());
  Vector3d rot_axis = relative_qaut.vec().normalized();

  error_y_ = theta * rot_axis;
}

void RotTaskSpaceTrackingData::UpdateYdotAndError(
    const VectorXd& x_w_spr, const Context<double>& context_w_spr) {
  MatrixXd J_spatial(6, plant_w_spr_.num_velocities());
  plant_w_spr_.CalcJacobianSpatialVelocity(
      context_w_spr, JacobianWrtVariable::kV,
      *body_frames_w_spr_.at(GetStateIdx()),
      frame_pose_.at(GetStateIdx()).translation(), world_w_spr_, world_w_spr_,
      &J_spatial);
  ydot_ = J_spatial.block(0, 0, kSpaceDim, J_spatial.cols()) *
          x_w_spr.tail(plant_w_spr_.num_velocities());
  // Transform qdot to w
  Quaterniond y_quat_des(y_des_(0), y_des_(1), y_des_(2), y_des_(3));
  Quaterniond dy_quat_des(ydot_des_(0), ydot_des_(1), ydot_des_(2),
                          ydot_des_(3));
  Vector3d w_des_ = 2 * (dy_quat_des * y_quat_des.conjugate()).vec();
  error_ydot_ = w_des_ - ydot_;
}

void RotTaskSpaceTrackingData::UpdateYddotDes() {
  // Convert ddq into angular acceleration
  // See https://physics.stackexchange.com/q/460311
  Quaterniond y_quat_des(y_des_(0), y_des_(1), y_des_(2), y_des_(3));
  Quaterniond yddot_quat_des(yddot_des_(0), yddot_des_(1), yddot_des_(2),
                             yddot_des_(3));
  yddot_des_converted_ = 2 * (yddot_quat_des * y_quat_des.conjugate()).vec();
}

void RotTaskSpaceTrackingData::UpdateJ(const VectorXd& x_wo_spr,
                                       const Context<double>& context_wo_spr) {
  MatrixXd J_spatial(6, plant_wo_spr_.num_velocities());
  plant_wo_spr_.CalcJacobianSpatialVelocity(
      context_wo_spr, JacobianWrtVariable::kV,
      *body_frames_wo_spr_.at(GetStateIdx()),
      frame_pose_.at(GetStateIdx()).translation(), world_wo_spr_, world_wo_spr_,
      &J_spatial);
  J_ = J_spatial.block(0, 0, kSpaceDim, J_spatial.cols());
}

void RotTaskSpaceTrackingData::UpdateJdotV(
    const VectorXd& x_wo_spr, const Context<double>& context_wo_spr) {
  JdotV_ = plant_wo_spr_
               .CalcBiasSpatialAcceleration(
                   context_wo_spr, JacobianWrtVariable::kV,
                   *body_frames_wo_spr_.at(GetStateIdx()),
                   frame_pose_.at(GetStateIdx()).translation(), world_wo_spr_,
                   world_wo_spr_)
               .rotational();
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

/**** JointSpaceTrackingData ****/
JointSpaceTrackingData::JointSpaceTrackingData(
    const string& name, const MatrixXd& K_p, const MatrixXd& K_d,
    const MatrixXd& W, const MultibodyPlant<double>& plant_w_spr,
    const MultibodyPlant<double>& plant_wo_spr)
    : OscTrackingData(name, 1, 1, K_p, K_d, W, plant_w_spr, plant_wo_spr) {}

void JointSpaceTrackingData::AddJointToTrack(
    const std::string& joint_pos_name, const std::string& joint_vel_name) {
  joint_pos_idx_w_spr_.push_back(
      makeNameToPositionsMap(plant_w_spr_).at(joint_pos_name));
  joint_vel_idx_w_spr_.push_back(
      makeNameToVelocitiesMap(plant_w_spr_).at(joint_vel_name));
  joint_pos_idx_wo_spr_.push_back(
      makeNameToPositionsMap(plant_wo_spr_).at(joint_pos_name));
  joint_vel_idx_wo_spr_.push_back(
      makeNameToVelocitiesMap(plant_wo_spr_).at(joint_vel_name));
}

void JointSpaceTrackingData::AddStateAndJointToTrack(
    int state, const std::string& joint_pos_name,
    const std::string& joint_vel_name) {
  AddState(state);
  AddJointToTrack(joint_pos_name, joint_vel_name);
}

void JointSpaceTrackingData::UpdateYAndError(
    const VectorXd& x_w_spr, const Context<double>& context_w_spr) {
  y_ = x_w_spr.segment(joint_pos_idx_w_spr_.at(GetStateIdx()), 1);
  error_y_ = y_des_ - y_;
}

void JointSpaceTrackingData::UpdateYdotAndError(
    const VectorXd& x_w_spr, const Context<double>& context_w_spr) {
  MatrixXd J = MatrixXd::Zero(1, plant_w_spr_.num_velocities());
  J(0, joint_vel_idx_w_spr_.at(GetStateIdx())) = 1;
  ydot_ = J * x_w_spr.tail(plant_w_spr_.num_velocities());
  error_ydot_ = ydot_des_ - ydot_;
}

void JointSpaceTrackingData::UpdateYddotDes() {
  yddot_des_converted_ = yddot_des_;
}

void JointSpaceTrackingData::UpdateJ(const VectorXd& x_wo_spr,
                                     const Context<double>& context_wo_spr) {
  J_ = MatrixXd::Zero(1, plant_wo_spr_.num_velocities());
  J_(0, joint_vel_idx_wo_spr_.at(GetStateIdx())) = 1;
}

void JointSpaceTrackingData::UpdateJdotV(
    const VectorXd& x_wo_spr, const Context<double>& context_wo_spr) {
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

}  // namespace dairlib::systems::controllers
