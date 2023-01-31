#include "acom_tracking_data.h"

#include <iostream>

using Eigen::Isometry3d;
using Eigen::MatrixXd;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::VectorXd;

using std::string;
using std::vector;

using drake::multibody::JacobianWrtVariable;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;

namespace dairlib::systems::controllers {

/**** AcomTrackingData ****/
AcomTrackingData::AcomTrackingData(const string& name, const MatrixXd& K_p,
                                   const MatrixXd& K_d, const MatrixXd& W,
                                   const MultibodyPlant<double>& plant_w_spr,
                                   const MultibodyPlant<double>& plant_wo_spr)
    : OptionsTrackingData(name, kQuaternionDim, kSpaceDim, K_p, K_d, W,
                          plant_w_spr, plant_wo_spr) {}

// void AcomTrackingData::AddFrameToTrack(
//    const std::string& body_name, const Eigen::Isometry3d& frame_pose) {
//  AddStateAndFrameToTrack(-1, body_name, frame_pose);
//}

void AcomTrackingData::AddStateToTrack(int fsm_state,
                                       const Eigen::Isometry3d& frame_pose) {
  AddFiniteStateToTrack(fsm_state);
  frame_poses_[fsm_state] = frame_pose;
}

void AcomTrackingData::UpdateY(const VectorXd& x_w_spr,
                               const Context<double>& context_w_spr) {

  Quaterniond y_quat;
  Eigen::Vector4d y_4d;
  y_4d << y_quat.w(), y_quat.vec();
  y_ = y_4d;
}

void AcomTrackingData::UpdateYError() {
  DRAKE_DEMAND(y_des_.size() == 4);
  Quaterniond y_quat_des(y_des_(0), y_des_(1), y_des_(2), y_des_(3));
  y_quat_des.normalize();

  Quaterniond y_quat(y_(0), y_(1), y_(2), y_(3));

  // Get relative quaternion (from current to desired)
  Quaterniond relative_quat = (y_quat_des * y_quat.inverse()).normalized();
  double theta = 2 * acos(relative_quat.w());
  Vector3d rot_axis = relative_quat.vec().normalized();
  error_y_ = theta * rot_axis;
}

void AcomTrackingData::UpdateYdot(const VectorXd& x_w_spr,
                                  const Context<double>& context_w_spr) {
  MatrixXd J_acom(3, plant_w_spr_.num_velocities());
  MatrixXd J_floating_base = MatrixXd::Zero(kSpaceDim, kSpaceDim + kSpaceDim);
  MatrixXd J_joints = MatrixXd::Zero(
      kSpaceDim, plant_w_spr_.num_velocities() - kSpaceDim + kSpaceDim);
  J_floating_base.block(0, 0, kSpaceDim, kSpaceDim) =
      MatrixXd::Identity(kSpaceDim, kSpaceDim);
  J_acom << J_floating_base, J_joints;
  ydot_ = J_acom * x_w_spr.tail(plant_w_spr_.num_velocities());
}

void AcomTrackingData::UpdateYdotError(const Eigen::VectorXd& v_proj) {
  // Transform qdot to w
  Quaterniond y_quat_des(y_des_(0), y_des_(1), y_des_(2), y_des_(3));
  Quaterniond dy_quat_des(ydot_des_(0), ydot_des_(1), ydot_des_(2),
                          ydot_des_(3));
  Vector3d w_des_ = 2 * (dy_quat_des * y_quat_des.conjugate()).vec();
  error_ydot_ = w_des_ - ydot_ - GetJ() * v_proj;

  ydot_des_ =
      w_des_;  // Overwrite 4d quat_dot with 3d omega. Need this for osc logging
}

void AcomTrackingData::UpdateJ(const VectorXd& x_wo_spr,
                               const Context<double>& context_wo_spr) {
  MatrixXd J_acom(3, plant_wo_spr_.num_velocities());
  MatrixXd J_floating_base = MatrixXd::Zero(kSpaceDim, kSpaceDim + kSpaceDim);
  MatrixXd J_joints = MatrixXd::Zero(
      kSpaceDim, plant_wo_spr_.num_velocities() - kSpaceDim + kSpaceDim);
  J_floating_base.block(0, 0, kSpaceDim, kSpaceDim) =
      MatrixXd::Identity(kSpaceDim, kSpaceDim);
  J_acom << J_floating_base, J_joints;
  J_ = J_acom;
}

void AcomTrackingData::UpdateJdotV(const VectorXd& x_wo_spr,
                                   const Context<double>& context_wo_spr) {
  JdotV_ = VectorXd::Zero(kSpaceDim);
}

void AcomTrackingData::UpdateYddotDes(double, double) {
  // Convert ddq into angular acceleration
  // See https://physics.stackexchange.com/q/460311
  Quaterniond y_quat_des(y_des_(0), y_des_(1), y_des_(2), y_des_(3));
  Quaterniond yddot_quat_des(yddot_des_(0), yddot_des_(1), yddot_des_(2),
                             yddot_des_(3));
  yddot_des_converted_ = 2 * (yddot_quat_des * y_quat_des.conjugate()).vec();
  if (!idx_zero_feedforward_accel_.empty()) {
    std::cerr << "AcomTrackingData does not support zero feedforward "
                 "acceleration";
  }
  if (ff_accel_multiplier_traj_ != nullptr) {
    std::cerr << "AcomTrackingData does not support feedforward multipliers ";
  }
}

void AcomTrackingData::CheckDerivedOscTrackingData() {
  if (!body_frames_w_spr_.empty()) {
    body_frames_w_spr_ = body_frames_wo_spr_;
  }
}
}  // namespace dairlib::systems::controllers
