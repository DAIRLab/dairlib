#include "acom_tracking_data.h"

#include <cmath>
#include <iostream>

#include "cassie_acom_func.h"

#include "drake/math/autodiff_gradient.h"

using Eigen::Isometry3d;
using Eigen::MatrixXd;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::VectorXd;

using std::cout;
using std::endl;
using std::string;
using std::vector;

using drake::multibody::JacobianWrtVariable;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;

using drake::AutoDiffVecXd;
using drake::math::ExtractGradient;
using drake::math::ExtractValue;
using drake::math::InitializeAutoDiff;

namespace dairlib::systems::controllers {

Eigen::MatrixXd MapWToQuatDot(const Eigen::Vector4d& Q) {
  // clang-format off
  Eigen::MatrixXd ret(4,3);
  ret <<  -Q(1), -Q(2), -Q(3),
           Q(0),  Q(3), -Q(2),
          -Q(3),  Q(0),  Q(1),
           Q(2), -Q(1),  Q(0);
  ret *= 0.5;
  // clang-format on
  return ret;
}

template <typename T>
drake::MatrixX<T> E_from_Quat(const drake::VectorX<T>& Q) {
  // Given Q = quaternion of frame B r.t. frame A,
  // find the 4x4 E transform (quat rates to omega)
  //
  // i.e.
  //      Given: Q_dot = 4x1 of quat rates
  //             omega = 3x1 ang vel of frame B r.t. frame A, ewrt frame B
  //      Then:
  //             2*E*Q_dot = [0; omega]
  // clang-format off
  drake::MatrixX<T> E(3,4);
  E << -Q(1),  Q(0),  Q(3), -Q(2),
       -Q(2), -Q(3),  Q(0),  Q(1),
       -Q(3),  Q(2), -Q(1),  Q(0);
  // clang-format on
  return E;
}

template <typename T>
drake::MatrixX<T> R_from_Quat(const drake::VectorX<T>& Q) {
  // Given Q = quaternion of frame B r.t. frame A,
  // find the coord transformation C_AB
  drake::MatrixX<T> R(3, 3);

  // clang-format off
  R << 2 * (Q(0) * Q(0) + Q(1) * Q(1)) - 1,
       2 * (Q(1) * Q(2) - Q(0) * Q(3)),
       2 * (Q(1) * Q(3) + Q(0) * Q(2)),
       2 * (Q(1) * Q(2) + Q(0) * Q(3)),
       2 * (Q(0) * Q(0) + Q(2) * Q(2)) - 1,
       2 * (Q(2) * Q(3) - Q(0) * Q(1)),
       2 * (Q(1) * Q(3) - Q(0) * Q(2)),
       2 * (Q(2) * Q(3) + Q(0) * Q(1)),
       2 * (Q(0) * Q(0) + Q(3) * Q(3)) - 1;
  // clang-format on
  return R;
}

// template <typename T>
// drake::VectorX<T> EvalQBaseAcom(const drake::VectorX<T>& q) {
//  DRAKE_DEMAND(q.size() == 16);
//  drake::VectorX<T> Q(4);
//  Q(1) = getQx(q);
//  Q(2) = getQy(q);
//  Q(3) = getQz(q);
//  //Q(0) = std::sqrt(1 - (Q(1) * Q(1) + Q(2) * Q(2) + Q(3) * Q(3)));
//  Q(0) = std::sqrt(1 - (Q(1) * Q(1) + Q(2) * Q(2) + Q(3) * Q(3)));
//  return Q;
//};
drake::VectorX<double> EvalQBaseAcom(const drake::VectorX<double>& q) {
  DRAKE_DEMAND(q.size() == 16);
  drake::VectorX<double> Q(4);
  Q(1) = getQx(q);
  Q(2) = getQy(q);
  Q(3) = getQz(q);
  // Q(0) = std::sqrt(1 - (Q(1) * Q(1) + Q(2) * Q(2) + Q(3) * Q(3)));
  Q(0) = std::sqrt(1 - (Q(1) * Q(1) + Q(2) * Q(2) + Q(3) * Q(3)));
  return Q;
};
drake::VectorX<AutoDiffXd> EvalQBaseAcom(const drake::VectorX<AutoDiffXd>& q) {
  DRAKE_DEMAND(q.size() == 16);
  drake::VectorX<AutoDiffXd> Q(4);
  Q(1) = getQx(q);
  Q(2) = getQy(q);
  Q(3) = getQz(q);

  // Not sure why we cannot do std::sqrt() operation, so I just do it manually
  // below
  //   Q(0) = std::sqrt(1 - (Q(1) * Q(1) + Q(2) * Q(2) + Q(3) * Q(3)));
  AutoDiffXd x = 1 - (Q(1) * Q(1) + Q(2) * Q(2) + Q(3) * Q(3));
  Q(0).value() = std::sqrt(x.value());
  Q(0).derivatives() = x.derivatives() / (2 * Q(0).value());

  // cout << "ExtractGradient(Q) = \n" << ExtractGradient(Q) << endl;
  return Q;
};

template <typename T>
drake::MatrixX<T> EvalJOmegaBaseAcomEwrtAcom(const drake::VectorX<T>& q) {
  DRAKE_DEMAND(q.size() == 16);

  drake::VectorX<T> Q = EvalQBaseAcom(q);

  drake::MatrixX<T> JQBaseAcomEwrtAcom = drake::MatrixX<T>::Zero(4, 16);
  getJQx(q, JQBaseAcomEwrtAcom);
  getJQy(q, JQBaseAcomEwrtAcom);
  getJQz(q, JQBaseAcomEwrtAcom);
  JQBaseAcomEwrtAcom.row(0) = -Q.template tail<3>().transpose() *
                              JQBaseAcomEwrtAcom.block(1, 0, 3, 16) / Q(0);

  drake::MatrixX<T> JOmegaBaseAcomEwrtAcom(3, 16);
  JOmegaBaseAcomEwrtAcom = 2 * E_from_Quat(Q) * JQBaseAcomEwrtAcom;

  return JOmegaBaseAcomEwrtAcom;
};

template <typename T>
drake::MatrixX<T> EvalJOmegaWorldAcomEwrtWorld(const drake::VectorX<T>& q) {
  DRAKE_DEMAND(q.size() == 23);
  drake::VectorX<T> Q_base = q.template head<4>();
  drake::MatrixX<T> R_WB = R_from_Quat(Q_base);

  drake::VectorX<T> q_joint = q.template tail<16>();
  drake::VectorX<T> Q_BC = EvalQBaseAcom(q_joint);
  drake::MatrixX<T> R_BC = R_from_Quat(Q_BC);

  drake::MatrixX<T> JOmegaWorldAcomEwrtWorld = drake::MatrixX<T>::Zero(3, 22);
  JOmegaWorldAcomEwrtWorld(0, 0) = 1;
  JOmegaWorldAcomEwrtWorld(1, 1) = 1;
  JOmegaWorldAcomEwrtWorld(2, 2) = 1;
  JOmegaWorldAcomEwrtWorld.template rightCols<16>() =
      R_WB * R_BC * EvalJOmegaBaseAcomEwrtAcom(q_joint);

  return JOmegaWorldAcomEwrtWorld;
};

template drake::MatrixX<double> E_from_Quat(const drake::VectorX<double>& Q);
template drake::MatrixX<AutoDiffXd> E_from_Quat(
    const drake::VectorX<AutoDiffXd>& Q);
template drake::MatrixX<double> R_from_Quat(const drake::VectorX<double>& Q);
template drake::MatrixX<AutoDiffXd> R_from_Quat(
    const drake::VectorX<AutoDiffXd>& Q);
// template drake::VectorX<double> EvalQBaseAcom(const drake::VectorX<double>&
// q); template drake::VectorX<AutoDiffXd> EvalQBaseAcom(const
// drake::VectorX<AutoDiffXd>& q);
template drake::MatrixX<double> EvalJOmegaBaseAcomEwrtAcom(
    const drake::VectorX<double>& q);
template drake::MatrixX<AutoDiffXd> EvalJOmegaBaseAcomEwrtAcom(
    const drake::VectorX<AutoDiffXd>& q);
template drake::MatrixX<double> EvalJOmegaWorldAcomEwrtWorld(
    const drake::VectorX<double>& q);
template drake::MatrixX<AutoDiffXd> EvalJOmegaWorldAcomEwrtWorld(
    const drake::VectorX<AutoDiffXd>& q);

//////

drake::VectorX<double> EvalJdotVOmegaWorldAcomEwrtWorld(
    const drake::VectorX<double>& x) {
  DRAKE_DEMAND(x.size() == 45);

  VectorXd q = x.head<23>();
  AutoDiffVecXd q_autodiff = InitializeAutoDiff(q);

  drake::MatrixX<AutoDiffXd> JOmegaWorldAcomEwrtWorld =
      EvalJOmegaWorldAcomEwrtWorld(q_autodiff);
  //  std::cout << "JOmegaWorldAcomEwrtWorld.rows() = \n" <<
  //  JOmegaWorldAcomEwrtWorld.rows() << std::endl; std::cout <<
  //  "JOmegaWorldAcomEwrtWorld.cols() = \n" << JOmegaWorldAcomEwrtWorld.cols()
  //  << std::endl; std::cout << "JOmegaWorldAcomEwrtWorld = \n" <<
  //  JOmegaWorldAcomEwrtWorld << std::endl;

  drake::MatrixX<double> JdotOmegaWorldAcomEwrtWorld =
      drake::MatrixX<double>::Zero(3, 22);
  for (int i = 6; i < 22; i++) {
    drake::MatrixX<double> JJ_wrt_qodt =
        ExtractGradient(JOmegaWorldAcomEwrtWorld.col(i));
    //    std::cout << "JJ_wrt_qodt = \n" << JJ_wrt_qodt << std::endl;

    drake::MatrixX<double> JJ_wrt_omega = drake::MatrixX<double>::Zero(3, 22);
    JJ_wrt_omega.leftCols<3>() =
        JJ_wrt_qodt.leftCols<4>() * MapWToQuatDot(x.head<4>());
    JJ_wrt_omega.rightCols<19>() = JJ_wrt_qodt.rightCols<19>();

    JdotOmegaWorldAcomEwrtWorld.col(i) = JJ_wrt_omega * x.tail<22>();
    //    cout << "i = " << i << endl << endl;
  }

  drake::VectorX<double> JdotVOmegaWorldAcomEwrtWorld =
      JdotOmegaWorldAcomEwrtWorld * x.tail<22>();
  return JdotVOmegaWorldAcomEwrtWorld;
};

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
  Quaterniond Q_WB(x_w_spr(0), x_w_spr(1), x_w_spr(2), x_w_spr(3));
  VectorXd q_joint = x_w_spr.segment<16>(7);
  VectorXd Q_BC_vec = EvalQBaseAcom(q_joint);
  Quaterniond Q_BC(Q_BC_vec(0), Q_BC_vec(1), Q_BC_vec(2), Q_BC_vec(3));
  Quaterniond Q_WC = (Q_WB * Q_BC).normalized();

  Eigen::Vector4d y_4d;
  y_4d << Q_WC.w(), Q_WC.vec();
  y_ = y_4d;
//  cout << "y_ = \n" << y_.transpose() << endl;
}

void AcomTrackingData::UpdateYError() {
  // Hack -- set the desired value here.
  Quaterniond y_des_quat(y_(0), 0, 0, y_(3));

  // Set desired pitch to non-zero
  // TODO: havne't checked if this is the right way of setting the desired orientation
  double pitch = 0.2;
  Quaterniond y_des_delta(cos(pitch / 2), 0 * sin(pitch / 2),
                          1 * sin(pitch / 2), 0 * sin(pitch / 2));
  y_des_quat = y_des_delta * y_des_quat;

  y_des_quat.normalize();
  y_des_ << y_des_quat.w(), y_des_quat.vec();
  // End of Hack

  DRAKE_DEMAND(y_des_.size() == 4);
  Quaterniond y_quat_des(y_des_(0), y_des_(1), y_des_(2), y_des_(3));
  y_quat_des.normalize();

  Quaterniond y_quat(y_(0), y_(1), y_(2), y_(3));

  // Get relative quaternion (from current to desired)
  Quaterniond relative_quat = (y_quat_des * y_quat.inverse()).normalized();
  double theta = 2 * acos(relative_quat.w());
  Vector3d rot_axis = relative_quat.vec().normalized();
  error_y_ = theta * rot_axis;
//  cout << "error_y_ = \n" << error_y_.transpose() << endl;
}

void AcomTrackingData::UpdateYdot(const VectorXd& x_w_spr,
                                  const Context<double>& context_w_spr) {
  VectorX<double> q = x_w_spr.head<23>();
  MatrixXd J_acom = EvalJOmegaWorldAcomEwrtWorld(q);
  DRAKE_DEMAND(J_acom.rows() == 3);
  DRAKE_DEMAND(J_acom.cols() == plant_w_spr_.num_velocities());
  ydot_ = J_acom * x_w_spr.tail(plant_w_spr_.num_velocities());
//  cout << "ydot_ = \n" << ydot_.transpose() << endl;
}

void AcomTrackingData::UpdateYdotError(const Eigen::VectorXd& v_proj) {
  //  ydot_des_.setZero();
  //  // Transform qdot to w
  //  Quaterniond y_quat_des(y_des_(0), y_des_(1), y_des_(2), y_des_(3));
  //  Quaterniond dy_quat_des(ydot_des_(0), ydot_des_(1), ydot_des_(2),
  //                          ydot_des_(3));
  //  Vector3d w_des_ = 2 * (dy_quat_des * y_quat_des.conjugate()).vec();

  Vector3d w_des_ = Vector3d::Zero();
  error_ydot_ = w_des_ - ydot_ - GetJ() * v_proj;

  ydot_des_ =
      w_des_;  // Overwrite 4d quat_dot with 3d omega. Need this for osc logging
//  cout << "ydot_des_ = \n" << ydot_des_.transpose() << endl;
}

void AcomTrackingData::UpdateJ(const VectorXd& x_wo_spr,
                               const Context<double>& context_wo_spr) {
//  auto start = std::chrono::high_resolution_clock::now();

  VectorX<double> q = x_wo_spr.template head<23>();
  MatrixXd J_acom = EvalJOmegaWorldAcomEwrtWorld(q);
  J_ = J_acom;
//  cout << "J_ = \n" << J_ << endl;

//  auto finish = std::chrono::high_resolution_clock::now();
//  std::chrono::duration<double> elapsed = finish - start;
//  cout << "J eval time:" << elapsed.count() << endl;

// <10us most of the time. It can hit 60us (but very rarely)
}

void AcomTrackingData::UpdateJdotV(const VectorXd& x_wo_spr,
                                   const Context<double>& context_wo_spr) {
//  auto start = std::chrono::high_resolution_clock::now();

  //JdotV_ = Vector3d::Zero();
  JdotV_ = EvalJdotVOmegaWorldAcomEwrtWorld(x_wo_spr);
//  cout << "JdotV_ = \n" << JdotV_.transpose() << endl;

//  auto finish = std::chrono::high_resolution_clock::now();
//  std::chrono::duration<double> elapsed = finish - start;
//  cout << "JdotV eval time:" << elapsed.count() << endl;

// evaluation time is ~3ms using autodiff.
}

void AcomTrackingData::UpdateYddotDes(double, double) {
  //  // Convert ddq into angular acceleration
  //  // See https://physics.stackexchange.com/q/460311
  //  Quaterniond y_quat_des(y_des_(0), y_des_(1), y_des_(2), y_des_(3));
  //  Quaterniond yddot_quat_des(yddot_des_(0), yddot_des_(1), yddot_des_(2),
  //                             yddot_des_(3));
  //  yddot_des_converted_ = 2 * (yddot_quat_des *
  //  y_quat_des.conjugate()).vec();

  // yddot_des_converted_.setZero();
  yddot_des_ = Vector3d::Zero();
  yddot_des_converted_ = Vector3d::Zero();

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
