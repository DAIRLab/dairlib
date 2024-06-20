#include "cassie_acom_tracking_data.h"
#include "cassie_acom_function.h"

#include <cmath>
#include <iostream>


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

double pitch = 0; //-0.2;

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

drake::VectorX<double> EvalQBaseAcom(const drake::VectorX<double>& q) {
  DRAKE_DEMAND(q.size() == 16);
  drake::VectorX<double> Q(4);
  Q(1) = getQx(q);
  Q(2) = getQy(q);
  Q(3) = getQz(q);
  // Q(0) = std::sqrt(1 - (Q(1) * Q(1) + Q(2) * Q(2) + Q(3) * Q(3)));
  Q(0) = std::sqrt(1 - (Q(1) * Q(1) + Q(2) * Q(2) + Q(3) * Q(3)));

  // Hack -- set offset here
  Quaterniond y_offset(cos(pitch / 2), 0 * sin(pitch / 2),
                       1 * sin(pitch / 2), 0 * sin(pitch / 2));
  Quaterniond y_current(Q(0),Q(1),Q(2),Q(3));
  y_current = y_current * y_offset;
  y_current.normalize();
  Q << y_current.w(), y_current.vec();
  // End of Hack

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

  // Hack -- set offset here
  Quaterniond y_offset(cos(pitch / 2), 0 * sin(pitch / 2),
                       1 * sin(pitch / 2), 0 * sin(pitch / 2));
  Quaterniond y_current(Q(0).value(),Q(1).value(),Q(2).value(),Q(3).value());
  y_current = y_current * y_offset;
  y_current.normalize();
  Q << y_current.w(), y_current.vec();
  // End of Hack

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

/**** CassieCassieAcomTrackingData ****/
CassieAcomTrackingData::CassieAcomTrackingData(const string& name, const MatrixXd& K_p,
                                   const MatrixXd& K_d, const MatrixXd& W,
                                   const MultibodyPlant<double>& plant)
    : OptionsTrackingData(name, kQuaternionDim, kSpaceDim, K_p, K_d, W, plant) {
  is_rotational_tracking_data_ = true;
}

void CassieAcomTrackingData::AddStateToTrack(
    int fsm_state, const Eigen::Isometry3d& frame_pose) {
  AddFiniteStateToTrack(fsm_state);
  frame_poses_[fsm_state] = frame_pose;
}

void CassieAcomTrackingData::UpdateY(
    const VectorXd& x, const Context<double>& context,
    OscTrackingDataState& tracking_data_state) const {

  Quaterniond Q_WB(x(0), x(1), x(2), x(3));
  VectorXd q_joint = x.segment<16>(7);
  VectorXd Q_BC_vec = EvalQBaseAcom(q_joint);
  Quaterniond Q_BC(Q_BC_vec(0), Q_BC_vec(1), Q_BC_vec(2), Q_BC_vec(3));
  Quaterniond Q_WC = (Q_WB * Q_BC).normalized();

  Eigen::Vector4d y_4d;
  y_4d << Q_WC.w(), Q_WC.vec();
  tracking_data_state.y_ = y_4d;
}

void CassieAcomTrackingData::UpdateYError(
    OscTrackingDataState& td_state) const {
  Quaterniond y_quat_des(td_state.y_des_(0), td_state.y_des_(1), td_state.y_des_(2), td_state.y_des_(3));
  Quaterniond y_quat(td_state.y_(0), td_state.y_(1), td_state.y_(2), td_state.y_(3));

  Eigen::AngleAxis<double> angle_axis_diff(y_quat_des * y_quat.inverse());
  td_state.error_y_ = angle_axis_diff.angle() * angle_axis_diff.axis();
  if (with_view_frame_) {
    td_state.error_y_ = td_state.view_frame_rot_T_ * td_state.error_y_;
  }
}

void CassieAcomTrackingData::UpdateYdot(
    const VectorXd& x, const Context<double>& context,
    OscTrackingDataState& td_state) const {
  VectorX<double> q = x.head<23>();
  MatrixXd J_acom = EvalJOmegaWorldAcomEwrtWorld(q);
  DRAKE_DEMAND(J_acom.rows() == 3);
  DRAKE_DEMAND(J_acom.cols() == plant_.num_velocities());
  td_state.ydot_ = J_acom * x.tail(plant_.num_velocities());
}

void CassieAcomTrackingData::UpdateYdotError(
    const Eigen::VectorXd& v_proj, OscTrackingDataState& td_state) const {
  // Transform qdot to w
  Quaterniond y_quat_des(td_state.y_des_(0), td_state.y_des_(1), td_state.y_des_(2), td_state.y_des_(3));
  Quaterniond dy_quat_des(td_state.ydot_des_(0), td_state.ydot_des_(1), td_state.ydot_des_(2),
                          td_state.ydot_des_(3));
  Vector3d w_des_ = 2 * (dy_quat_des * y_quat_des.conjugate()).vec();
  // Because we transform the error here rather than in the parent
  // options_tracking_data, and because J_y is already transformed in the view
  // frame, we need to undo the transformation on J_y
  td_state.error_ydot_ =
      w_des_ - td_state.ydot_ - td_state.view_frame_rot_T_.transpose() * td_state.J_ * v_proj;
  if (with_view_frame_) {
    td_state.error_ydot_ = td_state.view_frame_rot_T_ * td_state.error_ydot_;
  }

  td_state.ydot_des_ =
      w_des_;  // Overwrite 4d quat_dot with 3d omega. Need this for osc logging
}

void CassieAcomTrackingData::UpdateJ(
    const VectorXd& x, const Context<double>& context,
    OscTrackingDataState& td_state) const {
  VectorX<double> q = x.template head<23>();
  MatrixXd J_acom = EvalJOmegaWorldAcomEwrtWorld(q);
  td_state.J_ = J_acom;
}

void CassieAcomTrackingData::UpdateJdotV(
    const VectorXd& x, const Context<double>& context,
    OscTrackingDataState& td_state) const {
  td_state.JdotV_ = Vector3d::Zero(); //EvalJdotVOmegaWorldAcomEwrtWorld(x);
}

void CassieAcomTrackingData::UpdateYddotDes(
    double, double, OscTrackingDataState& td_state) const {
  // Convert ddq into angular acceleration
  // See https://physics.stackexchange.com/q/460311
  Quaterniond y_quat_des(
      td_state.y_des_(0), td_state.y_des_(1), td_state.y_des_(2), td_state.y_des_(3));
  Quaterniond yddot_quat_des(td_state.yddot_des_(0), td_state.yddot_des_(1), td_state.yddot_des_(2),
                             td_state.yddot_des_(3));
  td_state.yddot_des_converted_ = 2 * (yddot_quat_des * y_quat_des.conjugate()).vec();
  if (!idx_zero_feedforward_accel_.empty()) {
    std::cerr << "RotTaskSpaceTrackingData does not support zero feedforward "
                 "acceleration";
  }
  if (ff_accel_multiplier_traj_ != nullptr) {
    std::cerr
        << "RotTaskSpaceTrackingData does not support feedforward multipliers ";
  }
}

void CassieAcomTrackingData::CheckDerivedOscTrackingData() {}
}  // namespace dairlib::systems::controllers