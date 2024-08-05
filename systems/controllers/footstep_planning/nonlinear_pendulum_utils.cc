#include "nonlinear_pendulum_utils.h"
#include "common/eigen_utils.h"
#include "multibody/multibody_utils.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/differentiable_norm.h"

#include <iostream>
namespace dairlib::systems::controllers::nonlinear_pendulum {

using alip_utils::PointOnFramed;
using multibody::ReExpressWorldVector3InBodyYawFrame;

using drake::multibody::MultibodyPlant;
using drake::systems::Context;

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector4d;
using Eigen::Vector3d;
using Eigen::Vector2d;
using drake::Vector6d;
using drake::Vector6;
using drake::Vector4;
using drake::Vector3;
using drake::Vector2;
using drake::Vector1;

using drake::AutoDiffXd;
using drake::AutoDiffVecXd;
using drake::math::InitializeAutoDiff;
using drake::math::ExtractGradient;
using drake::math::ExtractValue;

Vector6d CalcPendulumState(
    const MultibodyPlant<double>& plant, const Context<double>& context,
    PointOnFramed stance_foot,  std::string floating_base_body) {
  Vector3d stance_w;
  plant.CalcPointsPositions(
      context, stance_foot.second, stance_foot.first,
      plant.world_frame(), &stance_w);

  Vector3d com_w = plant.CalcCenterOfMassPositionInWorld(context);
  Vector3d com_dot_w = plant.CalcCenterOfMassTranslationalVelocityInWorld(context);
  Vector3d com_dot_s = ReExpressWorldVector3InBodyYawFrame(
      plant, context, floating_base_body, com_dot_w);

  Vector3d com_s = ReExpressWorldVector3InBodyYawFrame(
      plant, context, floating_base_body, com_w - stance_w);

  Vector3d L_w = plant.CalcSpatialMomentumInWorldAboutPoint(
      context, stance_w).rotational();

  Vector3d L_s = ReExpressWorldVector3InBodyYawFrame(
      plant, context, floating_base_body, L_w);

  double r = com_s.norm();
  double theta_y = atan2(com_s.x(), com_s.z());
  double theta_x = atan2(-com_s.y(), com_s.z());

  Vector6d x;
  x(theta_y_idx) = theta_y;
  x(theta_x_idx) = theta_x;
  x(r_idx) = r;
  x(l_y_idx) = L_s.y();
  x(l_x_idx) = L_s.x();
  x(rdot_idx) = com_dot_s.dot(com_s) / com_s.norm();
  return x;
}

void LinearizeTrapezoidalCollocationConstraint(
    double h, const Vector6d& x0, const Vector6d& x1, const Vector2d& u0,
    const Vector2d& u1, double m,
    MatrixXd& A, MatrixXd& B, VectorXd& b) {

  constexpr int nx = 6;
  constexpr int nu = 2;

  Eigen::VectorXd varsd0 = stack<double>({x0, u0});
  Eigen::VectorXd varsd1 = stack<double>({x1, u1});
  AutoDiffVecXd vars0 = InitializeAutoDiff(varsd0);
  AutoDiffVecXd vars1 = InitializeAutoDiff(varsd1);

  const Vector6<AutoDiffXd> x0_ad = vars0.head<nx>();
  const Vector6<AutoDiffXd> x1_ad = vars1.head<nx>();
  const Vector2<AutoDiffXd> u0_ad = vars0.tail<nu>();
  const Vector2<AutoDiffXd> u1_ad = vars1.tail<nu>();

  AutoDiffVecXd xdot0 = CalcPendulumDynamics(x0_ad, u0_ad, m);
  AutoDiffVecXd xdot1 = CalcPendulumDynamics(x1_ad, u1_ad, m);

  MatrixXd J0 = ExtractGradient(xdot0);
  MatrixXd J1 = ExtractGradient(xdot1);

  A = MatrixXd::Zero(nx, 2*nx);
  B = MatrixXd::Zero(nx, 2*nu);
  b = VectorXd::Zero(nx);

  MatrixXd A0 = J0.block<nx, nx>(0,0);
  MatrixXd A1 = J1.block<nx, nx>(0,0);
  MatrixXd B0 = J0.rightCols<nu>();
  MatrixXd B1 = J1.rightCols<nu>();

  A.block<nx, nx>(0,0) = Matrix<double, nx, nx>::Identity() + 0.5 * h * A0;
  A.block<nx, nx>(0, nx) = -Matrix<double, nx, nx>::Identity() + 0.5 * h * A1;

  B.block(0, 0, nx, nu) = 0.5 * h * B0;
  B.block(0, nu, nx, nu) = 0.5 * h * B0;

  b = 0.5 * h * (A0 * x0 + A1 * x1 + B0 * u0 + B1 * u1 - ExtractValue(xdot0) - ExtractValue(xdot1));
}

void LinearizeALIPReset(
    const Vector6d& x, const Vector3d& p_pre, const Vector3d& p_post, double m,
    MatrixXd& Ax, MatrixXd& Bp, Vector4d& b) {

  VectorX<AutoDiffXd> vars = InitializeAutoDiff(stack<double>({x, p_post}));
  Vector6<AutoDiffXd> x_ad = vars.head<6>();
  Vector3<AutoDiffXd> p_ad = vars.tail<3>();

  Vector4<AutoDiffXd> x_post = CalcALIPReset(x_ad, p_pre, p_ad, m);
  MatrixXd A = ExtractGradient(x_post);
  Ax = A.leftCols<6>();
  Bp = A.rightCols<3>();
  b = ExtractValue(x_post);
}

template <typename T>
Vector6<T> CalcPendulumDynamics(
    const drake::Vector6<T>& xp, const drake::Vector2<T>& u, double m) {
  T r = xp(r_idx);
  T theta_y = xp(theta_y_idx);
  T theta_x = xp(theta_x_idx);
  T x = r * cos(theta_x) * sin(theta_y);
  T y = -r * cos(theta_y) * sin(theta_x);

  double g = 9.81;

  Vector6<T> xdot;
  xdot(theta_y_idx) = xp(l_y_idx) / (m * r * r * cos(theta_x) * cos(theta_x));
  xdot(theta_x_idx) = xp(l_x_idx) / (m * r * r * cos(theta_y) * cos(theta_y));
  xdot(r_idx) = xp(rdot_idx);
  xdot(l_y_idx) = m * g * x + u(0);
  xdot(l_x_idx) = -m * g * y;
  xdot(rdot_idx) = u(1);
  return xdot;
}

template <typename T>
Vector4<T> CalcALIPReset(
    const Vector6<T>& x_pre, const Vector3d& p_pre, const Vector3<T>& p_post, double m) {
  const T& r = x_pre(r_idx);
  const T& theta_y = x_pre(theta_y_idx);
  const T& theta_x = x_pre(theta_x_idx);
  const T& rdot = x_pre(rdot_idx);
  const T theta_y_dot = x_pre(l_y_idx) / (m * r * r * cos(theta_x) * cos(theta_x));
  const T theta_x_dot = x_pre(l_x_idx) / (m * r * r * cos(theta_y) * cos(theta_y));
  const T x = r * cos(theta_x) * sin(theta_y);
  const T y = -r * cos(theta_y) * sin(theta_x);
  const T xdot = rdot * sin(theta_y) * cos(theta_x) + r * theta_y_dot * cos(theta_y) * cos(theta_x) - r * theta_x_dot * sin(theta_y) * sin(theta_x);
  const T ydot = -rdot * sin(theta_x) * cos(theta_y) - r * theta_x_dot * cos(theta_x) * cos(theta_y) + r * theta_y_dot * sin(theta_x) * sin(theta_y);
  const T zdot = (rdot * r - xdot*x - ydot * y) / sqrt(r*r - x*x - y*y);

  Vector3<T> v(xdot, ydot, zdot);

  Vector3<T> L_pre(x_pre(l_x_idx), x_pre(l_y_idx), 0.0);
  Vector3<T> u = p_pre - p_post;
  Vector3<T> L_post = L_pre + m * (u).cross(v);
  Vector4<T> x_post(x + u(0), y + u(1), L_post(0), L_post(1));
  return x_post;
}

namespace {

// TODO (@Brian-Acosta) leaving the plane equation un-normalized might cause
//  weird scaling issues
template<typename T>
Vector1<T> CalcDistanceToVirtualCoMPlane(
    const Vector6<T> &xp, const Vector3<T> &p_pre, const Vector3<T> &p_post) {
  Vector3<T> p = p_post - p_pre;
  T kx = p.x() * p.z() / (p.x() * p.x() + p.y() * p.y());
  T ky = p.y() * p.z() / (p.x() * p.x() + p.y() * p.y());
  T r = xp(r_idx);
  T theta_y = xp(theta_y_idx);
  T theta_x = xp(theta_x_idx);
  T x = r * cos(theta_x) * sin(theta_y);
  T y = -r * cos(theta_y) * sin(theta_x);
  T z = sqrt(r * r - x * x - y * y);

  Vector3<T> plane_eqn(kx, ky, -1);
  Vector3<T> com(x, y, z);

  return plane_eqn.transpose() * com / drake::math::DifferentiableNorm(plane_eqn);

//  return Vector1<T>(kx * x + ky * y - z);
}
template Vector1<AutoDiffXd> CalcDistanceToVirtualCoMPlane(const Vector6<AutoDiffXd>&, const Vector3<AutoDiffXd>&, const Vector3<AutoDiffXd>&);
}

void LinearizePlanarCoMEquation(
    const Vector6d& x, const Vector3d& p_pre,
    const Vector3d& p_post, double com_z, MatrixXd& JxJp, VectorXd& b) {

  VectorX<AutoDiffXd> vars = InitializeAutoDiff(stack<double>({x, p_pre, p_post}));
  Vector6<AutoDiffXd> x_ad = vars.head<6>();
  Vector3<AutoDiffXd> p_ad0 = vars.segment<3>(6);
  Vector3<AutoDiffXd> p_ad1 = vars.tail<3>();

  Vector1<AutoDiffXd> res = CalcDistanceToVirtualCoMPlane(x_ad, p_ad0, p_ad1);
  JxJp = ExtractGradient(res);
  b = ExtractValue(res) - Vector1<double>(com_z);
}

Vector4d CalcAlipStateAtTouchdown(const Vector6d& xp, double m, double t) {
  double r = xp(r_idx);
  double theta_y = xp(theta_y_idx);
  double theta_x = xp(theta_x_idx);
  double x = r * cos(theta_x) * sin(theta_y);
  double y = -r * cos(theta_y) * sin(theta_x);
  double ly = xp(l_y_idx);
  double lx = xp(l_x_idx);

  Vector4d alip_state(x, y, lx, ly);
  return alip_utils::CalcAd(r, m, t) * alip_state;
}

Vector6d PropogatePendulumState(const Vector6d& xp, double m, double t) {
  double r = xp(r_idx);
  double theta_y = xp(theta_y_idx);
  double theta_x = xp(theta_x_idx);
  double x = r * cos(theta_x) * sin(theta_y);
  double y = -r * cos(theta_y) * sin(theta_x);
  double z = sqrt(r*r - x*x - y*y);
  double ly = xp(l_y_idx);
  double lx = xp(l_x_idx);

  Vector4d alip_state(x, y, lx, ly);
  Vector4d xa = alip_utils::CalcAd(r, m, t) * alip_state;

  Vector3d com_s(xa(0), xa(1), z);
  r = com_s.norm();
  theta_y = atan2(com_s.x(), com_s.z());
  theta_x = atan2(-com_s.y(), com_s.z());

  double Lx = xa(2);
  double Ly = xa(3);
  Vector6d ret;
  ret(theta_x_idx) = theta_x;
  ret(theta_y_idx) = theta_y;
  ret(r_idx) = r;
  ret(l_x_idx) = Lx;
  ret(l_y_idx) = Ly;
  ret(rdot_idx) = 0;

  return ret;
}


template Vector4<AutoDiffXd> CalcALIPReset(const Vector6<AutoDiffXd>&, const Vector3d&, const Vector3<AutoDiffXd>&, double);
template Vector6<AutoDiffXd> CalcPendulumDynamics(const Vector6<AutoDiffXd>&, const Vector2<AutoDiffXd>&, double);
}