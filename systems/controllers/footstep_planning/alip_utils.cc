#include <cmath>
#include <iostream>
#include "alip_utils.h"
#include "multibody/multibody_utils.h"
#include "drake/math/discrete_algebraic_riccati_equation.h"

namespace dairlib::systems::controllers::alip_utils {

using std::vector;
using std::pair;

using drake::EigenPtr;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;

using Eigen::MatrixXd;
using Eigen::Matrix4d;
using Eigen::Matrix2d;
using Eigen::VectorXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::Matrix;

void CalcAlipState(const MultibodyPlant<double> &plant,
                   Context<double> *context,
                   const VectorXd &x,
                   const vector<PointOnFramed> &stance_feet,
                   const vector<double> &cop_weights,
                   const EigenPtr<Vector3d> &CoM_p,
                   const EigenPtr<Vector3d> &L_p,
                   const EigenPtr<Vector3d> &stance_pos_p) {

  multibody::SetPositionsAndVelocitiesIfNew<double>(plant, x, context);
  Vector3d CoM = plant.CalcCenterOfMassPositionInWorld(*context);

  // Take average of contact points a stance position
  Vector3d stance_foot_pos = Vector3d::Zero();
  for (int i = 0; i < stance_feet.size(); i++) {
    Vector3d position;
    const auto &stance_foot = stance_feet.at(i);
    plant.CalcPointsPositions(*context, stance_foot.second, stance_foot.first,
                              plant.world_frame(), &position);
    stance_foot_pos += cop_weights.at(i) * position;
  }
  Vector3d L = plant.CalcSpatialMomentumInWorldAboutPoint(
      *context, stance_foot_pos).rotational();

  *CoM_p = CoM;
  *L_p = L;
  *stance_pos_p = stance_foot_pos;
}

namespace {

Matrix<double, 4, 2> CalcBdForResetMap(
    const Matrix4d& Ad, const Matrix4d& Ainv, const Matrix<double, 4, 2>& B,
    double Tds,
    ResetDiscretization discretization) {

  Matrix4d Adinv = Ad.inverse();
  Matrix4d I = Matrix4d::Identity();
  Matrix<double, 4, 2> Bd = Matrix<double, 4, 2>::Zero();

  switch (discretization) {
    case ResetDiscretization::kZOH:
      Bd = Ainv * (Ad - I) * B;
      break;
    case ResetDiscretization::kFOH:
      Bd = Ad * (-Ainv * Adinv + (1.0 / Tds) * Ainv * Ainv * (I - Adinv)) * B;
      break;
    case ResetDiscretization::kSPLIT:
      Matrix<double, 4, 2> Bd_ZOH = Ainv * (Ad - I) * B;
      Matrix<double, 4, 2> Bd_FOH =
          Ad * (-Ainv * Adinv + (1.0 / Tds) * Ainv * Ainv * (I - Adinv)) * B;
      Bd.row(0) = Bd_FOH.row(0);
      Bd.row(1) = Bd_ZOH.row(1);
      Bd.row(2) = Bd_ZOH.row(2);
      Bd.row(3) = Bd_FOH.row(3);
  }
  return Bd;
}

Matrix<double, 4, 8> AssembleResetMap(
    const Matrix4d& Ad, const Matrix<double, 4, 2>& Bd) {

  Matrix<double, 4, 2> Bfp = Matrix<double, 4, 2>::Zero();
  Bfp.topRows(2) = -Eigen::Matrix2d::Identity();

  Matrix<double, 4, 8> Aeq = Matrix<double, 4, 8>::Zero();
  Aeq.topLeftCorner<4, 4>() = Ad;
  Aeq.block<4, 2>(0, 4) = -Bd - Bfp;
  Aeq.block<4, 2>(0, 6) = Bd + Bfp;

  return Aeq;
}
}

Matrix<double, 4, 8> CalcResetMap(
    double com_z, double m, double Tds, ResetDiscretization discretization) {
  DRAKE_DEMAND(Tds > 0);
  MatrixXd A = CalcA(com_z, m);
  Matrix4d Ad = CalcAd(com_z, m, Tds);
  Matrix4d Ainv = A.inverse();
  Matrix<double, 4, 2> B = Matrix<double, 4, 2>::Zero();
  B(2, 1) = m * 9.81;
  B(3, 0) = -m * 9.81;
  Matrix<double, 4, 2> Bd = CalcBdForResetMap(Ad, Ainv, B, Tds, discretization);
  return AssembleResetMap(Ad, Bd);
}

Matrix<double, 4, 8> CalcMassNormalizedResetMap(
    double com_z, double Tds, ResetDiscretization discretization) {
  DRAKE_DEMAND(Tds > 0);
  MatrixXd A = CalcMassNormalizedA(com_z);
  Matrix4d Ad = CalcMassNormalizedAd(com_z, Tds);
  Matrix4d Adinv = Ad.inverse();
  Matrix4d Ainv = A.inverse();
  Matrix<double, 4, 2> B = Matrix<double, 4, 2>::Zero();
  B(2, 1) = 9.81;
  B(3, 0) = -9.81;
  Matrix<double, 4, 2> Bd = CalcBdForResetMap(Ad, Ainv, B, Tds, discretization);
  return AssembleResetMap(Ad, Bd);
}

pair<MatrixXd, MatrixXd> AlipStepToStepDynamics(
    double com_z, double m, double Tss, double Tds,
    ResetDiscretization discretization) {
  auto M = CalcResetMap(com_z, m, Tds, discretization);
  MatrixXd A = CalcAd(com_z, m, Tss + Tds);
  MatrixXd B = CalcAd(com_z, m, Tss) * M.rightCols<2>();
  return {A, B};
}

pair<MatrixXd, MatrixXd> MassNormalizedAlipStepToStepDynamics(
    double com_z, double Tss, double Tds,
    ResetDiscretization discretization) {
  auto M = CalcMassNormalizedResetMap(com_z, Tds, discretization);
  MatrixXd A = CalcMassNormalizedAd(com_z, Tss + Tds);
  MatrixXd B = CalcMassNormalizedAd(com_z, Tss) * M.rightCols<2>();
  return {A, B};
}

Vector4d CalcReset(double com_z, double m, double Tds,
                   const Vector4d &x, const Vector3d &p0, const Vector3d &p1,
                   ResetDiscretization reset_discretization) {
  Matrix<double, 4, 8> A = CalcResetMap(com_z, m, Tds, reset_discretization);
  Vector4d xp =
      A.leftCols<4>() * x +
          A.block<4, 2>(0, 4) * p0.head<2>() + A.rightCols<2>() * p1.head<2>();
  return xp;
}

Matrix4d CalcA(double com_z, double m) {
  // Dynamics of ALIP: (eqn 6) https://arxiv.org/pdf/2109.14862.pdf
  constexpr double g = 9.81;
  double dx_Ly = 1.0 / (m * com_z);
  double dy_Lx = -1.0 / (m * com_z);
  double dLx_y = -m * g;
  double dLy_x = m * g;

  MatrixXd A = Matrix4d::Zero();
  A(0, 3) = dx_Ly;
  A(1, 2) = dy_Lx;
  A(2, 1) = dLx_y;
  A(3, 0) = dLy_x;
  return A;
}

Matrix4d CalcMassNormalizedA(double com_z) {
  constexpr double g = 9.81;
  double dx_Ly = 1.0 / com_z;
  double dy_Lx = -1.0 / com_z;
  double dLx_y = -g;
  double dLy_x = g;

  MatrixXd A = Matrix4d::Zero();
  A(0, 3) = dx_Ly;
  A(1, 2) = dy_Lx;
  A(2, 1) = dLx_y;
  A(3, 0) = dLy_x;
  return A;
}

// Calculate analytical discretization of Alip Dynamics
Matrix4d CalcAd(double com_z, double m, double t) {
  constexpr double g = 9.81;
  double omega = sqrt(g / com_z);
  double d = 1.0 / (m * g);
  double a = sinh(t * omega);
  Matrix4d Ad = cosh(omega * t) * Matrix4d::Identity();
  Ad(0, 3) = a * omega * d;
  Ad(1, 2) = -a * omega * d;
  Ad(2, 1) = -a / (omega * d);
  Ad(3, 0) = a / (omega * d);
  return Ad;
}

Matrix4d CalcMassNormalizedAd(double com_z, double t) {
  constexpr double g = 9.81;
  double omega = sqrt(g / com_z);
  double d = 1.0 / g;
  double a = sinh(t * omega);
  Matrix4d Ad = cosh(omega * t) * Matrix4d::Identity();
  Ad(0, 3) = a * omega * d;
  Ad(1, 2) = -a * omega * d;
  Ad(2, 1) = -a / (omega * d);
  Ad(3, 0) = a / (omega * d);
  return Ad;
}

Vector4d CalcBd(double com_z, double m, double t) {
  return CalcA(com_z, m).inverse() *
      (CalcAd(com_z, m, t) - Matrix4d::Identity()) * Vector4d::UnitW();
}

std::pair<Vector4d, Vector4d> MakePeriodicAlipGait(
    const AlipGaitParams &gait_params) {

  double s = gait_params.initial_stance_foot == Stance::kLeft ? -1 : 1;

  const Vector2d u0 = Vector2d(
      gait_params.desired_velocity(0) * (gait_params.single_stance_duration +
          gait_params.double_stance_duration),
      gait_params.stance_width * s + gait_params.desired_velocity(1) *
          (gait_params.single_stance_duration
              + gait_params.double_stance_duration)
  );
  const Vector2d
      u1 = u0 - 2 * (s * gait_params.stance_width * Vector2d::UnitY());

  const auto [A, B] = AlipStepToStepDynamics(
      gait_params.height,
      gait_params.mass,
      gait_params.single_stance_duration,
      gait_params.double_stance_duration,
      gait_params.reset_discretization_method
  );

  // A^2 x0 + ABu0 + Bu1 = x0
  Vector4d x0 = (Matrix4d::Identity() - A * A).inverse() * (A * B * u0 + B * u1);
  Vector4d x1 = A*x0 + B* u0;

  return {x0, x1};
}


std::vector<Eigen::Vector2d> MakeP2Orbit(const AlipGaitParams& gait_params) {
  double s = gait_params.initial_stance_foot == Stance::kLeft ? -1 : 1;
  Vector2d u0 = Vector2d::Zero();
  u0(0) = gait_params.desired_velocity(0) * (
      gait_params.single_stance_duration +
          gait_params.double_stance_duration
  );
  u0(1) = gait_params.stance_width * s + gait_params.desired_velocity(1) * (
      gait_params.single_stance_duration +
          gait_params.double_stance_duration
  );
  Vector2d u1 = u0;
  u1(1) = - 2 * (s * gait_params.stance_width);
  return {u0, u1};
}

Eigen::Matrix<double, 2, 4>
CalcStateToCapturePointMatrix(const alip_utils::AlipGaitParams& gait_params) {
  Matrix<double, 2, 4> ret;
  double omega = sqrt(9.81 / gait_params.height);
  double xc = 1.0 / (gait_params.mass * gait_params.height * omega);
  ret.setZero();
  ret(0, 0) = 1;
  ret(0, 3) = xc;
  ret(1, 1) = 1;
  ret(1, 2) = -xc;
  return ret;
}

void MakeAlipStepToStepCostMatrices(
    const alip_utils::AlipGaitParams& gait_params,
    const Eigen::Matrix4d& Q, const Eigen::Matrix4d& Qf,
    Eigen::Matrix4d& Q_proj,
    Eigen::Matrix4d& Q_proj_f,
    Eigen::Matrix<double, 4, 2>& g_proj_p1,
    Eigen::Matrix<double, 4, 2>& g_proj_p2,
    Eigen::Matrix4d& p2o_premul,
    Eigen::Matrix4d& projection_to_p2o_complement,
    Eigen::Matrix<double, 4, 2>& p2o_orthogonal_complement,
    Eigen::Matrix<double, 4, 2>& p2o_basis) {

  const auto[A, B] = AlipStepToStepDynamics(
      gait_params.height,
      gait_params.mass,
      gait_params.single_stance_duration,
      gait_params.double_stance_duration,
      gait_params.reset_discretization_method
  );

  p2o_premul = (Matrix4d::Identity() - A * A).inverse();
  p2o_basis = p2o_premul * (A* B - B);
  p2o_orthogonal_complement = Eigen::FullPivLU<Eigen::Matrix<double, 2, 4>>(
      p2o_basis.transpose()
  ).kernel();

  Matrix4d projection_in_basis_coords = Matrix4d::Zero();
  projection_in_basis_coords.topLeftCorner<2,2>() = Matrix2d::Identity();

  Matrix4d bases_in_R4 = Matrix4d::Zero();
  bases_in_R4.leftCols<2>() = p2o_orthogonal_complement;
  bases_in_R4.rightCols<2>() = p2o_basis;

  projection_to_p2o_complement = bases_in_R4 * projection_in_basis_coords * bases_in_R4.inverse();
  Q_proj = projection_to_p2o_complement.transpose() * Q * projection_to_p2o_complement;
  Q_proj_f = projection_to_p2o_complement.transpose() * Qf * projection_to_p2o_complement;
  g_proj_p1 = p2o_premul * B;
  g_proj_p2 = A * p2o_premul * B;
}

}
