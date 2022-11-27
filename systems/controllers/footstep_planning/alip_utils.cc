#include <cmath>
#include <iostream>
#include "alip_utils.h"
#include "multibody/multibody_utils.h"

namespace dairlib::systems::controllers::alip_utils {

using drake::EigenPtr;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using Eigen::MatrixXd;
using Eigen::Matrix4d;
using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::Matrix;

void CalcAlipState(const MultibodyPlant<double>& plant, Context<double>* context,
                   const VectorXd &x,
                   const std::vector<PointOnFramed>& stance_feet,
                   const std::vector<double>& cop_weights,
                   const EigenPtr<Vector3d> &CoM_p,
                   const EigenPtr<Vector3d> &L_p,
                   const EigenPtr<Vector3d> &stance_pos_p) {

  multibody::SetPositionsAndVelocitiesIfNew<double>(plant, x, context);
  Vector3d CoM = plant.CalcCenterOfMassPositionInWorld(*context);

  // Take average of contact points a stance position
  Vector3d stance_foot_pos = Vector3d::Zero();
  for (int i = 0; i < stance_feet.size(); i++) {
    Vector3d position;
    const auto& stance_foot = stance_feet.at(i);
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

Matrix<double, 4, 8> CalcResetMap(double com_z, double m, double Tds) {
  MatrixXd A = CalcA(com_z, m);
  Matrix4d Ad = CalcAd(com_z, m, Tds);
  Matrix4d Adinv = Ad.inverse();
  Matrix4d Ainv = A.inverse();
  Matrix4d I = Matrix4d::Identity();
  Matrix<double, 4, 2> B = Matrix<double, 4, 2>::Zero();
  B(2,1) = m * 9.81;
  B(3,0) = -m * 9.81;
  Matrix<double, 4, 2> Bd =
      Ad * (-Ainv * Adinv + (1.0 / Tds) * Ainv * Ainv * (I - Adinv)) * B;
  Matrix<double, 4, 2> Bs = Matrix<double, 4, 2>::Zero();
  Bs.topRows(2) = -Eigen::Matrix2d::Identity();

  Matrix<double, 4, 8> Aeq = Matrix<double, 4, 8>::Zero();
  Aeq.topLeftCorner<4, 4>() = Ad;
  Aeq.block<4, 2>(0, 4) = -Bd - Bs;
  Aeq.block<4, 2>(0, 6) = Bd + Bs;
  return Aeq;
}

Vector4d CalcReset(double com_z, double m, double Tds,
                   const Vector4d& x, const Vector3d& p0, const Vector3d& p1) {
  Matrix<double, 4, 8> A = CalcResetMap(com_z, m, Tds);
  Vector4d xp =
      A.leftCols<4>() * x +
      A.block<4,2>(0,4) * p0.head<2>() + A.rightCols<2>() * p1.head<2>();
  return xp;
}

Matrix4d CalcA(double com_z, double m) {
  // Dynamics of ALIP: (eqn 6) https://arxiv.org/pdf/2109.14862.pdf
  const double g = 9.81;
  double a1x = 1.0 / (m * com_z);
  double a2x = -m * g;
  double a1y = -1.0 / (m * com_z);
  double a2y = m * g;

  MatrixXd A = Matrix4d::Zero();
  A(0, 3) = a1x;
  A(1, 2) = a1y;
  A(2, 1) = a2x;
  A(3, 0) = a2y;
  return A;
}

// Calculate analytical discretization of Alip Dynamics
Matrix4d CalcAd(double com_z, double m, double t) {
  double g = 9.81;
  double omega = sqrt(g/com_z);
  double d = 1.0 / (m * g);
  double a = sinh(t * omega);
  Matrix4d Ad = cosh(omega * t) * Vector4d::Ones().asDiagonal();
  Ad(0,3) = a * omega * d;
  Ad(1,2) = -a * omega * d;
  Ad(2,1) = -a / (omega * d);
  Ad(3,0) = a / (omega * d);
  return Ad;
}

double XImpactTime(double t_start, double H, double x, double Ly,
                   double x_impact) {
  return 0;
}

double YImpactTime(double t_start, double H, double y, double Lx,
                   double y_impact) {
  return 0;
}

}
