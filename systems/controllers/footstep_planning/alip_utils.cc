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

void CalcAlipState(const MultibodyPlant<double>& plant, Context<double>* context,
                   const VectorXd &x,
                   const std::vector<PointOnFramed> stance_feet,
                   const EigenPtr<Vector3d> &CoM_p,
                   const EigenPtr<Vector3d> &L_p,
                   const EigenPtr<Vector3d> &stance_pos_p) {

  multibody::SetPositionsAndVelocitiesIfNew<double>(plant, x, context);
  Vector3d CoM = plant.CalcCenterOfMassPositionInWorld(*context);

  // Take average of contact points a stance position
  Vector3d stance_foot_pos = Vector3d::Zero();
  for (const auto& stance_foot : stance_feet) {
    Vector3d position;
    plant.CalcPointsPositions(*context, stance_foot.second, stance_foot.first,
                               plant.world_frame(), &position);
    stance_foot_pos += position;
  }
  stance_foot_pos /= stance_feet.size();

  Vector3d L = plant.CalcSpatialMomentumInWorldAboutPoint(
      *context, stance_foot_pos).rotational();

  *CoM_p = CoM;
  *L_p = L;
  *stance_pos_p = stance_foot_pos;
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

}
