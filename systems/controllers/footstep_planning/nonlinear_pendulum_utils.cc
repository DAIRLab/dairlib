#include "nonlinear_pendulum_utils.h"
#include "multibody/multibody_utils.h"

namespace dairlib::systems::controllers {

using alip_utils::PointOnFramed;
using multibody::ReExpressWorldVector3InBodyYawFrame;

using drake::multibody::MultibodyPlant;
using drake::systems::Context;

using Eigen::Vector3d;
using drake::Vector6d;

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
  double theta_x = -atan2(com_s.y(), com_s.z());

  Vector6d x;
  x(0) = theta_y;
  x(1) = theta_x;
  x(2) = r;
  x.segment<2>(3) = L_s.head<2>();
  x(5) = com_dot_s.dot(com_s) / com_s.norm();
  return x;
}



}