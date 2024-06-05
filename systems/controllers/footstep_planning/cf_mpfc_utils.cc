#include "cf_mpfc_utils.h"

namespace dairlib::systems::controllers::cf_mpfc_utils {

using drake::systems::Context;
using drake::math::RigidTransformd;
using drake::math::RotationMatrixd;
using drake::multibody::Frame;
using drake::multibody::MultibodyPlant;
using drake::multibody::SpatialInertia;
using drake::multibody::SpatialMomentum;

using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::VectorXd;


CentroidalState GetCentroidalState(
    const MultibodyPlant<double>& plant, const Context<double>& plant_context,
    const Frame<double>& floating_base_frame,
    const std::function<Matrix3d(const MultibodyPlant<double>&, const Context<double>&)>& acom_function,
    const alip_utils::PointOnFramed& stance_foot) {

  CentroidalState x = CentroidalState::Zero();

  // Floating base pose in the world frame
  RigidTransformd X_WP = floating_base_frame.CalcPoseInWorld(plant_context);
  const Vector3d& body_x = X_WP.rotation().matrix().col(0);
  double yaw = atan2(body_x(1), body_x(0));

  // Transforms coordinates to the world frame from yaw frame
  auto R_WY = RotationMatrixd::MakeZRotation(yaw);

  // Orientation: R_YA = R_YW * R_WA
  Matrix3d R_YA = R_WY.transpose() * acom_function(plant, plant_context);

  // COM position
  Vector3d CoM_w = plant.CalcCenterOfMassPositionInWorld(plant_context);
  Vector3d CoM_w_dot = plant.CalcCenterOfMassTranslationalVelocityInWorld(plant_context);

  Vector3d p_w;
  plant.CalcPointsPositions(
      plant_context,
      stance_foot.second,
      stance_foot.first,
      plant.world_frame(),
      &p_w
  );

  // Can get angular velocity of ACOM
  SpatialInertia<double> I = plant.CalcSpatialInertia(
      plant_context, floating_base_frame,
      plant.GetBodyIndices(drake::multibody::ModelInstanceIndex{0}));
  I.ReExpressInPlace(X_WP.rotation());
  I.ShiftInPlace(CoM_w - X_WP.translation());

  Vector3d omega_w = I.CalcRotationalInertia().CopyToFullMatrix3().inverse() *
      plant.CalcSpatialMomentumInWorldAboutPoint(plant_context, CoM_w).rotational();

  x.head<3>() = R_YA.col(0);
  x.segment<3>(3) = R_YA.col(1);
  x.segment<3>(6) = R_YA.col(2);
  x.segment<3>(9) = R_WY.transpose() * (CoM_w - p_w);
  x.segment<3>(12) = R_WY.transpose() * omega_w;
  x.segment<3>(15);
  return x;
}


}

