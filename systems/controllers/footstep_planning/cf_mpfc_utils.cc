#include <iostream>
#include "cf_mpfc_utils.h"
#include "common/eigen_utils.h"

namespace dairlib::systems::controllers::cf_mpfc_utils {

using drake::systems::Context;
using drake::AutoDiffXd;
using drake::math::InitializeAutoDiff;
using drake::math::ExtractGradient;
using drake::math::ExtractValue;
using drake::math::RigidTransformd;
using drake::math::RotationMatrixd;
using drake::multibody::Frame;
using drake::multibody::MultibodyPlant;
using drake::multibody::RotationalInertia;
using drake::multibody::SpatialInertia;
using drake::multibody::SpatialMomentum;
using drake::multibody::ModelInstanceIndex;

using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using drake::VectorX;
using drake::Vector6;
using drake::Vector4;
using drake::Vector3;


RotationalInertia<double> CalcRotationalInertiaAboutCoM(
    const MultibodyPlant<double>& plant,
    const Context<double>& plant_context,
    drake::multibody::ModelInstanceIndex model_instance,
    const drake::multibody::Frame<double>& floating_base_frame) {

  Vector3d CoM_w = plant.CalcCenterOfMassPositionInWorld(plant_context);
  RigidTransformd X_WP = floating_base_frame.CalcPoseInWorld(plant_context);

  // Can get angular velocity of ACOM
  SpatialInertia<double> I = plant.CalcSpatialInertia(
      plant_context, floating_base_frame,
      plant.GetBodyIndices(model_instance));

  I.ReExpressInPlace(X_WP.rotation());
  I.ShiftInPlace(CoM_w - X_WP.translation());

  return I.CalcRotationalInertia();
}

CentroidalState<double> GetCentroidalState(
    const MultibodyPlant<double>& plant, const Context<double>& plant_context,
    drake::multibody::ModelInstanceIndex model_instance,
    std::function<Matrix3d(const MultibodyPlant<double>&, const Context<double>&)> acom_function,
    const alip_utils::PointOnFramed& stance_foot) {

  // for floating base plants only
  DRAKE_DEMAND(plant.HasUniqueFreeBaseBody(model_instance));

  CentroidalState<double> x = CentroidalState<double>::Zero();

  // Floating base pose in the world frame
  const auto& floating_base_frame = plant.get_body(
      *(plant.GetFloatingBaseBodies().begin())).body_frame();

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

  Matrix3d I = CalcRotationalInertiaAboutCoM(
      plant, plant_context, model_instance, floating_base_frame
  ).CopyToFullMatrix3();

  Vector3d omega_w = I.inverse() * plant.CalcSpatialMomentumInWorldAboutPoint(
      plant_context, CoM_w).rotational();

  x.segment<3>(R_x_idx) = R_YA.col(0);
  x.segment<3>(R_y_idx) = R_YA.col(1);
  x.segment<3>(R_z_idx) = R_YA.col(2);
  x.segment<3>(com_idx) = R_WY.transpose() * (CoM_w - p_w);
  x.segment<3>(w_idx) = R_WY.transpose() * omega_w;
  x.segment<3>(com_dot_idx) = R_WY.transpose() * CoM_w_dot;
  return x;
}

template <typename T>
CentroidalStateDeriv<T> SRBDynamics(
    const CentroidalState<T>& state,
    const std::vector<Vector3<T>>& contact_points,
    const std::vector<Vector3<T>>& contact_forces,
    const Matrix3d& I, double m) {
  DRAKE_DEMAND(contact_points.size() == contact_forces.size());

  CentroidalStateDeriv<T> xdot = CentroidalStateDeriv<T>::Zero();

  const drake::Vector3<T> w = state.template segment<3>(w_idx);
  for (int i = 0; i < 3; ++i) {
    xdot.template segment<3>(3 * i) = w.cross(state.template segment<3>(3*i));
  }
  xdot.template segment<3>(com_idx) = state.template segment<3>(com_dot_idx);

  for (int i = 0; i < contact_points.size(); ++i) {
    drake::Vector3<T> r = contact_points[i] - state.template segment<3>(com_idx);
    xdot.template segment<3>(w_idx) += I.inverse() * r.cross(contact_forces[i]);
    xdot.template segment<3>(com_dot_idx) += (1.0 / m) * contact_forces[i];
  }
  xdot[17] -= 9.81;
  return xdot;
}

void LinearizeSRBDynamics(
    const CentroidalState<double>& x,
    const std::vector<Vector3d>& contact_points,
    const std::vector<Vector3d>& contact_forces,
    const Eigen::Matrix3d& I, double m,
    Eigen::MatrixXd& A, Eigen::MatrixXd& Bp, Eigen::MatrixXd&Bf, Eigen::VectorXd& c) {

  std::vector<Eigen::VectorXd> vars;
  vars.push_back(x);
  for (const auto& p : contact_points) {
    vars.push_back(p);
  }
  for (const auto & f : contact_forces) {
    vars.push_back(f);
  }

  VectorX<AutoDiffXd> vars_ad = InitializeAutoDiff(stack(vars));
  VectorX<AutoDiffXd> p_ad = vars_ad.segment(SrbDim, 3 * contact_points.size());
  VectorX<AutoDiffXd> f_ad = vars_ad.segment(
      SrbDim + 3 * contact_points.size(), 3 * contact_forces.size());

  CentroidalState<AutoDiffXd> x_ad = vars_ad.head<SrbDim>();

  CentroidalStateDeriv<AutoDiffXd> xdot_ad = SRBDynamics(
      x_ad, unstack<AutoDiffXd, 3>(p_ad), unstack<AutoDiffXd, 3>(f_ad), I, m);

  MatrixXd AB = ExtractGradient(xdot_ad);

  A = AB.leftCols<SrbDim>();
  Bp = AB.middleCols(SrbDim, p_ad.rows());
  Bf = AB.rightCols(f_ad.rows());
  c = ExtractValue(xdot_ad);
}

template <typename T>
Vector4<T> CalculateReset(const CentroidalState<T>& x_pre,
                          const Vector3<T>& p_pre, const Vector3<T>& p_post,
                          const Eigen::Matrix3d& I, double m) {
  Vector3<T> r = x_pre.template segment<3>(com_idx) + p_pre - p_post;

  // post impact 3d AM about new pivot point
  Vector3<T> L_post = I * x_pre.template segment<3>(w_idx) +
      m * r.cross(x_pre.template segment<3>(com_dot_idx));

  Vector4<T> x_post;
  x_post.template head<2>() = r.template head<2>();
  x_post.template tail<2>() = L_post.template head<2>();

  return x_post;
}

void LinearizeReset(
    const CentroidalState<double>& x_pre, const Vector3d& p_pre,
    const Vector3d& p_post, const Matrix3d& I, double m,
    Eigen::MatrixXd& Ax, Eigen::MatrixXd& B_post,
    Eigen::Vector4d& b) {

  VectorX<AutoDiffXd> vars = InitializeAutoDiff(
      stack<double>({x_pre, p_pre, p_post}));
  CentroidalState<AutoDiffXd> x_ad = vars.head<SrbDim>();
  Vector3<AutoDiffXd> p_pre_ad =  vars.segment<3>(SrbDim);
  Vector3<AutoDiffXd> p_post_ad =  vars.tail<3>();

  Vector4<AutoDiffXd> x_post = CalculateReset(x_ad, p_pre_ad, p_post_ad, I, m);

  Eigen::MatrixXd A = ExtractGradient(x_post);
  Ax = A.leftCols<SrbDim>();
  B_post = A.rightCols<3>();
  b = ExtractValue(x_post);
}

template CentroidalStateDeriv<double> SRBDynamics(const CentroidalState<double>&, const std::vector<Vector3d>&, const std::vector<Vector3d>&, const Matrix3d&, double);
template CentroidalStateDeriv<AutoDiffXd> SRBDynamics(const CentroidalState<AutoDiffXd>&, const std::vector<drake::Vector3<AutoDiffXd>>&, const std::vector<drake::Vector3<AutoDiffXd>>&, const Matrix3d&, double);
template Vector4<double> CalculateReset(const CentroidalState<double>& x_pre, const Vector3<double>& p_pre, const Vector3<double>& p_post, const Eigen::Matrix3d& I, double m);
template Vector4<AutoDiffXd> CalculateReset(const CentroidalState<AutoDiffXd>& x_pre, const Vector3<AutoDiffXd>& p_pre, const Vector3<AutoDiffXd>& p_post, const Eigen::Matrix3d& I, double m);


}

