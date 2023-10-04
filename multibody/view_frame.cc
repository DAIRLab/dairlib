#include "multibody/view_frame.h"

using Eigen::MatrixXd;
using Eigen::Matrix3d;
using Eigen::Vector3d;

namespace dairlib::multibody {

template<typename T>
ViewFrame<T>::~ViewFrame(){};

/**** WorldYawViewFrame ****/
template <>
drake::Matrix3X<double> WorldYawViewFrame<double>::CalcWorldToFrameRotation(
    const drake::multibody::MultibodyPlant<double>& plant,
    const drake::systems::Context<double>& context) const {
  // Get approximated heading angle of pelvis and rotational matrix
  Vector3d body_x_axis =
      plant.EvalBodyPoseInWorld(context, body_).rotation().col(0);
  double approx_body_yaw = atan2(body_x_axis(1), body_x_axis(0));
  Eigen::MatrixXd rot(3, 3);
  rot << cos(approx_body_yaw), -sin(approx_body_yaw), 0, sin(approx_body_yaw),
      cos(approx_body_yaw), 0, 0, 0, 1;
  return rot.transpose();
}

/**** RotationFromBodyViewFrame ****/
template<>
drake::Matrix3X<double> RotationFromBodyViewFrame<double>::CalcWorldToFrameRotation(
    const drake::multibody::MultibodyPlant<double>& plant,
    const drake::systems::Context<double>& context) const {
  auto R_WB = plant.EvalBodyPoseInWorld(context, body_).rotation();
  // R_FW = R_FB * R_BW = (R_WB * R_BF)^T
  return (R_WB.matrix() * R_BF_).transpose();
}

/**** IdentityViewFrame ****/
template <>
drake::Matrix3X<double> IdentityViewFrame<double>::CalcWorldToFrameRotation(
    const drake::multibody::MultibodyPlant<double>& plant,
    const drake::systems::Context<double>& context) const {
  return Eigen::MatrixXd::Identity(3, 3);
}

template ViewFrame<double>::~ViewFrame();

}  // namespace dairlib::multibody