#include "multibody/view_frame.h"

using Eigen::MatrixXd;
using Eigen::Vector3d;

namespace dairlib::multibody {

template<typename T>
ViewFrame<T>::~ViewFrame(){};

/**** WorldYawViewFrame ****/
template <>
drake::Matrix3X<double> WorldYawViewFrame<double>::CalcWorldToFrameRotation(
    const drake::multibody::MultibodyPlant<double>& plant_w_spr,
    const drake::systems::Context<double>& context_w_spr) const {
  // Get approximated heading angle of pelvis and rotational matrix
  Vector3d body_x_axis =
      plant_w_spr.EvalBodyPoseInWorld(context_w_spr, body_).rotation().col(0);
  double approx_body_yaw = atan2(body_x_axis(1), body_x_axis(0));
  Eigen::MatrixXd rot(3, 3);
  rot << cos(approx_body_yaw), -sin(approx_body_yaw), 0, sin(approx_body_yaw),
      cos(approx_body_yaw), 0, 0, 0, 1;
  return rot.transpose();
}

/**** WorldYawViewFrame ****/
template <>
drake::Matrix3X<double> IdentityViewFrame<double>::CalcWorldToFrameRotation(
    const drake::multibody::MultibodyPlant<double>& plant_w_spr,
    const drake::systems::Context<double>& context_w_spr) const {
  return Eigen::MatrixXd::Identity(3, 3);
}

template ViewFrame<double>::~ViewFrame();

}  // namespace dairlib::multibody