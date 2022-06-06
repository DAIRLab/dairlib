#include "systems/cameras/camera_utils.h"

// Original code by Hersh Sanghvi, Extended by Brian Acosta

using Eigen::Vector3d;
using Eigen::VectorXd;
using drake::geometry::SceneGraph;
using drake::math::RigidTransform;
using drake::math::RigidTransformd;
using drake::math::RollPitchYaw;
using drake::math::RotationMatrix;

namespace dairlib {
namespace camera {

//  https://github.com/RobotLocomotion/drake/blob/master/examples/manipulation_station/manipulation_station.cc
std::pair <drake::geometry::render::ColorRenderCamera,
drake::geometry::render::DepthRenderCamera>
MakeD415CameraModel(const std::string &renderer_name) {
  // Typical D415 intrinsics for 848 x 480 resolution, note that rgb and
  // depth are slightly different (in both intrinsics and relative to the
  // camera body frame).
  // RGB:
  // - w: 848, h: 480, fx: 616.285, fy: 615.778, ppx: 405.418, ppy: 232.864
  // DEPTH:
  // - w: 848, h: 480, fx: 645.138, fy: 645.138, ppx: 420.789, ppy: 239.13
  const int kHeight = 480/2;
  const int kWidth = 848/2;

  // To pose the two sensors relative to the camera body, we'll assume X_BC = I,
  // and select a representative value for X_CD drawn from calibration to define
  // X_BD.
  drake::geometry::render::ColorRenderCamera color_camera{
      {renderer_name,
//             {kWidth, kHeight, 616.285, 615.778, 405.418, 232.864} /* intrinsics */,
       {kWidth,kHeight, 615.778/2, 405.418/2, 405.418, 232.864},
       {0.01, 10.0} /* clipping_range */,
       {} /* X_BC */},
      false};
  const RigidTransformd X_BD(
      RotationMatrix<double>(RollPitchYaw<double>(
          -0.19 * M_PI / 180, -0.016 * M_PI / 180, -0.03 * M_PI / 180)),
      Vector3d(0.015, -0.00019, -0.0001));
  drake::geometry::render::DepthRenderCamera depth_camera{
      {renderer_name,
       {kWidth, kHeight, 645.138/2, 645.138/2, 420.789, 239.13} /* intrinsics */,
       {0.01, 10.0} /* clipping_range */,
       X_BD},
      {0.1,   10.0} /* depth_range */};
  return {color_camera, depth_camera};
}

std::pair <drake::geometry::render::ColorRenderCamera,
drake::geometry::render::DepthRenderCamera>
MakeGenericCameraModel(const std::string &renderer_name, int kHeight, int kWidth) {
  // const int kHeight = 128;
  // const int kWidth = 128;

  // To pose the two sensors relative to the camera body, we'll assume X_BC = I,
  // and select a representative value for X_CD drawn from calibration to define
  // X_BD.
  const double center = kHeight/2;
  drake::geometry::render::ColorRenderCamera color_camera{
      {renderer_name,
       {kWidth, kHeight, 100, 100, center, center},
       {0.01, 10.0} /* clipping_range */,
       {} /* X_BC */},
      false};
  const RigidTransformd X_BD(
      RotationMatrix<double>(RollPitchYaw<double>(
          0.0, 0.0, 0.0)),
      Vector3d(0.015, -0.00019, -0.0001));
  drake::geometry::render::DepthRenderCamera depth_camera{
      {renderer_name,
       {kWidth, kHeight, 100, 100, center, center},
       {0.01, 10.0} /* clipping_range */,
       X_BD},
      {0.1,   10.0} /* depth_range */};
  return {color_camera, depth_camera};
}

std::pair <drake::geometry::render::ColorRenderCamera,
drake::geometry::render::DepthRenderCamera>
MakeWideCameraModel(const std::string &renderer_name) {
  const int kHeight = 128;
  const int kWidth = 128;

  // To pose the two sensors relative to the camera body, we'll assume X_BC = I,
  // and select a representative value for X_CD drawn from calibration to define
  // X_BD.
  drake::geometry::render::ColorRenderCamera color_camera{
      {renderer_name,
       {kWidth, kHeight, 50, 50, 64, 64},
       {0.01, 10.0} /* clipping_range */,
       {} /* X_BC */},
      false};
  const RigidTransformd X_BD(
      RotationMatrix<double>(RollPitchYaw<double>(
          0.0, 0.0, 0.0)),
      Vector3d(0.015, -0.00019, -0.0001));
  drake::geometry::render::DepthRenderCamera depth_camera{
      {renderer_name,
       {kWidth, kHeight, 50, 50, 64, 64},
       {0.01, 10.0} /* clipping_range */,
       X_BD},
      {0.1,   10.0} /* depth_range */};
  return {color_camera, depth_camera};
}

};
} // namespace dairlib::camera