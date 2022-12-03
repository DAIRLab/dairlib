#pragma once
#include "drake/common/eigen_types.h"
#include "drake/geometry/render/render_engine.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram.h"

// Original code by Hersh Sanghvi, Extended by Brian Acosta

namespace dairlib {
namespace camera {

enum D455ImageSize {
  k1920x1080 = 0,
  k1280x720 = 1,
  k640x480 = 2,
  k848x480 = 3,
  k640x360= 4,
  k424x240 = 5,
  k320x240 = 6,
  k480x270 = 7,
  k1280x800 = 8,
  k960x540 = 9,
  k640x400 = 12,
  k576x576 = 13,
  k720x720 = 14,
  k1152x1152 = 15
};

/// Returns a rotation matrix which will make a drake rgbd sensor "look"
/// parallel to the X-Z plane the parent frame, with angle "pitch" between the
/// positive X-Axis of the parent frame and the center of the image,
/// and the parent Y axis pointing left in the image.
drake::math::RotationMatrix<double> inline MakeXZAlignedCameraRotation(double pitch) {
  return drake::math::RotationMatrixd::MakeFromOrthonormalColumns(
      {0, -1, 0},
      {sin(pitch), 0, -cos(pitch)},
      {cos(pitch), 0, sin(pitch)}
      );
}

// Returns a rigid transform from a rotation matrix and translation vector
// defined in a Yaml file
drake::math::RigidTransformd ReadCameraPoseFromYaml(const std::string& fname);

std::map<int, drake::systems::sensors::CameraInfo>
    LoadRealsenseCalibrationsAsCameraInfo(const std::string& yaml_filename);

std::pair<drake::geometry::render::ColorRenderCamera,
drake::geometry::render::DepthRenderCamera> MakeD415CameraModel(
    const std::string &renderer_name);

std::pair<drake::geometry::render::ColorRenderCamera,
drake::geometry::render::DepthRenderCamera> MakeDairD455CameraModel(
    const std::string& renderer_name, D455ImageSize image_size);


std::pair<drake::geometry::render::ColorRenderCamera,
drake::geometry::render::DepthRenderCamera> MakeGenericCameraModel(
    const std::string &renderer_name, const int kHeight, const int kWidth);

std::pair <drake::geometry::render::ColorRenderCamera,
drake::geometry::render::DepthRenderCamera> MakeWideCameraModel(
    const std::string &renderer_name);

}
}

