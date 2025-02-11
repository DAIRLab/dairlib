#include "systems/perception/camera_utils.h"
#include "common/find_resource.h"

#include "drake/common/yaml/yaml_io.h"

// Original code by Hersh Sanghvi, Extended by Brian Acosta
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using drake::geometry::SceneGraph;
using drake::math::RigidTransform;
using drake::math::RigidTransformd;
using drake::math::RollPitchYaw;
using drake::math::RotationMatrix;
using drake::math::RigidTransformd;
using drake::systems::sensors::CameraInfo;

namespace dairlib {
namespace camera {

std::map<int, drake::systems::sensors::CameraInfo>
LoadRealsenseCalibrationsAsCameraInfo(const std::string& yaml_filename) {
  const auto archive =
      drake::yaml::LoadYamlFile<
          std::map<std::string, std::unordered_map<std::string, double>>>(
          yaml_filename);
  std::map<int, drake::systems::sensors::CameraInfo> camera_infos{};
  for (const std::pair<std::string,
       std::unordered_map<std::string, double>> entry: archive) {
    int i = stoi(entry.first);
    const std::unordered_map<std::string, double>& cam = entry.second;
    CameraInfo info = CameraInfo{((int)cam.at("width")),
                                 ((int)cam.at("height")),
                       cam.at("focal_x"),
                       cam.at("focal_y"),
                       cam.at("center_x"),
                       cam.at("center_y")};
    camera_infos.emplace(i, info);
  }
  return camera_infos;
}

RigidTransformd ReadCameraPoseFromYaml(const std::string& fname) {
  auto R_p =
      drake::yaml::LoadYamlFile<std::map<std::string, std::vector<double>>>(
          fname);
  DRAKE_ASSERT(R_p.count("translation") == 1);
  DRAKE_ASSERT(R_p.count("rotation") == 1);
  DRAKE_ASSERT(R_p.at("translation").size() == 3);
  DRAKE_ASSERT(R_p.at("rotation").size() == 9);

  Matrix3d R = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
      R_p.at("rotation").data());

  Vector3d t = Eigen::Map<Vector3d>(R_p.at("translation").data());
  return {RotationMatrix<double>::ProjectToRotationMatrix(R), t};
}

//  Based on https://github.com/RobotLocomotion/drake/blob/master/examples/manipulation_station/manipulation_station.cc
std::pair<drake::geometry::render::ColorRenderCamera,
          drake::geometry::render::DepthRenderCamera> MakeDairD455CameraModel(
    const std::string &renderer_name, D455ImageSize image_size) {
  // Intrinsics specific to the D455 realsense for use with Cassie
  auto camera_info = LoadRealsenseCalibrationsAsCameraInfo(
      FindResourceOrThrow("systems/perception/dair_d455.yaml")).at(image_size);

  drake::geometry::render::ColorRenderCamera color_camera{
      {renderer_name,
       camera_info,
       {0.1, 10.0},
       {},
      }, false};

  drake::geometry::render::DepthRenderCamera depth_camera{
      {renderer_name,
       camera_info,
       {0.6, 6.0},
       RigidTransformd(),
      }, {0.6, 6.0}};
  return {color_camera, depth_camera};
}

};
} // namespace dairlib::camera