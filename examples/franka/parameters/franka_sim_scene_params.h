#pragma once

#include "drake/common/yaml/yaml_read_archive.h"

// Currently this scene only defines static environment obstacles
struct FrankaSimSceneParams {
  std::vector<std::string> environment_models;
  std::vector<Eigen::VectorXd> environment_orientations;
  std::vector<Eigen::VectorXd> environment_positions;

  std::vector<Eigen::VectorXd> camera_pose;
  std::vector<Eigen::VectorXd> camera_target;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(environment_models));
    a->Visit(DRAKE_NVP(environment_orientations));
    a->Visit(DRAKE_NVP(environment_positions));

    a->Visit(DRAKE_NVP(camera_pose));
    a->Visit(DRAKE_NVP(camera_target));
  }
};