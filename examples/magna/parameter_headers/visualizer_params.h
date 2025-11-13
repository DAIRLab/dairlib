#pragma once

#include <Eigen/Dense>

#include "drake/common/yaml/yaml_read_archive.h"

struct MagnaVisualizerParams {
  double visualizer_publish_rate;
  Eigen::VectorXd camera_pose;
  Eigen::VectorXd camera_target;
  std::string belt_keypoint_model;
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(visualizer_publish_rate));
    a->Visit(DRAKE_NVP(camera_pose));
    a->Visit(DRAKE_NVP(camera_target));
    a->Visit(DRAKE_NVP(belt_keypoint_model));
  }
};
