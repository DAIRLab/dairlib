#pragma once

#include "drake/common/yaml/yaml_read_archive.h"

struct PlateBalancingTargetConfig {
  
  double near_target_threshold;
  std::vector<Eigen::Vector3d> first_target;
  std::vector<Eigen::Vector3d> second_target;
  std::vector<Eigen::Vector3d> third_target;
  double x_scale;
  double y_scale;
  double z_scale;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(first_target));
    a->Visit(DRAKE_NVP(second_target));
    a->Visit(DRAKE_NVP(third_target));
    a->Visit(DRAKE_NVP(x_scale));
    a->Visit(DRAKE_NVP(y_scale));
    a->Visit(DRAKE_NVP(z_scale));
    a->Visit(DRAKE_NVP(near_target_threshold));
  }
};