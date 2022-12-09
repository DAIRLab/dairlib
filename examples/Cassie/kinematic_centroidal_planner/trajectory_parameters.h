#pragma once

#include "yaml-cpp/yaml.h"

#include "drake/common/yaml/yaml_read_archive.h"

struct TrajectoryParameters {
  double n_knot_points;
  double target_com_height;
  double stance_width;
  double swing_foot_minimum_height;

  std::vector<double> duration_scaling;
  std::vector<std::string> gait_sequence;
  std::vector<double> com_vel_values;

  std::vector<Eigen::Vector3d> com_vel_vector;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(n_knot_points));
    a->Visit(DRAKE_NVP(target_com_height));
    a->Visit(DRAKE_NVP(stance_width));
    a->Visit(DRAKE_NVP(swing_foot_minimum_height));
    a->Visit(DRAKE_NVP(duration_scaling));
    a->Visit(DRAKE_NVP(gait_sequence));
    a->Visit(DRAKE_NVP(com_vel_values));
    Eigen::Map<Eigen::Matrix3Xd> input(com_vel_values.data(), 3,
                                       com_vel_values.size() / 3);
    for (int i = 0; i < input.cols(); ++i) {
      com_vel_vector.push_back(input.col(i));
    }
    DRAKE_DEMAND(duration_scaling.size() == gait_sequence.size());
  }
};