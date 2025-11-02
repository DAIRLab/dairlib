#pragma once

#include <Eigen/Dense>
#include "drake/common/yaml/yaml_read_archive.h"

struct CubeFlipVisualizerParams {
  std::string plate_file;
  std::string cube_file;

  double visualizer_publish_rate;

  int ic3_num_iters;
  int trajectory_length;

  std::string trajectory_lcm_channel;

  std::vector<double> x_des;
  Eigen::VectorXd trace_color;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(plate_file));
    a->Visit(DRAKE_NVP(cube_file));
    a->Visit(DRAKE_NVP(visualizer_publish_rate));
    a->Visit(DRAKE_NVP(ic3_num_iters));
    a->Visit(DRAKE_NVP(trajectory_length));
    a->Visit(DRAKE_NVP(trajectory_lcm_channel));
    a->Visit(DRAKE_NVP(x_des));
    a->Visit(DRAKE_NVP(trace_color));
}
};
