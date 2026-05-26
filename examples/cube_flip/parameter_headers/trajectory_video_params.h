#pragma once

#include <Eigen/Dense>
#include "drake/common/yaml/yaml_read_archive.h"

struct TrajectoryVideoParams {
 std::string ee_file;
  std::string cube_file;

  double visualizer_publish_rate;

  double playback_speed;
  double dt;

  int ic3_iter;
  int trajectory_length;

  bool visualize_c3_traj;

  std::string trajectory_lcm_channel_x;
  std::string trajectory_lcm_channel_u;
  std::string trajectory_lcm_channel_c3;
  std::string trajectory_lcm_channel_x_real;
  std::string trajectory_lcm_channel_plate;

  std::vector<double> x_des;
  std::vector<double> x_init;

  Eigen::VectorXd trace_color;
  Eigen::VectorXd ee_trace_color;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(ee_file));
    a->Visit(DRAKE_NVP(cube_file));
    a->Visit(DRAKE_NVP(visualizer_publish_rate));
    a->Visit(DRAKE_NVP(ic3_iter));
    a->Visit(DRAKE_NVP(playback_speed));
    a->Visit(DRAKE_NVP(dt));
    a->Visit(DRAKE_NVP(visualize_c3_traj));
    a->Visit(DRAKE_NVP(trajectory_length));
    a->Visit(DRAKE_NVP(trajectory_lcm_channel_x));
    a->Visit(DRAKE_NVP(trajectory_lcm_channel_u));
    a->Visit(DRAKE_NVP(trajectory_lcm_channel_c3));
    a->Visit(DRAKE_NVP(trajectory_lcm_channel_x_real));
    a->Visit(DRAKE_NVP(trajectory_lcm_channel_plate));
    a->Visit(DRAKE_NVP(x_des));
    a->Visit(DRAKE_NVP(x_init));
    a->Visit(DRAKE_NVP(trace_color));
    a->Visit(DRAKE_NVP(ee_trace_color));

}
};
