#pragma once

// #include "solvers/c3_options.h"   Is this needed in this file?

#include "drake/common/yaml/yaml_read_archive.h"

struct SamplingC3ControllerParams {
  int trajectory_type;
  double traj_radius;
  double x_c;
  double y_c;
  double lead_angle;
  double fixed_goal_x;
  double fixed_goal_y;
  double step_size;
  double start_point_x;
  double start_point_y;
  double end_point_x;
  double end_point_y;
  double lookahead_step_size;
  double max_step_size;
  
  // Sampling parameters
  double ee_goal_height;
  double object_half_width;

  template <typename Archive>
  void Serialize(Archive* a) {
    // Trajectory parameters
    a->Visit(DRAKE_NVP(trajectory_type));
    a->Visit(DRAKE_NVP(traj_radius));
    a->Visit(DRAKE_NVP(x_c));
    a->Visit(DRAKE_NVP(y_c));
    a->Visit(DRAKE_NVP(lead_angle));
    a->Visit(DRAKE_NVP(fixed_goal_x));
    a->Visit(DRAKE_NVP(fixed_goal_y));
    a->Visit(DRAKE_NVP(step_size));
    a->Visit(DRAKE_NVP(start_point_x));
    a->Visit(DRAKE_NVP(start_point_y));
    a->Visit(DRAKE_NVP(end_point_x));
    a->Visit(DRAKE_NVP(end_point_y));
    a->Visit(DRAKE_NVP(lookahead_step_size));
    a->Visit(DRAKE_NVP(max_step_size));

    // Sampling parameters
    a->Visit(DRAKE_NVP(ee_goal_height));
    a->Visit(DRAKE_NVP(object_half_width));
  }
};