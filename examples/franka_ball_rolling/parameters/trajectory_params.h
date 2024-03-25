#pragma once

// #include "solvers/c3_options.h"   Is this needed in this file?

#include "drake/common/yaml/yaml_read_archive.h"

struct SamplingC3TrajectoryParams {
  int trajectory_type;
  double traj_radius;
  double x_c;
  double y_c;
  double lead_angle;
  double time_increment_circle;
  double hold_order_circle;
  double start_x;
  double start_y;
  double end_x;
  double end_y;
  double lead_step;
  double time_increment_line;
  double hold_order_line;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(trajectory_type));
    a->Visit(DRAKE_NVP(traj_radius));
    a->Visit(DRAKE_NVP(x_c));
    a->Visit(DRAKE_NVP(y_c));
    a->Visit(DRAKE_NVP(lead_angle));
    a->Visit(DRAKE_NVP(time_increment_circle));
    a->Visit(DRAKE_NVP(hold_order_circle));
    a->Visit(DRAKE_NVP(start_x));
    a->Visit(DRAKE_NVP(start_y));
    a->Visit(DRAKE_NVP(end_x));
    a->Visit(DRAKE_NVP(end_y));
    a->Visit(DRAKE_NVP(lead_step));
    a->Visit(DRAKE_NVP(time_increment_line));
    a->Visit(DRAKE_NVP(hold_order_line));
  }
};