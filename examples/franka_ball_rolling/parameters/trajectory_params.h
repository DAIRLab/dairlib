#pragma once

// #include "solvers/c3_options.h"   Is this needed in this file?

#include "drake/common/yaml/yaml_read_archive.h"

struct BallRollingTrajectoryParams {
  int trajectory_type;
  double traj_radius;
  double x_c;
  double y_c;
  double lead_angle;
  double velocity_circle;
  double start_x;
  double start_y;
  double end_x;
  double end_y;
  double lead_step;
  double velocity_line;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(trajectory_type));
    a->Visit(DRAKE_NVP(traj_radius));
    a->Visit(DRAKE_NVP(x_c));
    a->Visit(DRAKE_NVP(y_c));
    a->Visit(DRAKE_NVP(lead_angle));
    a->Visit(DRAKE_NVP(velocity_circle));
    a->Visit(DRAKE_NVP(start_x));
    a->Visit(DRAKE_NVP(start_y));
    a->Visit(DRAKE_NVP(end_x));
    a->Visit(DRAKE_NVP(end_y));
    a->Visit(DRAKE_NVP(lead_step));
    a->Visit(DRAKE_NVP(velocity_line));
  }
};