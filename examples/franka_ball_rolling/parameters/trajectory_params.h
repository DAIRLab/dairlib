#pragma once

#include "drake/common/yaml/yaml_read_archive.h"

struct BallRollingTrajectoryParams {
  int trajectory_type;
  double traj_radius;
  double x_c;
  double y_c;
  double init_phase;
  double lead_angle;
  double velocity_circle;
  double start_x;
  double start_y;
  double end_x;
  double end_y;
  double lead_step;
  double velocity_line;
  VectorXd q_init_ball;

  VectorXd traj_init = VectorXd::Zero(2);

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(trajectory_type));
    a->Visit(DRAKE_NVP(traj_radius));
    a->Visit(DRAKE_NVP(x_c));
    a->Visit(DRAKE_NVP(y_c));
    a->Visit(DRAKE_NVP(init_phase));
    a->Visit(DRAKE_NVP(lead_angle));
    a->Visit(DRAKE_NVP(velocity_circle));
    a->Visit(DRAKE_NVP(start_x));
    a->Visit(DRAKE_NVP(start_y));
    a->Visit(DRAKE_NVP(end_x));
    a->Visit(DRAKE_NVP(end_y));
    a->Visit(DRAKE_NVP(lead_step));
    a->Visit(DRAKE_NVP(velocity_line));
    a->Visit(DRAKE_NVP(q_init_ball));


    if (trajectory_type == 0 or trajectory_type == 1) {
      traj_init(0) = x_c + traj_radius * sin(M_PI * init_phase / 180.0);
      traj_init(1) = y_c + traj_radius * cos(M_PI * init_phase / 180.0);
    }
    if (trajectory_type == 2 or trajectory_type == 3) {
      traj_init(0) = start_x;
      traj_init(1) = start_y;
    }
  }
};