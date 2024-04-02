#pragma once

#include "drake/common/yaml/yaml_read_archive.h"

using Eigen::VectorXd;

struct HeuristicGaitParams {
  double roll_phase;
  double return_phase;
  VectorXd gait_parameters;
  int axis_option;
  double tilt_degrees;
  VectorXd q_new_vector;

  VectorXd initial_start;
  VectorXd initial_finish;
  double stabilize_time1;
  double move_time;
  double stabilize_time2;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(roll_phase));
    a->Visit(DRAKE_NVP(return_phase));
    a->Visit(DRAKE_NVP(gait_parameters));
    a->Visit(DRAKE_NVP(axis_option));
    a->Visit(DRAKE_NVP(tilt_degrees));
    a->Visit(DRAKE_NVP(q_new_vector));

    a->Visit(DRAKE_NVP(initial_start));
    a->Visit(DRAKE_NVP(initial_finish));
    a->Visit(DRAKE_NVP(stabilize_time1));
    a->Visit(DRAKE_NVP(move_time));
    a->Visit(DRAKE_NVP(stabilize_time2));
  }
};