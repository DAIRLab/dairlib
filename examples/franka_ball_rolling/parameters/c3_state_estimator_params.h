#pragma once

#include "drake/common/yaml/yaml_read_archive.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;

struct C3StateEstimatorParams {
  std::string franka_model;
  std::string end_effector_model;
  std::string ball_model;
  std::string ground_model;
  std::string offset_model;

  Vector3d tool_attachment_frame;
  Vector3d ground_offset_frame;
  Vector3d end_effector_offset;

  std::vector<double> p_FIR_value;
  std::vector<double> v_FIR_value;
  double ball_noise_stddev;
  double estimation_rate;

  VectorXd q_init_ball;
  double traj_radius;
  double phase;
  double x_c;
  double y_c;
  double ball_radius;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(franka_model));
    a->Visit(DRAKE_NVP(end_effector_model));
    a->Visit(DRAKE_NVP(ball_model));
    a->Visit(DRAKE_NVP(ground_model));
    a->Visit(DRAKE_NVP(offset_model));

    a->Visit(DRAKE_NVP(tool_attachment_frame));
    a->Visit(DRAKE_NVP(ground_offset_frame));
    a->Visit(DRAKE_NVP(end_effector_offset));

    a->Visit(DRAKE_NVP(p_FIR_value));
    a->Visit(DRAKE_NVP(v_FIR_value));
    a->Visit(DRAKE_NVP(ball_noise_stddev));
    a->Visit(DRAKE_NVP(estimation_rate));

    a->Visit(DRAKE_NVP(ground_offset_frame));
    a->Visit(DRAKE_NVP(q_init_ball));
    a->Visit(DRAKE_NVP(traj_radius));
    a->Visit(DRAKE_NVP(phase));
    a->Visit(DRAKE_NVP(x_c));
    a->Visit(DRAKE_NVP(y_c));
    a->Visit(DRAKE_NVP(ball_radius));
  }
};