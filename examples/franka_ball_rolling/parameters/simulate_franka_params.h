#pragma once

#include "drake/common/yaml/yaml_read_archive.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;

struct SimulateFrankaParams {
  std::string franka_model;
  std::string end_effector_model;
  std::string ball_model;
  std::string ground_model;
  std::string offset_model;

  double sim_dt;
  double publish_dt;
  double realtime_rate;

  Vector3d tool_attachment_frame;
  Vector3d ground_offset_frame;

  VectorXd q_init_franka;
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

    a->Visit(DRAKE_NVP(sim_dt));
    a->Visit(DRAKE_NVP(publish_dt));
    a->Visit(DRAKE_NVP(realtime_rate));
    a->Visit(DRAKE_NVP(tool_attachment_frame));
    a->Visit(DRAKE_NVP(ground_offset_frame));

    a->Visit(DRAKE_NVP(q_init_franka));
    a->Visit(DRAKE_NVP(q_init_ball));

    a->Visit(DRAKE_NVP(traj_radius));
    a->Visit(DRAKE_NVP(phase));
    a->Visit(DRAKE_NVP(x_c));
    a->Visit(DRAKE_NVP(y_c));
    a->Visit(DRAKE_NVP(ball_radius));
  }
};