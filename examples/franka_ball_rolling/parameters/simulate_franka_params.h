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
  double visualizer_publish_rate;

  Vector3d tool_attachment_frame;
  Vector3d ground_offset_frame;

  VectorXd q_init_franka;
  VectorXd q_init_ball;

  double traj_radius;
  double phase;
  double x_c;
  double y_c;
  double ball_radius;
  double ee_radius;

  bool visualize_defualt_drake;
  bool visualize_pose_trace;
  bool visualize_plan;
  bool visualize_c3_forces;
  bool visualize_c3_object_state;
  bool visualize_c3_end_effector_state;

  Eigen::VectorXd camera_pose;
  Eigen::VectorXd camera_target;

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
    a->Visit(DRAKE_NVP(visualizer_publish_rate));

    a->Visit(DRAKE_NVP(tool_attachment_frame));
    a->Visit(DRAKE_NVP(ground_offset_frame));

    a->Visit(DRAKE_NVP(q_init_franka));
    a->Visit(DRAKE_NVP(q_init_ball));

    a->Visit(DRAKE_NVP(traj_radius));
    a->Visit(DRAKE_NVP(phase));
    a->Visit(DRAKE_NVP(x_c));
    a->Visit(DRAKE_NVP(y_c));
    a->Visit(DRAKE_NVP(ball_radius));
    a->Visit(DRAKE_NVP(ee_radius));

    a->Visit(DRAKE_NVP(visualize_defualt_drake));
    a->Visit(DRAKE_NVP(visualize_pose_trace));
    a->Visit(DRAKE_NVP(visualize_plan));
    a->Visit(DRAKE_NVP(visualize_c3_forces));
    a->Visit(DRAKE_NVP(visualize_c3_object_state));
    a->Visit(DRAKE_NVP(visualize_c3_end_effector_state));

    a->Visit(DRAKE_NVP(camera_pose));
    a->Visit(DRAKE_NVP(camera_target));

  }
};