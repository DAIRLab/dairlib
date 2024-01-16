#pragma once

#include "drake/common/yaml/yaml_read_archive.h"


struct FrankaSimParams {
  std::string franka_model;
  std::string end_effector_model;
  std::string table_model;
  std::string table_w_supports_model;
  std::string tray_model;
  std::string box_model;
  std::string left_support_model;
  std::string right_support_model;

  double dt;
  double realtime_rate;
  double actuator_delay;
  double franka_publish_rate;
  double tray_publish_rate;
  double visualizer_publish_rate;

  int scene_index;
  bool visualize_drake_sim;
  bool publish_efforts;

  Eigen::VectorXd camera_pose;
  Eigen::VectorXd camera_target;

  Eigen::VectorXd q_init_franka;
  std::vector<Eigen::VectorXd> q_init_plate;
  Eigen::VectorXd q_init_box;
  Eigen::VectorXd tool_attachment_frame;
  Eigen::VectorXd left_support_position;
  Eigen::VectorXd right_support_position;

  bool visualize_pose_trace;
  bool visualize_center_of_mass_plan;
  bool visualize_c3_forces;
  bool visualize_c3_state;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(franka_model));
    a->Visit(DRAKE_NVP(end_effector_model));
    a->Visit(DRAKE_NVP(table_model));
    a->Visit(DRAKE_NVP(table_w_supports_model));
    a->Visit(DRAKE_NVP(tray_model));
    a->Visit(DRAKE_NVP(box_model));
    a->Visit(DRAKE_NVP(left_support_model));
    a->Visit(DRAKE_NVP(right_support_model));

    a->Visit(DRAKE_NVP(dt));
    a->Visit(DRAKE_NVP(realtime_rate));
    a->Visit(DRAKE_NVP(actuator_delay));
    a->Visit(DRAKE_NVP(franka_publish_rate));
    a->Visit(DRAKE_NVP(tray_publish_rate));
    a->Visit(DRAKE_NVP(visualizer_publish_rate));

    a->Visit(DRAKE_NVP(scene_index));
    a->Visit(DRAKE_NVP(visualize_drake_sim));
    a->Visit(DRAKE_NVP(publish_efforts));

    a->Visit(DRAKE_NVP(camera_pose));
    a->Visit(DRAKE_NVP(camera_target));

    a->Visit(DRAKE_NVP(q_init_franka));
    a->Visit(DRAKE_NVP(q_init_plate));
    a->Visit(DRAKE_NVP(q_init_box));
    a->Visit(DRAKE_NVP(tool_attachment_frame));
    a->Visit(DRAKE_NVP(left_support_position));
    a->Visit(DRAKE_NVP(right_support_position));

    a->Visit(DRAKE_NVP(visualize_pose_trace));
    a->Visit(DRAKE_NVP(visualize_center_of_mass_plan));
    a->Visit(DRAKE_NVP(visualize_c3_forces));
    a->Visit(DRAKE_NVP(visualize_c3_state));
  }
};