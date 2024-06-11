#pragma once

#include "drake/common/yaml/yaml_read_archive.h"

struct FrankaSimParams {
  std::vector<std::string> sim_scene_file;
  std::string franka_model;
  std::string end_effector_model;
  std::string tray_model;
  std::string object_model;

  double dt;
  double realtime_rate;
  double actuator_delay;
  double franka_publish_rate;
  double tray_publish_rate;
  double object_publish_rate;
  double visualizer_publish_rate;

  int scene_index;
  bool visualize_drake_sim;
  bool publish_efforts;
  bool publish_object_velocities;

  Eigen::VectorXd q_init_franka;
  std::vector<Eigen::VectorXd> q_init_tray;
  std::vector<Eigen::VectorXd> q_init_object;
  Eigen::VectorXd tool_attachment_frame;

  std::vector<Eigen::VectorXd> world_x_limits;
  std::vector<Eigen::VectorXd> world_y_limits;
  std::vector<Eigen::VectorXd> world_z_limits;

  std::vector<double> external_force_scaling;

  bool visualize_pose_trace;
  bool visualize_center_of_mass_plan;
  bool visualize_c3_forces;
  bool visualize_c3_object_state;
  bool visualize_c3_end_effector_state;
  bool visualize_workspace;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(sim_scene_file));
    a->Visit(DRAKE_NVP(franka_model));
    a->Visit(DRAKE_NVP(end_effector_model));
    a->Visit(DRAKE_NVP(tray_model));
    a->Visit(DRAKE_NVP(object_model));

    a->Visit(DRAKE_NVP(dt));
    a->Visit(DRAKE_NVP(realtime_rate));
    a->Visit(DRAKE_NVP(actuator_delay));
    a->Visit(DRAKE_NVP(franka_publish_rate));
    a->Visit(DRAKE_NVP(tray_publish_rate));
    a->Visit(DRAKE_NVP(object_publish_rate));
    a->Visit(DRAKE_NVP(visualizer_publish_rate));

    a->Visit(DRAKE_NVP(scene_index));
    a->Visit(DRAKE_NVP(visualize_drake_sim));
    a->Visit(DRAKE_NVP(publish_efforts));
    a->Visit(DRAKE_NVP(publish_object_velocities));

    a->Visit(DRAKE_NVP(q_init_franka));
    a->Visit(DRAKE_NVP(q_init_tray));
    a->Visit(DRAKE_NVP(q_init_object));
    a->Visit(DRAKE_NVP(tool_attachment_frame));

    a->Visit(DRAKE_NVP(world_x_limits));
    a->Visit(DRAKE_NVP(world_y_limits));
    a->Visit(DRAKE_NVP(world_z_limits));
    a->Visit(DRAKE_NVP(external_force_scaling));

    a->Visit(DRAKE_NVP(visualize_pose_trace));
    a->Visit(DRAKE_NVP(visualize_center_of_mass_plan));
    a->Visit(DRAKE_NVP(visualize_c3_forces));
    a->Visit(DRAKE_NVP(visualize_c3_object_state));
    a->Visit(DRAKE_NVP(visualize_c3_end_effector_state));
    a->Visit(DRAKE_NVP(visualize_workspace));
  }
};