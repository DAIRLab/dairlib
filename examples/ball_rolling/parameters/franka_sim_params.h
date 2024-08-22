#pragma once

#include "drake/common/yaml/yaml_read_archive.h"


struct FrankaSimParams {
  std::string franka_model;
  std::string end_effector_model;
  std::string end_effector_name;
  std::string ground_model;
  std::string jack_model;
  std::string object_body_name;
  std::string platform_model;

  double dt;
  double realtime_rate;
  double actuator_delay;
  double franka_publish_rate;
  double object_publish_rate;
  double visualizer_publish_rate;

  bool visualize_drake_sim;
  bool publish_efforts;

  Eigen::VectorXd camera_pose;
  Eigen::VectorXd camera_target;

  Eigen::VectorXd q_init_franka;
  Eigen::VectorXd q_init_object;
  Eigen::VectorXd tool_attachment_frame;
  Eigen::Vector3d p_world_to_franka;
  Eigen::Vector3d p_franka_to_ground;
  Eigen::Vector3d p_franka_to_platform;

  Eigen::VectorXd world_x_limits;
  Eigen::VectorXd world_y_limits;
  Eigen::VectorXd world_z_limits;

  // Visualizer settings
  std::string visualizer_sample_locations_model;
  std::string visualizer_c3_mode_model;
  bool visualize_workspace;
  bool visualize_c3_state;
  bool visualize_is_c3_mode;
  bool visualize_sample_locations;

  bool visualize_execution_plan;

  std::string visualizer_curr_sample_end_effector_model;
  std::string visualizer_curr_sample_traj_jack_model;
  bool visualize_pose_trace_curr;
  bool visualize_center_of_mass_plan_curr;
  bool visualize_c3_forces_curr;

  std::string visualizer_best_sample_end_effector_model;
  std::string visualizer_best_sample_traj_jack_model;
  bool visualize_pose_trace_best;
  bool visualize_center_of_mass_plan_best;
  bool visualize_c3_forces_best;


  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(franka_model));
    a->Visit(DRAKE_NVP(end_effector_model));
    a->Visit(DRAKE_NVP(end_effector_name));
    a->Visit(DRAKE_NVP(ground_model));
    a->Visit(DRAKE_NVP(jack_model));
    a->Visit(DRAKE_NVP(object_body_name));
    a->Visit(DRAKE_NVP(platform_model));

    a->Visit(DRAKE_NVP(dt));
    a->Visit(DRAKE_NVP(realtime_rate));
    a->Visit(DRAKE_NVP(actuator_delay));
    a->Visit(DRAKE_NVP(franka_publish_rate));
    a->Visit(DRAKE_NVP(object_publish_rate));
    a->Visit(DRAKE_NVP(visualizer_publish_rate));

    a->Visit(DRAKE_NVP(visualize_drake_sim));
    a->Visit(DRAKE_NVP(publish_efforts));

    a->Visit(DRAKE_NVP(camera_pose));
    a->Visit(DRAKE_NVP(camera_target));

    a->Visit(DRAKE_NVP(q_init_franka));
    a->Visit(DRAKE_NVP(q_init_object));
    a->Visit(DRAKE_NVP(tool_attachment_frame));
    a->Visit(DRAKE_NVP(p_world_to_franka));
    a->Visit(DRAKE_NVP(p_franka_to_ground));
    a->Visit(DRAKE_NVP(p_franka_to_platform));

    a->Visit(DRAKE_NVP(world_x_limits));
    a->Visit(DRAKE_NVP(world_y_limits));
    a->Visit(DRAKE_NVP(world_z_limits));

    a->Visit(DRAKE_NVP(visualizer_sample_locations_model));
    a->Visit(DRAKE_NVP(visualizer_c3_mode_model));
    a->Visit(DRAKE_NVP(visualize_workspace));
    a->Visit(DRAKE_NVP(visualize_c3_state));
    a->Visit(DRAKE_NVP(visualize_is_c3_mode));
    a->Visit(DRAKE_NVP(visualize_sample_locations));

    a->Visit(DRAKE_NVP(visualize_execution_plan));

    a->Visit(DRAKE_NVP(visualizer_curr_sample_end_effector_model));
    a->Visit(DRAKE_NVP(visualizer_curr_sample_traj_jack_model));
    a->Visit(DRAKE_NVP(visualize_pose_trace_curr));
    a->Visit(DRAKE_NVP(visualize_center_of_mass_plan_curr));
    a->Visit(DRAKE_NVP(visualize_c3_forces_curr));

    a->Visit(DRAKE_NVP(visualizer_best_sample_end_effector_model));
    a->Visit(DRAKE_NVP(visualizer_best_sample_traj_jack_model));
    a->Visit(DRAKE_NVP(visualize_pose_trace_best));
    a->Visit(DRAKE_NVP(visualize_center_of_mass_plan_best));
    a->Visit(DRAKE_NVP(visualize_c3_forces_best));
  }
};