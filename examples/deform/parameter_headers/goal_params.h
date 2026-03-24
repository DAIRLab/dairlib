#pragma once

#include <numeric>

#include "common/file_utils.h"
#include "examples/sampling_c3/parameter_headers/goal_params.h"

#include "drake/common/yaml/yaml_read_archive.h"

struct ElastoPlasticGoalParams {
  GoalMode goal_mode;

  /// Parameters used for multiple goal modes.
  /// Success thresholds for node positions.
  double node_success_threshold;

  /// Lookahead parameters to define a sub-goal for C3.
  double lookahead_step_size;

  /// Initial goal (and only goal for fixed goal mode).
  std::vector<Eigen::Vector3d> fixed_node_targets_vector;
  double ee_target_z_offset_above_object;

  /// Random-specific parameters.
  Eigen::VectorXd random_goal_x_limits;
  Eigen::VectorXd random_goal_y_limits;
  Eigen::VectorXd random_goal_radius_limits;

  // Variables that are built from the above parameters for convenience.
  Eigen::VectorXd fixed_node_targets;
  Eigen::Vector3d fixed_ee_target;
  Eigen::VectorXd fixed_q_target;
  std::vector<std::string> state_names;

  template <typename Archive>
  void Serialize(Archive* a) {
    ENUM_DESERIALIZE(a, goal_mode);
    a->Visit(DRAKE_NVP(node_success_threshold));
    a->Visit(DRAKE_NVP(lookahead_step_size));
    a->Visit(DRAKE_NVP(fixed_node_targets_vector));
    a->Visit(DRAKE_NVP(ee_target_z_offset_above_object));
    a->Visit(DRAKE_NVP(random_goal_x_limits));
    a->Visit(DRAKE_NVP(random_goal_y_limits));
    a->Visit(DRAKE_NVP(random_goal_radius_limits));

    fixed_node_targets.resize(fixed_node_targets_vector.size() * 3);
    Eigen::Vector3d sum_node_targets;
    for (size_t i = 0; i < fixed_node_targets_vector.size(); ++i) {
      fixed_node_targets.segment<3>(i * 3) = fixed_node_targets_vector[i];
      sum_node_targets += fixed_node_targets_vector[i];
    }
    fixed_ee_target(0) = sum_node_targets(0) / fixed_node_targets_vector.size();
    fixed_ee_target(1) = sum_node_targets(1) / fixed_node_targets_vector.size();
    fixed_ee_target(2) = sum_node_targets(2) / fixed_node_targets_vector.size();
    fixed_ee_target(2) += ee_target_z_offset_above_object;

    fixed_q_target.resize(fixed_node_targets.size() + 3);
    fixed_q_target << fixed_ee_target, fixed_node_targets;

    state_names.resize(fixed_q_target.size());
    state_names[0] = "ee_x";
    state_names[1] = "ee_y";
    state_names[2] = "ee_z";
    for (size_t i = 0; i < fixed_node_targets_vector.size(); ++i) {
      state_names[3 + i * 3] = "node_" + std::to_string(i) + "_x";
      state_names[3 + i * 3 + 1] = "node_" + std::to_string(i) + "_y";
      state_names[3 + i * 3 + 2] = "node_" + std::to_string(i) + "_z";
    }
  }
};
