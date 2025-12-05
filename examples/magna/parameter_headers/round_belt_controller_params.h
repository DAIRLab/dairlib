#pragma once

#include <stdexcept>

#include <Eigen/Dense>

#include "drake/common/yaml/yaml_read_archive.h"

struct RoundBeltControllerParams {
  std::string franka_arm_hand_model;
  std::string ee_model;
  std::string belt_element_model;
  std::string task_board_model;
  double spring_stiffness;                     // N/m
  double spring_rest_length;                   // m
  double spring_damping;                       // Ns/m
  std::vector<double> task_board_position;     // x, y, z
  std::vector<double> task_board_orientation;  // roll, pitch, yaw
  std::vector<std::vector<double>>
      target_lcs_positions;  // List of target LCS positions, each containing:
                             // x, y, z, roll, pitch, yaw, x_keypoint,
                             // y_keypoint, z_keypoint
  std::vector<double> fixed_keypoint_position;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(franka_arm_hand_model));
    a->Visit(DRAKE_NVP(ee_model));
    a->Visit(DRAKE_NVP(belt_element_model));
    a->Visit(DRAKE_NVP(task_board_model));
    a->Visit(DRAKE_NVP(spring_stiffness));
    a->Visit(DRAKE_NVP(spring_rest_length));
    a->Visit(DRAKE_NVP(spring_damping));
    a->Visit(DRAKE_NVP(task_board_position));
    a->Visit(DRAKE_NVP(task_board_orientation));
    a->Visit(DRAKE_NVP(target_lcs_positions));
    a->Visit(DRAKE_NVP(fixed_keypoint_position));
    // Initialize target_lcs_states after loading from YAML
    InitTargetLcsStates();
  }

  /// Get target LCS states as Eigen vectors (positions + zero velocities).
  const std::vector<Eigen::VectorXd>& GetTargetLcsStates() const {
    return target_lcs_states_;
  }

 private:
  // Converted target LCS states (positions + zero velocities)
  std::vector<Eigen::VectorXd> target_lcs_states_;

  /// Convert target_lcs_positions to vector of Eigen::VectorXd and store in
  /// target_lcs_states_. Each target position is padded with zeros for
  /// velocities to form full LCS state (state_dim = 2 * position_dim).
  void InitTargetLcsStates() {
    target_lcs_states_.clear();
    if (target_lcs_positions.empty()) {
      return;
    }
    // All target positions should have the same size
    int position_size = static_cast<int>(target_lcs_positions[0].size());
    int lcs_state_dim = position_size * 2;  // positions + velocities

    for (size_t i = 0; i < target_lcs_positions.size(); ++i) {
      const auto& target_pos = target_lcs_positions[i];
      if (static_cast<int>(target_pos.size()) != position_size) {
        throw std::runtime_error(
            "Target LCS position " + std::to_string(i) + " has size " +
            std::to_string(target_pos.size()) + ", but expected " +
            std::to_string(position_size) + " (same as first target)");
      }
      Eigen::VectorXd target_x_lcs = Eigen::VectorXd::Zero(lcs_state_dim);
      Eigen::VectorXd positions = Eigen::Map<const Eigen::VectorXd>(
          target_pos.data(), target_pos.size());
      target_x_lcs.segment(0, positions.size()) = positions;
      target_lcs_states_.push_back(target_x_lcs);
    }
  }
};
