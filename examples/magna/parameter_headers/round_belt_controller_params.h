#pragma once

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
  std::vector<double>
      target_lcs_position;  // x, y, z, roll, pitch, yaw, x_keypoint,
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
    a->Visit(DRAKE_NVP(target_lcs_position));
    a->Visit(DRAKE_NVP(fixed_keypoint_position));
  }
};
