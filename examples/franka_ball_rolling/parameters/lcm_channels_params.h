#pragma once

#include "drake/common/yaml/yaml_read_archive.h"

struct BallRollingLcmChannels {
  std::string franka_input_channel;
  std::string c3_object_trajectory_channel;
  std::string c3_target_state_channel;
  std::string c3_actual_state_channel;
  std::string c3_force_channel;
  std::string impedance_input_channel;
  std::string franka_output_channel;
  std::string franka_state_channel;
  std::string true_ball_state_channel;
  std::string estimated_ball_state_channel;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(franka_input_channel));
    a->Visit(DRAKE_NVP(c3_object_trajectory_channel));
    a->Visit(DRAKE_NVP(c3_target_state_channel));
    a->Visit(DRAKE_NVP(c3_actual_state_channel));
    a->Visit(DRAKE_NVP(c3_force_channel));
    a->Visit(DRAKE_NVP(impedance_input_channel));
    a->Visit(DRAKE_NVP(franka_output_channel));
    a->Visit(DRAKE_NVP(franka_state_channel));
    a->Visit(DRAKE_NVP(true_ball_state_channel));
    a->Visit(DRAKE_NVP(estimated_ball_state_channel));
  }
};