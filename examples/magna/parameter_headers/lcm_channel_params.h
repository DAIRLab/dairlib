#pragma once

#include "drake/common/yaml/yaml_read_archive.h"

struct MagnaLcmChannels {
  std::string franka_state_channel;
  std::string franka_input_channel;
  std::string franka_hand_state_channel;
  std::string franka_hand_input_channel;
  std::string tracking_trajectory_actor_channel;
  std::string osc_channel;
  std::string osc_debug_channel;
  std::string radio_channel;
  std::string franka_hand_target_position_channel;
  std::string deformable_geometry_channel;
  std::string c3_target_state_channel;
  std::string c3_actual_state_channel;
  std::string planned_keypoints_trajectory_channel;
  std::string c3_force_channel;
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(franka_state_channel));
    a->Visit(DRAKE_NVP(franka_input_channel));
    a->Visit(DRAKE_NVP(franka_hand_state_channel));
    a->Visit(DRAKE_NVP(franka_hand_input_channel));
    a->Visit(DRAKE_NVP(tracking_trajectory_actor_channel));
    a->Visit(DRAKE_NVP(osc_channel));
    a->Visit(DRAKE_NVP(osc_debug_channel));
    a->Visit(DRAKE_NVP(radio_channel));
    a->Visit(DRAKE_NVP(franka_hand_target_position_channel));
    a->Visit(DRAKE_NVP(deformable_geometry_channel));
    a->Visit(DRAKE_NVP(c3_target_state_channel));
    a->Visit(DRAKE_NVP(c3_actual_state_channel));
    a->Visit(DRAKE_NVP(planned_keypoints_trajectory_channel));
    a->Visit(DRAKE_NVP(c3_force_channel));
  }
};
