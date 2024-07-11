#pragma once

#include "drake/common/yaml/yaml_read_archive.h"


struct TrifingerLcmChannels {
  bool is_real_robot;
  std::string trifinger_state_channel;
  std::string cube_state_channel;
  std::string trifinger_input_channel;
  std::string osc_channel;
  std::string osc_debug_channel;
  std::string fingertips_delta_position_channel;
  std::string estimated_friction_torque;
  std::string impedance_debug_channel;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(is_real_robot));
    a->Visit(DRAKE_NVP(trifinger_state_channel));
    a->Visit(DRAKE_NVP(cube_state_channel));
    a->Visit(DRAKE_NVP(trifinger_input_channel));
    a->Visit(DRAKE_NVP(osc_channel));
    a->Visit(DRAKE_NVP(osc_debug_channel));
    a->Visit(DRAKE_NVP(fingertips_delta_position_channel));
    a->Visit(DRAKE_NVP(estimated_friction_torque));
    a->Visit(DRAKE_NVP(impedance_debug_channel));
  }
};