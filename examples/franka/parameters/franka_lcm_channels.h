#pragma once

#include "drake/common/yaml/yaml_read_archive.h"


struct FrankaLcmChannels {
  std::string franka_state_channel;
  std::string tray_state_channel;
  std::string box_state_channel;
  std::string franka_input_channel;
  std::string franka_input_echo;
  std::string osc_channel;
  std::string osc_debug_channel;
  std::string c3_actor_channel;
  std::string c3_object_channel;
  std::string c3_force_channel;
  std::string c3_debug_output_channel;
  std::string radio_channel;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(franka_state_channel));
    a->Visit(DRAKE_NVP(tray_state_channel));
    a->Visit(DRAKE_NVP(box_state_channel));
    a->Visit(DRAKE_NVP(franka_input_channel));
    a->Visit(DRAKE_NVP(franka_input_echo));
    a->Visit(DRAKE_NVP(osc_channel));
    a->Visit(DRAKE_NVP(osc_debug_channel));
    a->Visit(DRAKE_NVP(c3_actor_channel));
    a->Visit(DRAKE_NVP(c3_object_channel));
    a->Visit(DRAKE_NVP(c3_force_channel));
    a->Visit(DRAKE_NVP(c3_debug_output_channel));
    a->Visit(DRAKE_NVP(radio_channel));
  }
};