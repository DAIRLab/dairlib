#pragma once

#include "drake/common/yaml/yaml_read_archive.h"


struct TrifingerLcmChannels {
  std::string trifinger_state_channel;
  std::string cube_state_channel;
  std::string trifinger_input_channel;
  std::string osc_channel;
  std::string osc_debug_channel;
  std::string fingertips_delta_position_channel;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(trifinger_state_channel));
    a->Visit(DRAKE_NVP(cube_state_channel));
    a->Visit(DRAKE_NVP(trifinger_input_channel));
    a->Visit(DRAKE_NVP(osc_channel));
    a->Visit(DRAKE_NVP(osc_debug_channel));
    a->Visit(DRAKE_NVP(fingertips_delta_position_channel));
  }
};