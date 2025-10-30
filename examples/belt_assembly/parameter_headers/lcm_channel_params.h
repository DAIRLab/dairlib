#pragma once

#include "drake/common/yaml/yaml_read_archive.h"

struct RoundBeltLcmChannels {
  std::string franka_state_channel;
  std::string franka_input_channel;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(franka_state_channel));
    a->Visit(DRAKE_NVP(franka_input_channel));
  }
};