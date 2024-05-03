#pragma once

#include "drake/common/yaml/yaml_read_archive.h"


struct FrankaDrakeLcmDriverChannels {
  std::string franka_status_channel;
  std::string franka_command_channel;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(franka_status_channel));
    a->Visit(DRAKE_NVP(franka_command_channel));
  }
};