#pragma once

#include "drake/common/yaml/yaml_read_archive.h"


struct FrankaRosChannels {
  std::string franka_state_channel;
  std::string tray_state_channel;
  std::string box_state_channel;
  std::string franka_input_channel;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(franka_state_channel));
    a->Visit(DRAKE_NVP(tray_state_channel));
    a->Visit(DRAKE_NVP(box_state_channel));
    a->Visit(DRAKE_NVP(franka_input_channel));
  }
};