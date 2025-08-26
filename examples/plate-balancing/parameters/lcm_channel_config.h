#pragma once

#include "drake/common/yaml/yaml_read_archive.h"

namespace dairlib {
namespace examples {
namespace plate_balancing {

/**
 * @brief LCM channel configuration for the plate balancing example.
 *
 * Holds the names of LCM channels used for communication between modules.
 */
struct LcmChannelConfig {
  std::string franka_state_channel;  ///< LCM channel for Franka robot state.
  std::string tray_state_channel;    ///< LCM channel for tray state.
  std::string object_state_channel;  ///< LCM channel for object state.
  std::string franka_input_channel;  ///< LCM channel for Franka input commands.
  std::string franka_input_echo;     ///< LCM channel for echoing Franka input.
  std::string osc_channel;        ///< LCM channel for OSC controller commands.
  std::string osc_debug_channel;  ///< LCM channel for OSC debug information.
  std::string c3_actor_channel;   ///< LCM channel for C3 actor commands.
  std::string c3_object_channel;  ///< LCM channel for C3 object commands.
  std::string c3_force_channel;   ///< LCM channel for C3 force outputs.
  std::string c3_debug_output_channel;  ///< LCM channel for C3 debug outputs.
  std::string c3_target_state_channel;  ///< LCM channel for C3 target state.
  std::string c3_actual_state_channel;  ///< LCM channel for C3 actual state.
  std::string radio_channel;  ///< LCM channel for radio communication.

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(franka_state_channel));
    a->Visit(DRAKE_NVP(tray_state_channel));
    a->Visit(DRAKE_NVP(object_state_channel));
    a->Visit(DRAKE_NVP(franka_input_channel));
    a->Visit(DRAKE_NVP(franka_input_echo));
    a->Visit(DRAKE_NVP(osc_channel));
    a->Visit(DRAKE_NVP(osc_debug_channel));
    a->Visit(DRAKE_NVP(c3_actor_channel));
    a->Visit(DRAKE_NVP(c3_object_channel));
    a->Visit(DRAKE_NVP(c3_force_channel));
    a->Visit(DRAKE_NVP(c3_debug_output_channel));
    a->Visit(DRAKE_NVP(c3_target_state_channel));
    a->Visit(DRAKE_NVP(c3_actual_state_channel));
    a->Visit(DRAKE_NVP(radio_channel));
  }
};

}  // namespace plate_balancing
}  // namespace examples
}  // namespace dairlib
