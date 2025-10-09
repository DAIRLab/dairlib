#pragma once

#include "drake/common/yaml/yaml_read_archive.h"

struct DeformLcmChannels {
  std::string allegro_state_channel;
  std::string allegro_input_channel;

  std::string osc_channel;
  std::string osc_debug_channel;

  std::string c3_actor_plan_channel;
  std::string c3_object_plan_channel;
  std::string c3_force_channel;
  std::string c3_debug_output_channel;

  std::string c3_final_target_state_channel;
  std::string c3_target_state_channel;
  std::string c3_actual_state_channel;

  std::string dynamically_feasible_actor_plan_channel;
  std::string dynamically_feasible_plan_channel;

  std::string target_generator_info_channel;
  std::string radio_channel;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(allegro_state_channel));
    a->Visit(DRAKE_NVP(allegro_input_channel));

    a->Visit(DRAKE_NVP(osc_channel));
    a->Visit(DRAKE_NVP(osc_debug_channel));

    a->Visit(DRAKE_NVP(c3_actor_plan_channel));
    a->Visit(DRAKE_NVP(c3_object_plan_channel));
    a->Visit(DRAKE_NVP(c3_force_channel));
    a->Visit(DRAKE_NVP(c3_debug_output_channel));

    a->Visit(DRAKE_NVP(c3_final_target_state_channel));
    a->Visit(DRAKE_NVP(c3_target_state_channel));
    a->Visit(DRAKE_NVP(c3_actual_state_channel));
    
    a->Visit(DRAKE_NVP(dynamically_feasible_actor_plan_channel));
    a->Visit(DRAKE_NVP(dynamically_feasible_plan_channel));

    a->Visit(DRAKE_NVP(target_generator_info_channel));
    a->Visit(DRAKE_NVP(radio_channel));
  }
};
