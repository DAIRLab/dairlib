#pragma once

#include "drake/common/yaml/yaml_read_archive.h"


struct FrankaLcmChannels {
  std::string franka_state_channel;
  std::string object_state_channel;
  std::string box_state_channel;
  std::string franka_input_channel;
  std::string franka_input_echo;
  std::string osc_channel;
  std::string osc_debug_channel;

  std::string c3_actor_curr_plan_channel;
  std::string c3_object_curr_plan_channel;
  std::string c3_force_curr_channel;
  std::string c3_debug_output_curr_channel;

  std::string c3_actor_best_plan_channel;
  std::string c3_object_best_plan_channel;
  std::string c3_force_best_channel;
  std::string c3_debug_output_best_channel;

  std::string tracking_trajectory_actor_channel;
  std::string tracking_trajectory_object_channel;

  std::string c3_target_state_channel;
  std::string c3_actual_state_channel;
  std::string radio_channel;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(franka_state_channel));
    a->Visit(DRAKE_NVP(object_state_channel));
    a->Visit(DRAKE_NVP(box_state_channel));
    a->Visit(DRAKE_NVP(franka_input_channel));
    a->Visit(DRAKE_NVP(franka_input_echo));
    a->Visit(DRAKE_NVP(osc_channel));
    a->Visit(DRAKE_NVP(osc_debug_channel));
    
    a->Visit(DRAKE_NVP(c3_actor_curr_plan_channel));
    a->Visit(DRAKE_NVP(c3_object_curr_plan_channel));
    a->Visit(DRAKE_NVP(c3_force_curr_channel));
    a->Visit(DRAKE_NVP(c3_debug_output_curr_channel));

    a->Visit(DRAKE_NVP(c3_actor_best_plan_channel));
    a->Visit(DRAKE_NVP(c3_object_best_plan_channel));
    a->Visit(DRAKE_NVP(c3_force_best_channel));
    a->Visit(DRAKE_NVP(c3_debug_output_best_channel));

    a->Visit(DRAKE_NVP(tracking_trajectory_actor_channel));
    a->Visit(DRAKE_NVP(tracking_trajectory_object_channel));

    a->Visit(DRAKE_NVP(c3_target_state_channel));
    a->Visit(DRAKE_NVP(c3_actual_state_channel));
    a->Visit(DRAKE_NVP(radio_channel));
  }
};