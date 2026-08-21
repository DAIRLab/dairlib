#pragma once
#include "drake/common/yaml/yaml_read_archive.h"

struct iC3Options {

  int num_iters;
  bool add_position_constraints;
  bool add_input_constraints;

  bool print_costs;

  bool add_acceleration_cost;
  double acceleration_cost_weight;

  int transform_update_frequency;
  double vf_trust_region_weight;

  int iter_to_use;
  double vertical_offset;
  bool hold_last_timestep_position;

  bool use_time_varying_lcs;

  bool track_ic3_inputs;
  bool add_constraints_follow_plan;

  int N;
  double dt; // REMOVE after importing all options from c3 repo

  int rollout_dt_scaling;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(num_iters));
    a->Visit(DRAKE_NVP(add_position_constraints));
    a->Visit(DRAKE_NVP(add_input_constraints));
    a->Visit(DRAKE_NVP(print_costs));
    a->Visit(DRAKE_NVP(iter_to_use));
    a->Visit(DRAKE_NVP(hold_last_timestep_position));
    a->Visit(DRAKE_NVP(vertical_offset));
    a->Visit(DRAKE_NVP(rollout_dt_scaling));
    a->Visit(DRAKE_NVP(N)); 
    a->Visit(DRAKE_NVP(dt)); 
    a->Visit(DRAKE_NVP(track_ic3_inputs));
    a->Visit(DRAKE_NVP(add_constraints_follow_plan));
    a->Visit(DRAKE_NVP(add_acceleration_cost));
    a->Visit(DRAKE_NVP(acceleration_cost_weight));
    a->Visit(DRAKE_NVP(transform_update_frequency));
    a->Visit(DRAKE_NVP(vf_trust_region_weight));
    a->Visit(DRAKE_NVP(use_time_varying_lcs)); 
  }
};
