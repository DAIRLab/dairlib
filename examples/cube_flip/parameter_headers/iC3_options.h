#pragma once
#include "drake/common/yaml/yaml_read_archive.h"

struct iC3Options {

  int num_iters;
  bool add_position_constraints;
  bool add_input_constraints;
  int num_segments;

  bool print_costs;

  int iter_to_use;
  int num_timesteps_to_use;
  double vertical_offset;
  bool hold_last_timestep_position;

  int N;
  double dt; // REMOVE after importing all options from c3 repo

  int c3_dt_scaling; 
  double ee_tracking_weight;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(num_iters));
    a->Visit(DRAKE_NVP(add_position_constraints));
    a->Visit(DRAKE_NVP(add_input_constraints));
    a->Visit(DRAKE_NVP(num_segments));
    a->Visit(DRAKE_NVP(print_costs));
    a->Visit(DRAKE_NVP(iter_to_use));
    a->Visit(DRAKE_NVP(num_timesteps_to_use));
    a->Visit(DRAKE_NVP(hold_last_timestep_position));
    a->Visit(DRAKE_NVP(vertical_offset));
    a->Visit(DRAKE_NVP(N)); // REMOVE after importing all options from c3 repo
    a->Visit(DRAKE_NVP(dt)); // REMOVE after importing all options from c3 repo
    a->Visit(DRAKE_NVP(c3_dt_scaling)); 
    a->Visit(DRAKE_NVP(ee_tracking_weight)); 

  }
};
