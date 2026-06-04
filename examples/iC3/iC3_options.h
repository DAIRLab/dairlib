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

  bool use_time_varying_lcs;

  bool track_ic3_inputs;
  bool add_constraints_follow_plan;

  int N;
  double dt; // REMOVE after importing all options from c3 repo

  int rollout_dt_scaling;

  Eigen::VectorXd value_function_ee_cost;
  double value_function_object_orientation_cost;
  Eigen::VectorXd value_function_object_position_cost;
  Eigen::VectorXd value_function_velocity_cost;

  int value_function_search_size;

  double ee_tracking_weight;
  Eigen::VectorXd ee_tracking_vector;

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
    a->Visit(DRAKE_NVP(rollout_dt_scaling));
    a->Visit(DRAKE_NVP(N)); 
    a->Visit(DRAKE_NVP(dt)); 
    a->Visit(DRAKE_NVP(track_ic3_inputs));
    a->Visit(DRAKE_NVP(add_constraints_follow_plan));
    a->Visit(DRAKE_NVP(rollout_dt_scaling));
    a->Visit(DRAKE_NVP(ee_tracking_weight)); 
    a->Visit(DRAKE_NVP(ee_tracking_vector)); 
    a->Visit(DRAKE_NVP(use_time_varying_lcs)); 
    a->Visit(DRAKE_NVP(value_function_ee_cost)); 
    a->Visit(DRAKE_NVP(value_function_object_orientation_cost)); 
    a->Visit(DRAKE_NVP(value_function_object_position_cost)); 
    a->Visit(DRAKE_NVP(value_function_velocity_cost)); 
    a->Visit(DRAKE_NVP(value_function_search_size)); 

  }
};
