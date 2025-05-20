#pragma once

// #include "solvers/c3_options.h"   Is this needed in this file?

#include "drake/common/yaml/yaml_read_archive.h"


enum SamplingStrategy { RADIALLY_SYMMETRIC_SAMPLING,
                        RANDOM_ON_CIRCLE_SAMPLING,
                        RANDOM_ON_SPHERE_SAMPLING, 
                        FIXED_SAMPLE,
                        SAMPLE_ON_GRID,
                        SAMPLE_IN_SHELL};

enum ProgressMetric { C3_COST,
                      CURRENT_POSITION_AND_ORIENTATION_COST,
                      POSITION_OR_ORIENTATION_ERROR,
                      MIN_PROGRESS_TO_CONTINUE};

struct SamplingC3SamplingParams {
  int control_loop_delay_ms;
  int sampling_strategy;
  bool filter_samples_for_safety;
  std::vector<Eigen::VectorXd> fixed_sample_locations;
  double sampling_radius;
  double min_angle_from_vertical;
  double max_angle_from_vertical;
  double sampling_height;
  std::vector<double> grid_x_limits;
  std::vector<double> grid_y_limits;
  double sample_projection_clearance;
  int num_additional_samples_repos;
  int num_additional_samples_c3;
  bool consider_best_buffer_sample_when_leaving_c3;
  double spline_width;
  double spherical_repositioning_radius;
  double circular_repositioning_radius;
  double circular_repositioning_height;
  double reposition_speed;
  double use_straight_line_traj_under;
  double use_straight_line_traj_within_angle;
  int repositioning_trajectory_type;
  double repositioning_waypoint_height;
  int N_sample_buffer;
  double pos_error_sample_retention;
  double ang_error_sample_retention;
  int cost_type;
  int cost_type_position_tracking;
  bool use_different_contacts_to_compute_cost;
  int num_control_loops_to_wait;
  int num_control_loops_to_wait_position_tracking;
  int track_c3_progress_via;
  double min_percentage_decrease_in_cost_to_continue;
  int num_control_loops_to_wait_for_progress;
  double cost_switching_threshold_distance;
  double travel_cost_per_meter;
  double c3_to_repos_hysteresis;
  double c3_to_repos_hysteresis_position_tracking;
  double finished_reposition_cost;
  double repos_to_c3_hysteresis;
  double repos_to_c3_hysteresis_position_tracking;
  double hysteresis_between_repos_targets;
  double hysteresis_between_repos_targets_position_tracking;
  bool use_relative_hysteresis;
  double c3_to_repos_cost_fraction;
  double repos_to_c3_cost_fraction;
  double repos_to_repos_cost_fraction;
  double c3_to_repos_cost_fraction_position_tracking;
  double repos_to_c3_cost_fraction_position_tracking;
  double repos_to_repos_cost_fraction_position_tracking;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(control_loop_delay_ms));
    a->Visit(DRAKE_NVP(sampling_strategy));
    a->Visit(DRAKE_NVP(filter_samples_for_safety));
    a->Visit(DRAKE_NVP(fixed_sample_locations));
    a->Visit(DRAKE_NVP(sampling_radius));
    a->Visit(DRAKE_NVP(min_angle_from_vertical));
    a->Visit(DRAKE_NVP(max_angle_from_vertical));
    a->Visit(DRAKE_NVP(sampling_height));
    a->Visit(DRAKE_NVP(grid_x_limits));
    a->Visit(DRAKE_NVP(grid_y_limits));
    a->Visit(DRAKE_NVP(sample_projection_clearance));
    a->Visit(DRAKE_NVP(num_additional_samples_repos));
    a->Visit(DRAKE_NVP(num_additional_samples_c3));
    a->Visit(DRAKE_NVP(consider_best_buffer_sample_when_leaving_c3));
    a->Visit(DRAKE_NVP(spline_width));
    a->Visit(DRAKE_NVP(spherical_repositioning_radius));
    a->Visit(DRAKE_NVP(circular_repositioning_radius));
    a->Visit(DRAKE_NVP(circular_repositioning_height));
    a->Visit(DRAKE_NVP(reposition_speed));
    a->Visit(DRAKE_NVP(use_straight_line_traj_under));
    a->Visit(DRAKE_NVP(use_straight_line_traj_within_angle));
    a->Visit(DRAKE_NVP(repositioning_trajectory_type));
    a->Visit(DRAKE_NVP(repositioning_waypoint_height));
    a->Visit(DRAKE_NVP(N_sample_buffer));
    a->Visit(DRAKE_NVP(pos_error_sample_retention));
    a->Visit(DRAKE_NVP(ang_error_sample_retention));
    a->Visit(DRAKE_NVP(cost_type));
    a->Visit(DRAKE_NVP(cost_type_position_tracking));
    a->Visit(DRAKE_NVP(use_different_contacts_to_compute_cost));
    a->Visit(DRAKE_NVP(num_control_loops_to_wait));
    a->Visit(DRAKE_NVP(num_control_loops_to_wait_position_tracking));
    a->Visit(DRAKE_NVP(track_c3_progress_via));
    a->Visit(DRAKE_NVP(min_percentage_decrease_in_cost_to_continue));
    a->Visit(DRAKE_NVP(num_control_loops_to_wait_for_progress));
    a->Visit(DRAKE_NVP(cost_switching_threshold_distance));
    a->Visit(DRAKE_NVP(travel_cost_per_meter));
    a->Visit(DRAKE_NVP(c3_to_repos_hysteresis));
    a->Visit(DRAKE_NVP(c3_to_repos_hysteresis_position_tracking));
    a->Visit(DRAKE_NVP(finished_reposition_cost));
    a->Visit(DRAKE_NVP(repos_to_c3_hysteresis));
    a->Visit(DRAKE_NVP(repos_to_c3_hysteresis_position_tracking));
    a->Visit(DRAKE_NVP(hysteresis_between_repos_targets));
    a->Visit(DRAKE_NVP(hysteresis_between_repos_targets_position_tracking));
    a->Visit(DRAKE_NVP(use_relative_hysteresis));
    a->Visit(DRAKE_NVP(c3_to_repos_cost_fraction));
    a->Visit(DRAKE_NVP(repos_to_c3_cost_fraction));
    a->Visit(DRAKE_NVP(repos_to_repos_cost_fraction));
    a->Visit(DRAKE_NVP(c3_to_repos_cost_fraction_position_tracking));
    a->Visit(DRAKE_NVP(repos_to_c3_cost_fraction_position_tracking));
    a->Visit(DRAKE_NVP(repos_to_repos_cost_fraction_position_tracking));
  }
};