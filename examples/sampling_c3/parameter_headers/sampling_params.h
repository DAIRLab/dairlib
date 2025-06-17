#pragma once

#include "drake/common/yaml/yaml_read_archive.h"

// TODO @bibit temporary, this import should go away when cost type is elsewhere
#include "solvers/c3_options.h"


enum SamplingStrategy {
  kRadiallySymmetric,
  kRandomOnCircle,
  kRandomOnSphere,
  kFixed,
  kRandomOnPerimeter,
  kRandomOnShell
};

enum ProgressMetric {
  kC3Cost,
  kConfigCost,
  kPosOrRotCost,
  kConfigProgressOverLoops
};


// TODO: @bibit parameter restructuring should reconsider many of these contents
struct SamplingParams {
  int control_loop_delay_ms;  // TODO @bibit does not belong here
  SamplingStrategy sampling_strategy;
  bool filter_samples_for_safety;
  std::vector<Eigen::VectorXd> fixed_sample_locations;  // TODO: @bibit 3D?
  double sampling_radius;
  double min_angle_from_vertical;
  double max_angle_from_vertical;
  double sampling_height;
  std::vector<double> grid_x_limits;
  std::vector<double> grid_y_limits;
  double sample_projection_clearance;
  double min_sampling_radius;
  double max_sampling_radius;
  int num_additional_samples_repos;
  int num_additional_samples_c3;
  bool consider_best_buffer_sample_when_leaving_c3;
  int N_sample_buffer;
  double pos_error_sample_retention;
  double ang_error_sample_retention;
  C3CostComputationType cost_type;
  C3CostComputationType cost_type_position_tracking;
  bool use_different_contacts_to_compute_cost;
  int num_control_loops_to_wait;
  int num_control_loops_to_wait_position_tracking;
  ProgressMetric track_c3_progress_via;
  double progress_enforced_cost_drop;
  int progress_enforced_over_n_loops;
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
    a->Visit(DRAKE_NVP(filter_samples_for_safety));
    a->Visit(DRAKE_NVP(fixed_sample_locations));
    a->Visit(DRAKE_NVP(sampling_radius));
    a->Visit(DRAKE_NVP(min_angle_from_vertical));
    a->Visit(DRAKE_NVP(max_angle_from_vertical));
    a->Visit(DRAKE_NVP(sampling_height));
    a->Visit(DRAKE_NVP(grid_x_limits));
    a->Visit(DRAKE_NVP(grid_y_limits));
    a->Visit(DRAKE_NVP(sample_projection_clearance));
    a->Visit(DRAKE_NVP(min_sampling_radius));
    a->Visit(DRAKE_NVP(max_sampling_radius));
    a->Visit(DRAKE_NVP(num_additional_samples_repos));
    a->Visit(DRAKE_NVP(num_additional_samples_c3));
    a->Visit(DRAKE_NVP(consider_best_buffer_sample_when_leaving_c3));
    a->Visit(DRAKE_NVP(N_sample_buffer));
    a->Visit(DRAKE_NVP(pos_error_sample_retention));
    a->Visit(DRAKE_NVP(ang_error_sample_retention));

    // Load yaml integers into custom enum type.
    int raw_cost_type = static_cast<int>(cost_type);
    int raw_cost_type_position_tracking = static_cast<int>(
      cost_type_position_tracking);
    a->Visit(drake::MakeNameValue("cost_type", &raw_cost_type));
    a->Visit(drake::MakeNameValue("cost_type_position_tracking",
                                  &raw_cost_type_position_tracking));
    cost_type = static_cast<C3CostComputationType>(raw_cost_type);
    cost_type_position_tracking = static_cast<C3CostComputationType>(
      raw_cost_type_position_tracking);

    int raw_track_c3_progress_via = static_cast<int>(track_c3_progress_via);
    a->Visit(drake::MakeNameValue("track_c3_progress_via",
                                  &raw_track_c3_progress_via));
    track_c3_progress_via = static_cast<ProgressMetric>(
      raw_track_c3_progress_via);

    int raw_sampling_strategy = static_cast<int>(sampling_strategy);
    a->Visit(drake::MakeNameValue("sampling_strategy", &raw_sampling_strategy));
    sampling_strategy = static_cast<SamplingStrategy>(raw_sampling_strategy);

    a->Visit(DRAKE_NVP(use_different_contacts_to_compute_cost));
    a->Visit(DRAKE_NVP(num_control_loops_to_wait));
    a->Visit(DRAKE_NVP(num_control_loops_to_wait_position_tracking));
    a->Visit(DRAKE_NVP(progress_enforced_cost_drop));
    a->Visit(DRAKE_NVP(progress_enforced_over_n_loops));
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
