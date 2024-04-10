#pragma once

// #include "solvers/c3_options.h"   Is this needed in this file?

#include "drake/common/yaml/yaml_read_archive.h"


enum SamplingStrategy { RADIALLY_SYMMETRIC_SAMPLING,
                        RANDOM_ON_CIRCLE_SAMPLING,
                        RANDOM_ON_SPHERE_SAMPLING, 
                        FIXED_SAMPLE};

struct SamplingC3SamplingParams {
  int control_loop_delay_ms;
  int sampling_strategy;
  Eigen::VectorXd fixed_sample_location;
  double sampling_radius;
  double min_angle_from_vertical;
  double max_angle_from_vertical;
  double sampling_height;
  int num_additional_samples_repos;
  int num_additional_samples_c3;
  double spline_width;
  double reposition_speed;
  int cost_type;
  double travel_cost_per_meter;
  double c3_to_repos_hysteresis;
  double finished_reposition_cost;
  double repos_to_c3_hysteresis;
  double hysterisis_between_repos_targets;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(control_loop_delay_ms));
    a->Visit(DRAKE_NVP(sampling_strategy));
    a->Visit(DRAKE_NVP(fixed_sample_location));
    a->Visit(DRAKE_NVP(sampling_radius));
    a->Visit(DRAKE_NVP(min_angle_from_vertical));
    a->Visit(DRAKE_NVP(max_angle_from_vertical));
    a->Visit(DRAKE_NVP(sampling_height));
    a->Visit(DRAKE_NVP(num_additional_samples_repos));
    a->Visit(DRAKE_NVP(num_additional_samples_c3));
    a->Visit(DRAKE_NVP(spline_width));
    a->Visit(DRAKE_NVP(reposition_speed));
    a->Visit(DRAKE_NVP(cost_type));
    a->Visit(DRAKE_NVP(travel_cost_per_meter));
    a->Visit(DRAKE_NVP(c3_to_repos_hysteresis));
    a->Visit(DRAKE_NVP(finished_reposition_cost));
    a->Visit(DRAKE_NVP(repos_to_c3_hysteresis));
    a->Visit(DRAKE_NVP(hysterisis_between_repos_targets));
  }
};