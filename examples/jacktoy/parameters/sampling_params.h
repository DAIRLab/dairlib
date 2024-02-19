#pragma once

// #include "solvers/c3_options.h"   Is this needed in this file?

#include "drake/common/yaml/yaml_read_archive.h"


enum SamplingStrategy { RADIALLY_SYMMETRIC_SAMPLING,
                        RANDOM_ON_CIRCLE_SAMPLING,
                        RANDOM_ON_SPHERE_SAMPLING };

struct SamplingC3SamplingParams {
  int sampling_strategy;
  double sampling_radius;
  double sampling_height;
  double min_angle_from_vertical;
  double max_angle_from_vertical;
  double sampling_height;
  int num_additional_samples_repos;
  int num_additional_samples_c3;
  double travel_cost_per_meter;
  double reposition_fixed_cost;
  double finished_reposition_cost;
  double switching_hysteresis;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(sampling_strategy));
    a->Visit(DRAKE_NVP(sampling_radius));
    a->Visit(DRAKE_NVP(sampling_height));
    a->Visit(DRAKE_NVP(min_angle_from_vertical));
    a->Visit(DRAKE_NVP(max_angle_from_vertical));
    a->Visit(DRAKE_NVP(sampling_height));
    a->Visit(DRAKE_NVP(num_additional_samples_repos));
    a->Visit(DRAKE_NVP(num_additional_samples_c3));
    a->Visit(DRAKE_NVP(travel_cost_per_meter));
    a->Visit(DRAKE_NVP(reposition_fixed_cost));
    a->Visit(DRAKE_NVP(finished_reposition_cost));
    a->Visit(DRAKE_NVP(switching_hysteresis));
  }
};