#pragma once

#include "drake/common/yaml/yaml_read_archive.h"

#include "common/file_utils.h"
#include "solvers/c3_options.h"


/* C3 progress metric options, all phrased as improvement requirements over a
   number of control loops:
  0. kC3Cost:           C3 cost.
  1. kConfigCost:       Current object configuration cost.
  2. kPosOrRotCost:     Current position or rotation error.
  3. kConfigCostDrop:   Drop in object configuration cost (this is the same as
                        kConfigCost if the required drop is 0; a more aggressive
                        drop cuts C3 off earlier).
*/
enum ProgressMetric {
  kC3Cost,
  kConfigCost,
  kPosOrRotCost,
  kConfigCostDrop
};


struct SamplingC3ProgressParams {
  C3CostComputationType cost_type;
  C3CostComputationType cost_type_position;
  int num_control_loops_to_wait;
  int num_control_loops_to_wait_position;
  ProgressMetric track_c3_progress_via;
  double progress_enforced_cost_drop;
  int progress_enforced_over_n_loops;
  double cost_switching_threshold_distance;
  double travel_cost_per_meter;
  double hyst_c3_to_repos;
  double hyst_c3_to_repos_position;
  double finished_reposition_cost;
  double hyst_repos_to_c3;
  double hyst_repos_to_c3_position;
  double hyst_repos_to_repos;
  double hyst_repos_to_repos_position;
  bool use_relative_hysteresis;
  double hyst_c3_to_repos_frac;
  double hyst_repos_to_c3_frac;
  double hyst_repos_to_repos_frac;
  double hyst_c3_to_repos_frac_position;
  double hyst_repos_to_c3_frac_position;
  double hyst_repos_to_repos_frac_position;

  template <typename Archive>
  void Serialize(Archive* a) {
    ENUM_DESERIALIZE(a, cost_type);
    ENUM_DESERIALIZE(a, cost_type_position);
    ENUM_DESERIALIZE(a, track_c3_progress_via);
    a->Visit(DRAKE_NVP(num_control_loops_to_wait));
    a->Visit(DRAKE_NVP(num_control_loops_to_wait_position));
    a->Visit(DRAKE_NVP(progress_enforced_cost_drop));
    a->Visit(DRAKE_NVP(progress_enforced_over_n_loops));
    a->Visit(DRAKE_NVP(cost_switching_threshold_distance));
    a->Visit(DRAKE_NVP(travel_cost_per_meter));
    a->Visit(DRAKE_NVP(hyst_c3_to_repos));
    a->Visit(DRAKE_NVP(hyst_c3_to_repos_position));
    a->Visit(DRAKE_NVP(finished_reposition_cost));
    a->Visit(DRAKE_NVP(hyst_repos_to_c3));
    a->Visit(DRAKE_NVP(hyst_repos_to_c3_position));
    a->Visit(DRAKE_NVP(hyst_repos_to_repos));
    a->Visit(DRAKE_NVP(hyst_repos_to_repos_position));
    a->Visit(DRAKE_NVP(use_relative_hysteresis));
    a->Visit(DRAKE_NVP(hyst_c3_to_repos_frac));
    a->Visit(DRAKE_NVP(hyst_repos_to_c3_frac));
    a->Visit(DRAKE_NVP(hyst_repos_to_repos_frac));
    a->Visit(DRAKE_NVP(hyst_c3_to_repos_frac_position));
    a->Visit(DRAKE_NVP(hyst_repos_to_c3_frac_position));
    a->Visit(DRAKE_NVP(hyst_repos_to_repos_frac_position));
  }
};
