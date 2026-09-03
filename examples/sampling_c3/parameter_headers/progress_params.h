#pragma once

#include <vector>

#include <optional>

#include "common/file_utils.h"

#include "drake/common/yaml/yaml_read_archive.h"

/* C3 progress metric options, all phrased as improvement requirements over a
   number of control loops:
  0. kC3Cost:           C3 cost.
  1. kConfigCost:       Current object configuration cost.
  2. kPosOrRotCost:     Current position or rotation error.
  3. kConfigCostDrop:   Drop in object configuration cost (this is the same as
                        kConfigCost if the required drop is 0; a more aggressive
                        drop cuts C3 off earlier).
*/
enum ProgressMetric { kC3Cost, kConfigCost, kPosOrRotCost, kConfigCostDrop };

/* Ways of computing C3 costs after solving the MPC problem:
  0. kSimLCS:                       Simulate the LCS dynamics from the planned
                                    inputs.
  1. kUseC3Plan:                    Use the C3 planned trajectory and inputs.
  2. kSimLCSReplaceC3EEPlan:        Simulate the LCS dynamics from the planned
                                    inputs only for the object; use the planned
                                    EE trajectory.
  3. kSimImpedance:                 Try to emulate the real cost of the system
                                    associated not only applying the planned
                                    inputs, but also tracking the planned EE
                                    trajectory with an impedance controller.
  4. kSimImpedanceReplaceC3EEPlan:  The same as kSimImpedance except the EE
                                    states are replaced with the plan from C3 at
                                    the end.
  5. kSimImpedanceObjectCostOnly:   The same as kSimImpedance except only the
                                    object terms contribute to the final cost.
  6. kSimImpedanceRetimedObjectCostOnly:
                                    The same as kSimImpedanceObjectCostOnly
                                    except the EE plan is first slowed to the
                                    configured EE velocity limits and then
                                    resampled back onto the original knot
                                    times, so every sample's cost covers the
                                    same amount of time and a plan that has to
                                    be slowed down simply gets less far.
*/
enum C3CostComputationType {
  kSimLCS,
  kUseC3Plan,
  kSimLCSReplaceC3EEPlan,
  kSimImpedance,
  kSimImpedanceReplaceC3EEPlan,
  kSimImpedanceObjectCostOnly,
  kSimImpedanceRetimedObjectCostOnly,
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
  // Optional per-goal-step override of cost_switching_threshold_distance,
  // indexed by the controller's goal-sequence step.  When set, its length must
  // equal the number of goal-sequence steps (validated in
  // SamplingC3ControllerParams::Serialize); unset => every goal uses the scalar
  // cost_switching_threshold_distance above.
  std::optional<std::vector<double>> cost_switching_threshold_distance_sequence;
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
    a->Visit(DRAKE_NVP(cost_switching_threshold_distance_sequence));
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
