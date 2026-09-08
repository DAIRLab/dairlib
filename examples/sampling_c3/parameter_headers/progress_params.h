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
  7. kSimDrakeObjectOnly:           The same as
                                    kSimImpedanceRetimedObjectCostOnly except
                                    the retimed EE plan is replayed through the
                                    demo's real Drake sim rather than the LCS,
                                    so the cost is the object motion the real
                                    contact model actually produces.  Only the
                                    object terms contribute.  Warning:  Drake
                                    sims are likely slower than the other
                                    LCS-based approaches.
*/
enum C3CostComputationType {
  kSimLCS,
  kUseC3Plan,
  kSimLCSReplaceC3EEPlan,
  kSimImpedance,
  kSimImpedanceReplaceC3EEPlan,
  kSimImpedanceObjectCostOnly,
  kSimImpedanceRetimedObjectCostOnly,
  kSimDrakeObjectOnly,
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

  // --- Shape of the window a Drake-sim cost (kSimDrakeObjectOnly) scores. ---
  // All optional so the demos that never select cost type 7 need no entry;
  // unset means the behaviour these knobs were introduced to vary.

  // How long to keep simulating after the plan's last knot, holding the end
  // effector there, as a fraction of the plan's own duration.  Unset (0.0)
  // scores exactly the plan's N+1 knots, which stops the measurement while
  // the object is often still moving:  under the real sim the median sample
  // scores within a fraction of a percent of doing nothing, and letting the
  // response finish is the direct way to separate the samples that pushed
  // from the samples that only looked like they did.  The settled knots are
  // scored too, so a longer window is a larger cost, not just a later one.
  std::optional<double> sim_cost_settle_fraction;
  // Weight on the final scored knot, relative to the rest.  Unset (1.0)
  // weights every knot equally, which dilutes a signal that arrives late:
  // the object's pose partway through a push does not say where the push put
  // it.  With sim_cost_settle_fraction set, the final knot is where the
  // object came to rest, which is the quantity this is meant to emphasise.
  std::optional<double> sim_cost_terminal_weight;
  // The Drake sim's discrete step for the cost rollout, in seconds, for the
  // pose- and position-tracking phases respectively.  Unset means the demo's
  // sim_params.yaml dt.  Coarsening buys most of the controller's per-loop
  // budget back but is scene dependent -- at the endgame pose a 4 ms step
  // invents millimetres of object motion that never happened -- so it must be
  // measured against cost ranking, per scene, before being lowered.
  std::optional<double> sim_cost_dt;
  std::optional<double> sim_cost_dt_position;
  // Drake's point contact model instead of the default
  // hydroelastic-with-fallback.  The cone declares compliant hydroelastic
  // properties on a *mesh*, so the default builds tet meshes and computes
  // contact surfaces every step for what is geometrically a 7-vertex cone.
  // Every collision in the scene also declares a point contact stiffness, so
  // the point model is fully parameterised -- but it is a different contact
  // model, not a cheaper solve of the same one, so it changes the physics and
  // therefore the cost.  Unset (false) is the default hydroelastic model.
  // The same knob as SampleRiskParams::point_contact.
  std::optional<bool> sim_cost_point_contact;
  // Write the interpolated end effector position straight into the state each
  // step instead of asking the printer's PD to track it.  This drops the three
  // actuated axes, the stiff PD and the joint damping -- the stiffest part of
  // the system -- but it also makes the end effector infinitely stiff, so it
  // ploughs through the object instead of stalling against it.  A cost built
  // on this cannot see a sample the printer could not actually execute, which
  // is most of what a Drake rollout is being paid for.  Unset (false) keeps
  // the PD-tracked end effector.  The same knob as
  // SampleRiskParams::prescribed_ee.
  std::optional<bool> sim_cost_prescribed_ee;
  //
  // SampleRiskParams::early_exit has no counterpart here on purpose:  it stops
  // a rollout as soon as a binary travel threshold is crossed, and a cost
  // needs every knot of the window scored, so there is nothing for it to
  // shorten.  SampleRiskParams::travel_threshold likewise configures that
  // label rather than a rollout.

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
    a->Visit(DRAKE_NVP(sim_cost_settle_fraction));
    a->Visit(DRAKE_NVP(sim_cost_terminal_weight));
    a->Visit(DRAKE_NVP(sim_cost_dt));
    a->Visit(DRAKE_NVP(sim_cost_dt_position));
    a->Visit(DRAKE_NVP(sim_cost_point_contact));
    a->Visit(DRAKE_NVP(sim_cost_prescribed_ee));
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
