// Offline probe that prints what the sampling-C3 controller's LCS actually
// resolves to at a single frozen scene state, and what that LCS does when
// nothing pushes on it.
//
// Two things about an LCS never reach any LCM channel, so a recorded log can
// never answer them:
//
//   1. WHICH geometry pairs the per-group contact budgets
//      (resolve_contacts_to_lists) resolved to.  C3_FORCES_* carries witness
//      points, but no geometry identity, so a log can show that two contact
//      slots landed on the same spot without showing that they are the same
//      corner counted twice against two adjacent pieces of a decomposed
//      surface.
//   2. phi, the signed distance at each resolved contact.  The nearest logged
//      analogue is the Anitescu residual, which smears separation across the
//      friction-cone rows at the *planned* states.
//
// This binary prints both, for the planning budget and the cost budget, plus
// the full sorted candidate list for each group so the margin between the
// pairs that made the cut and the ones that did not is visible.  It then rolls
// the cost LCS forward with u = 0 and the end effector held still, which
// isolates what the model believes the object does on its own -- no plan, no
// sample, no cost.  An object that falls in that rollout cannot be scored,
// because no end effector placement can change a fall it is not touching.
//
// The rollout is repeated across lcs_dt_resolution values so the discretization
// can be ruled in or out: a body that is genuinely unsupported in the model
// falls at every step size, while an integration artifact shrinks as the step
// shrinks.
//
// Everything before the probe itself is lifted from jamming_sweep.cc, so the
// scene, the parameters and the contact-pair groups are the ones the real
// controller binary builds.

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <gflags/gflags.h>

#include "common/update_context.h"
#include "examples/sampling_c3/parameter_headers/goal_params.h"
#include "examples/sampling_c3/parameter_headers/sampling_c3_controller_params.h"
#include "examples/sampling_c3/parameter_headers/sampling_c3_options.h"
#include "examples/sampling_c3/sampling_c3_utils.h"
#include "systems/controllers/sampling_based_c3_controller.h"

#include "core/lcs.h"
#include "multibody/lcs_factory.h"
#include "multibody/lcs_factory_options.h"

#include "drake/common/yaml/yaml_io.h"
#include "drake/geometry/query_object.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"

namespace dairlib {
namespace {

using c3::LCS;
using c3::multibody::LCSFactory;
using c3::LCSFactoryOptions;
using drake::geometry::GeometryId;
using drake::geometry::QueryObject;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::MultibodyPlant;
using drake::systems::DiagramBuilder;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using std::string;
using std::vector;
using systems::SamplingC3Controller;

DEFINE_string(demo_name, "cone", "Which three_d_printer demo to probe.");
DEFINE_int32(goal_step, 2,
             "Which goal in fixed_target_position_sequence is being pursued.  "
             "Only affects the printed goal; the LCS does not depend on it.");
DEFINE_int32(object_pose_from_goal_step, 1,
             "Which goal's pose to place the object at, unless --object_xyz / "
             "--object_quat override it.");
DEFINE_string(object_xyz, "", "\"x,y,z\" override of the object position.");
DEFINE_string(object_quat, "",
              "\"w,x,y,z\" override of the object orientation.");
DEFINE_string(ee_xyz, "",
              "\"x,y,z\" override of the end effector position.  The rollout "
              "holds the EE still wherever this puts it, so it changes which "
              "EE-object pair resolves but not the object's support set.");
DEFINE_string(dt_resolutions, "",
              "Comma-separated lcs_dt_resolution values to roll out, e.g. "
              "\"4,8,16\".  Empty uses the configured value alone.");
DEFINE_int32(max_candidates_printed, 12,
             "How many of each group's sorted candidate pairs to print.");
DEFINE_int32(max_per_object_geometry, -1,
             "Override the yaml's max_contacts_per_object_geometry for every "
             "group: 0 for the uncapped closest-N selection, >0 for that cap.  "
             "-1 (the default) uses whatever the yaml configures, so an A/B is "
             "--max_per_object_geometry=0 against =1.");

// Parses exactly `expected` comma-separated doubles out of `text`.  Returns
// false on an empty string so an unset flag reads as "no override", and throws
// on a string that is present but malformed rather than silently ignoring it.
bool ParseDoubles(const string& text, int expected, VectorXd* out) {
  if (text.empty()) return false;
  vector<double> values;
  std::stringstream stream(text);
  string token;
  while (std::getline(stream, token, ',')) {
    values.push_back(std::stod(token));
  }
  if (static_cast<int>(values.size()) != expected) {
    throw std::runtime_error("Expected " + std::to_string(expected) +
                             " comma-separated values, got: " + text);
  }
  *out = Eigen::Map<VectorXd>(values.data(), values.size());
  return true;
}

vector<int> ParseInts(const string& text) {
  vector<int> values;
  if (text.empty()) return values;
  std::stringstream stream(text);
  string token;
  while (std::getline(stream, token, ',')) {
    values.push_back(std::stoi(token));
  }
  return values;
}

// "cone::corner_6" -- the body the geometry hangs off plus the geometry's own
// name, which is what distinguishes one corner sphere (or one ramp piece) from
// the next.  Geometry names alone are ambiguous across models.
string DescribeGeometry(const MultibodyPlant<double>& plant,
                        const drake::geometry::SceneGraphInspector<double>& in,
                        GeometryId id) {
  const auto* body = plant.GetBodyFromFrameId(in.GetFrameId(id));
  const string body_name = body == nullptr ? "?" : body->name();
  string geometry_name = in.GetName(id);
  // Drake qualifies proximity names as "model::body::geometry"; the tail is
  // the only part that varies within a body.
  const size_t last = geometry_name.rfind("::");
  if (last != string::npos) geometry_name = geometry_name.substr(last + 2);
  return body_name + "::" + geometry_name;
}

const char* GroupName(int index, int num_groups) {
  switch (index) {
    case 0:
      return "EE-ground (incl. EE-ramp)";
    case 1:
      return "EE-object";
    case 2:
      return "object-ground (incl. object-ramp)";
    default:
      return index < num_groups ? "object-object" : "?";
  }
}

// Prints every candidate in a group sorted by signed distance, flagging which
// ones the budget kept and how many DISTINCT object-side geometries those
// represent.  The gap between "pairs kept" and "distinct geometries kept" is
// the duplication this probe exists to measure.
void ReportGroup(const MultibodyPlant<double>& plant,
                 const drake::systems::Context<double>& context,
                 const drake::geometry::SceneGraphInspector<double>& inspector,
                 const QueryObject<double>& query_object,
                 const vector<drake::SortedPair<GeometryId>>& candidates,
                 int budget, int cap, int group_index, int num_groups) {
  std::cout << "\n  group " << group_index << " ("
            << GroupName(group_index, num_groups) << "): " << candidates.size()
            << " candidate pairs, budget " << budget
            << ", per-object-geometry cap "
            << (cap > 0 ? std::to_string(cap) : string("none")) << std::endl;
  if (budget == 0) {
    std::cout << "    budget is 0 -- this group contributes no contacts to the "
                 "LCS."
              << std::endl;
    return;
  }

  vector<std::pair<double, drake::SortedPair<GeometryId>>> sorted;
  sorted.reserve(candidates.size());
  for (const auto& pair : candidates) {
    const auto result = query_object.ComputeSignedDistancePairClosestPoints(
        pair.first(), pair.second());
    sorted.emplace_back(result.distance, pair);
  }
  std::sort(sorted.begin(), sorted.end(),
            [](const auto& a, const auto& b) { return a.first < b.first; });

  // The object side is the member whose body is not welded to the world; both
  // the build plate and every ramp piece are welded, so for the object-ground
  // group this picks out the corner sphere.
  auto object_side = [&](const drake::SortedPair<GeometryId>& pair) {
    const auto* body_a = plant.GetBodyFromFrameId(inspector.GetFrameId(pair.first()));
    const bool a_is_fixed =
        body_a == nullptr || body_a->is_floating_base_body() == false;
    return a_is_fixed ? pair.second() : pair.first();
  };

  // Whatever the controller's own policy would keep, so the starred rows and
  // the LCS slots below can never disagree.
  const vector<drake::SortedPair<GeometryId>> kept =
      cap > 0 ? SamplingC3Controller::
                    GetClosestContactPairsCappedPerObjectGeometry(
                        plant, context, candidates, budget, cap)
              : LCSFactory::GetNClosestContactPairs(plant, context, candidates,
                                                    budget);
  std::set<drake::SortedPair<GeometryId>> kept_set(kept.begin(), kept.end());

  const int to_print = std::min<int>(
      sorted.size(), std::max(budget, FLAGS_max_candidates_printed));
  std::cout << "    " << std::left << std::setw(6) << "kept" << std::setw(12)
            << "phi [m]" << "pair" << std::endl;
  for (int i = 0; i < to_print; ++i) {
    std::cout << "    " << std::left << std::setw(6)
              << (kept_set.count(sorted[i].second) ? "  *" : "")
              << std::setw(12) << std::fixed << std::setprecision(6)
              << sorted[i].first
              << DescribeGeometry(plant, inspector, sorted[i].second.first())
              << "  <->  "
              << DescribeGeometry(plant, inspector, sorted[i].second.second())
              << std::endl;
  }
  if (to_print < static_cast<int>(sorted.size())) {
    std::cout << "    ... " << sorted.size() - to_print << " more" << std::endl;
  }

  std::set<GeometryId> kept_object_geometries;
  for (const auto& pair : kept) {
    kept_object_geometries.insert(object_side(pair));
  }
  std::set<GeometryId> all_object_geometries;
  for (const auto& entry : sorted) {
    all_object_geometries.insert(object_side(entry.second));
  }
  std::cout << "    kept " << kept.size() << " pairs spanning "
            << kept_object_geometries.size()
            << " DISTINCT object-side geometries (of "
            << all_object_geometries.size() << " in the group)";
  if (kept_object_geometries.size() < kept.size()) {
    std::cout << "   <-- DUPLICATED";
  }
  std::cout << std::endl;

  // How many geometries are near enough to plausibly carry load, which is the
  // ceiling on what any selection policy can recover.
  std::map<GeometryId, double> closest_per_geometry;
  for (const auto& entry : sorted) {
    const GeometryId id = object_side(entry.second);
    auto it = closest_per_geometry.find(id);
    if (it == closest_per_geometry.end() || entry.first < it->second) {
      closest_per_geometry[id] = entry.first;
    }
  }
  std::cout << "    closest pair per object-side geometry (what a per-geometry "
               "cap of 1 would select from):"
            << std::endl;
  vector<std::pair<double, GeometryId>> by_distance;
  for (const auto& entry : closest_per_geometry) {
    by_distance.emplace_back(entry.second, entry.first);
  }
  std::sort(by_distance.begin(), by_distance.end(),
            [](const auto& a, const auto& b) { return a.first < b.first; });
  for (const auto& entry : by_distance) {
    std::cout << "      " << std::setw(12) << std::fixed
              << std::setprecision(6) << entry.first
              << DescribeGeometry(plant, inspector, entry.second) << std::endl;
  }
}

// The slot ordering the LCS will carry, with each slot's phi.  Slot order is
// what every lambda index in the yaml (final_augmented_cost_contact_indices,
// the g_*/u_* lists) is expressed against, so it is worth printing explicitly.
//
// phi comes from the same Drake closest-point query that
// GetNClosestContactPairs ranks on, rather than from the factory's own
// ComputeContactJacobian, which is private in the pinned c3.
void ReportResolvedSlots(const MultibodyPlant<double>& plant,
                         const QueryObject<double>& query_object,
                         const drake::geometry::SceneGraphInspector<double>& in,
                         const vector<drake::SortedPair<GeometryId>>& resolved,
                         double dt, int N, const string& label) {
  std::cout << "\n--- " << label << ": " << resolved.size()
            << " resolved contacts, dt " << dt << ", N " << N << " ---"
            << std::endl;
  for (size_t i = 0; i < resolved.size(); ++i) {
    const auto result = query_object.ComputeSignedDistancePairClosestPoints(
        resolved[i].first(), resolved[i].second());
    std::cout << "    slot " << i << "  phi " << std::setw(12) << std::fixed
              << std::setprecision(6) << result.distance
              << DescribeGeometry(plant, in, resolved[i].first()) << "  <->  "
              << DescribeGeometry(plant, in, resolved[i].second()) << std::endl;
  }
}

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  if (FLAGS_demo_name != "cone") {
    throw std::runtime_error("Unknown --demo_name value: " + FLAGS_demo_name);
  }

  // --- Parameters, loaded the same way the real controller binary does. ---
  const string controller_params_path =
      "examples/sampling_c3/three_d_printer/" + FLAGS_demo_name +
      "/parameters/sampling_c3_controller_params.yaml";
  auto controller_params =
      drake::yaml::LoadYamlFile<SamplingC3ControllerParams>(
          controller_params_path);
  auto goal_params = drake::yaml::LoadYamlFile<SamplingC3GoalParams>(
      controller_params.goal_params_file);
  const SamplingC3Options& sampling_c3_options =
      controller_params.sampling_c3_options;

  // --- The LCS plant, as in three_d_printer_sampling_c3_controller.cc. ---
  DiagramBuilder<double> plant_lcs_builder;
  auto [plant_lcs, scene_graph] =
      AddMultibodyPlantSceneGraph(&plant_lcs_builder, 0.0);
  AddLCSModelsTo3DPrinterPlant(&plant_lcs, &scene_graph,
                               controller_params.object_models);
  plant_lcs.Finalize();

  std::unique_ptr<MultibodyPlant<drake::AutoDiffXd>> plant_lcs_autodiff =
      drake::systems::System<double>::ToAutoDiffXd(plant_lcs);
  auto plant_lcs_diagram = plant_lcs_builder.Build();
  auto diagram_context = plant_lcs_diagram->CreateDefaultContext();
  auto& plant_lcs_context = plant_lcs_diagram->GetMutableSubsystemContext(
      plant_lcs, diagram_context.get());
  auto plant_lcs_context_ad = plant_lcs_autodiff->CreateDefaultContext();

  const vector<vector<drake::SortedPair<GeometryId>>> contact_pairs =
      BuildConeContactPairs(plant_lcs, controller_params.base_names);

  // --- The frozen scene state. ---
  const int n_q = plant_lcs.num_positions();
  const int n_v = plant_lcs.num_velocities();
  const int n_u = plant_lcs.num_actuators();
  const int n_x = n_q + n_v;

  const int pose_step = std::clamp(
      FLAGS_object_pose_from_goal_step, 0,
      static_cast<int>(goal_params.fixed_target_position_sequence.size()) - 1);
  Vector3d object_position =
      goal_params.fixed_target_position_sequence.at(pose_step).at(0);
  Vector4d object_orientation =
      goal_params.fixed_target_orientation_sequence.at(pose_step).at(0);
  VectorXd override_vector;
  if (ParseDoubles(FLAGS_object_xyz, 3, &override_vector)) {
    object_position = override_vector;
  }
  if (ParseDoubles(FLAGS_object_quat, 4, &override_vector)) {
    object_orientation = override_vector;
  }
  // The linearization integrates the four quaternion coefficients
  // independently, so feeding it an un-normalized pose starts the rollout off
  // the unit sphere before a single step is taken.
  object_orientation.normalize();

  // Parked well clear of the object unless overridden, so the EE-object pair
  // resolves at a large phi and cannot be confused for support.
  Vector3d ee_position(0.175, 0.175, 0.175);
  if (ParseDoubles(FLAGS_ee_xyz, 3, &override_vector)) {
    ee_position = override_vector;
  }

  VectorXd x_lcs = VectorXd::Zero(n_x);
  x_lcs.head(3) = ee_position;
  x_lcs.segment(3, 4) = object_orientation;
  x_lcs.segment(7, 3) = object_position;

  const int goal_step = std::clamp(
      FLAGS_goal_step, 0,
      static_cast<int>(goal_params.fixed_target_position_sequence.size()) - 1);

  std::cout << "Object at " << object_position.transpose() << "  quat "
            << object_orientation.transpose() << "\nEE at "
            << ee_position.transpose() << "\nPursuing goal step " << goal_step
            << " at "
            << goal_params.fixed_target_position_sequence.at(goal_step)
                   .at(0)
                   .transpose()
            << "  quat "
            << goal_params.fixed_target_orientation_sequence.at(goal_step)
                   .at(0)
                   .transpose()
            << std::endl;

  UpdateContext(n_q, n_v, n_u, plant_lcs, &plant_lcs_context,
                *plant_lcs_autodiff, plant_lcs_context_ad.get(), x_lcs);

  const auto& query_object =
      plant_lcs.get_geometry_query_input_port().Eval<QueryObject<double>>(
          plant_lcs_context);
  const auto& inspector = query_object.inspector();

  // ================= 1. What each group resolves to =================
  const vector<int>& plan_budget = sampling_c3_options.resolve_contacts_to;
  const vector<int>& cost_budget =
      sampling_c3_options.resolve_contacts_to_for_cost;

  // The cap actually in force, so the probe reports what the controller would
  // resolve to rather than a second opinion.
  auto effective_cap = [&](const vector<int>& configured, size_t group) {
    if (FLAGS_max_per_object_geometry >= 0) {
      return FLAGS_max_per_object_geometry;
    }
    return group < configured.size() ? configured[group] : 0;
  };

  for (const auto& [label, budget] :
       {std::make_pair(string("PLANNING"), plan_budget),
        std::make_pair(string("COST"), cost_budget)}) {
    std::cout << "\n================ " << label
              << " budget resolve_contacts_to = [";
    for (size_t i = 0; i < budget.size(); ++i) {
      std::cout << budget[i] << (i + 1 < budget.size() ? ", " : "");
    }
    std::cout << "] ================" << std::endl;
    const vector<int>& caps =
        label == "PLANNING"
            ? sampling_c3_options.max_contacts_per_object_geometry
            : sampling_c3_options.max_contacts_per_object_geometry_for_cost;
    for (size_t g = 0; g < contact_pairs.size(); ++g) {
      ReportGroup(plant_lcs, plant_lcs_context, inspector, query_object,
                  contact_pairs[g], budget[g], effective_cap(caps, g), g,
                  contact_pairs.size());
    }
  }

  // ================= 2. phi at the resolved slots =================
  auto resolve = [&](const vector<int>& budget, const vector<int>& caps) {
    vector<drake::SortedPair<GeometryId>> resolved;
    for (size_t g = 0; g < contact_pairs.size(); ++g) {
      if (budget[g] == 0) continue;
      const int cap = effective_cap(caps, g);
      const auto kept =
          cap > 0 ? SamplingC3Controller::
                        GetClosestContactPairsCappedPerObjectGeometry(
                            plant_lcs, plant_lcs_context, contact_pairs[g],
                            budget[g], cap)
                  : LCSFactory::GetNClosestContactPairs(
                        plant_lcs, plant_lcs_context, contact_pairs[g],
                        budget[g]);
      resolved.insert(resolved.end(), kept.begin(), kept.end());
    }
    return resolved;
  };

  LCSFactoryOptions plan_options =
      sampling_c3_options.GetLCSFactoryOptions(/*is_pose_tracking=*/true);
  const vector<drake::SortedPair<GeometryId>> resolved_plan = resolve(
      plan_budget, sampling_c3_options.max_contacts_per_object_geometry);
  plan_options.num_contacts = resolved_plan.size();
  ReportResolvedSlots(plant_lcs, query_object, inspector, resolved_plan,
                      plan_options.dt, plan_options.N, "PLANNING LCS");

  const vector<drake::SortedPair<GeometryId>> resolved_cost =
      resolve(cost_budget,
              sampling_c3_options.max_contacts_per_object_geometry_for_cost);

  // ================= 3. The u = 0 rollout =================
  vector<int> resolutions = ParseInts(FLAGS_dt_resolutions);
  if (resolutions.empty()) {
    resolutions.push_back(sampling_c3_options.lcs_dt_resolution);
  }

  std::cout << "\n================ u = 0 ROLLOUT of the COST LCS ================"
            << "\nThe end effector is held still and no force is applied, so "
               "whatever the object\ndoes here is what the model believes it "
               "does on its own."
            << std::endl;

  for (int resolution : resolutions) {
    LCSFactoryOptions cost_options = {
        .contact_model = sampling_c3_options.contact_model,
        .N = sampling_c3_options.N * resolution,
        .dt = sampling_c3_options.planning_dt_pose / resolution,
        .num_contacts = static_cast<int>(resolved_cost.size()),
        .spring_stiffness = 0.0,
        .num_friction_directions_per_contact =
            sampling_c3_options.num_friction_directions_per_contact_for_cost,
        .mu_per_contact = sampling_c3_options.mu_for_cost,
        .planar_normal_direction = sampling_c3_options.planar_normal_direction};

    if (resolution == resolutions.front()) {
      ReportResolvedSlots(plant_lcs, query_object, inspector, resolved_cost,
                          cost_options.dt, cost_options.N, "COST LCS");
    }

    const LCS lcs = LCSFactory(plant_lcs, plant_lcs_context,
                               *plant_lcs_autodiff, *plant_lcs_context_ad,
                               resolved_cost, cost_options)
                        .GenerateLCS();

    std::cout << "\n  lcs_dt_resolution " << resolution << "  (dt "
              << cost_options.dt << " s, N " << cost_options.N << ", horizon "
              << cost_options.dt * cost_options.N << " s)" << std::endl;
    std::cout << "    " << std::left << std::setw(8) << "knot"
              << std::setw(11) << "obj x" << std::setw(11) << "obj y"
              << std::setw(11) << "obj z" << std::setw(11) << "|dx| mm"
              << "||q||" << std::endl;

    const VectorXd u_zero = VectorXd::Zero(plant_lcs.num_actuators());
    VectorXd x = x_lcs;
    const Vector3d start = x_lcs.segment(7, 3);
    // Print a handful of evenly spaced knots so a resolution of 16 stays as
    // readable as a resolution of 4.
    const int print_every = std::max(1, cost_options.N / 10);
    for (int k = 0; k <= cost_options.N; ++k) {
      if (k % print_every == 0 || k == cost_options.N) {
        std::cout << "    " << std::left << std::setw(8) << k << std::fixed
                  << std::setprecision(6) << std::setw(11) << x(7)
                  << std::setw(11) << x(8) << std::setw(11) << x(9)
                  << std::setw(11) << std::setprecision(2)
                  << (x.segment(7, 3) - start).norm() * 1000.0
                  << std::setprecision(6) << x.segment(3, 4).norm()
                  << std::endl;
      }
      if (k < cost_options.N) x = lcs.Simulate(x, u_zero);
    }
    const Vector3d total = x.segment(7, 3) - start;
    std::cout << "    net dx " << total(0) * 1000.0 << " mm, dy "
              << total(1) * 1000.0 << " mm, dz " << total(2) * 1000.0
              << " mm  (|dz| is " << std::setprecision(0)
              << 100.0 * std::abs(total(2)) / std::max(total.norm(), 1e-12)
              << "% of the total)" << std::setprecision(6) << std::endl;
  }

  return 0;
}

}  // namespace
}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::DoMain(argc, argv); }
