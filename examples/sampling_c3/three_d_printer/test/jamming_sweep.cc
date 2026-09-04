// Offline sweep measuring "how hard will the end effector have to push" as a
// function of where the EE samples, for the 3D printer cone demo.
//
// The scene is frozen at the cone's second-to-last goal -- lined up at the base
// of the ramp, heading for the final goal at the top -- because the common jam
// happens right after this, when the cone topples onto the ramp.  For each
// candidate EE position the sweep solves C3 exactly as the live controller
// would and records two independent predictors of peak EE force (see
// examples/sampling_c3/jamming_metrics.h), alongside the C3 cost the controller
// would have scored that sample with.  Both predictors measure the retimed
// plan, so they describe the same trajectory as that cost.
//
// The z-axis limits (u_vertical_limits, ee_velocity_vertical_limits) come from
// sampling_c3plus_options.yaml; edit them there between runs to sweep a
// different configuration.  Whatever was in effect is recorded in the output.
//
// Keep-out regions are deliberately NOT applied as a filter here.  The point of
// the experiment is to see the force field everywhere the sampler would
// otherwise be allowed to go, including inside the hand-authored goal-2
// keep-out box, so we can judge whether that box sits where the forces actually
// are.  Whether each sample falls inside the box is recorded as a column so the
// plots can overlay it.
//
// Samples come from the demo's real kRandomOnShell sampling strategy, so the
// distribution shown is the one the controller actually faces.
//
// Outputs one np.loadtxt-friendly file, jamming_sweep_random.txt, for
// three_d_printer/test/plot_jamming_sweep.py and jamming_visualizer.cc.

#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <unordered_set>
#include <vector>

#include <Eigen/Dense>
#include <gflags/gflags.h>

#include "examples/sampling_c3/generate_samples.h"
#include "examples/sampling_c3/jamming_ground_truth.h"
#include "examples/sampling_c3/jamming_metrics.h"
#include "examples/sampling_c3/parameter_headers/goal_params.h"
#include "examples/sampling_c3/parameter_headers/robot_sim_params.h"
#include "examples/sampling_c3/parameter_headers/sampling_c3_controller_params.h"
#include "examples/sampling_c3/parameter_headers/sampling_c3_options.h"
#include "examples/sampling_c3/parameter_headers/sampling_params.h"
#include "examples/sampling_c3/sampling_c3_utils.h"
#include "systems/controllers/sampling_based_c3_controller.h"

#include "drake/common/yaml/yaml_io.h"
#include "drake/geometry/query_object.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"

namespace dairlib {
namespace {

using drake::geometry::GeometryId;
using drake::geometry::GeometrySet;
using drake::geometry::QueryObject;
using drake::geometry::Role;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::systems::DiagramBuilder;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using std::string;
using std::vector;
using systems::JammingMetrics;
using systems::JammingMetricsAsRow;
using systems::JammingMetricsColumnNames;
using systems::KeepOutQuery;
using systems::SampleJammingResult;
using systems::SamplingC3Controller;

DEFINE_string(demo_name, "cone", "Demo within sampling_c3/three_d_printer/.");
DEFINE_string(out_dir, "/tmp", "Directory to write the sweep results into.");
DEFINE_int32(goal_step, 2,
             "Index into goal_params.yaml's fixed_target_position_sequence of "
             "the goal being pursued.  The default, 2, is the final goal (up "
             "the ramp), i.e. the leg starting from the second-to-last goal.");
DEFINE_int32(object_pose_from_goal_step, 1,
             "Index into the goal sequence to place the object at.  The "
             "default, 1, is the second-to-last goal: lined up at the base of "
             "the ramp, just before it topples onto it.");
DEFINE_string(object_xyz, "",
              "Optional 'x,y,z' overriding the object position implied by "
              "--object_pose_from_goal_step.");
DEFINE_string(object_quat, "",
              "Optional 'w,x,y,z' overriding the object orientation implied by "
              "--object_pose_from_goal_step.");

DEFINE_int32(num_draws, 2000,
             "Approximate number of samples to draw from the demo's real "
             "sampling strategy, with keep-out regions disabled.");
DEFINE_bool(parallel_solves, false,
            "Solve samples in parallel.  Faster, but the serial default is the "
            "version to trust; solver-fallback attribution is correct either "
            "way.");
DEFINE_bool(label_with_sim, false,
            "After solving, replay each sample's retimed EE plan through the "
            "demo's real Drake sim and record what the object actually did, as "
            "extra columns on the same file.  Ground truth for ranking the "
            "predictors against -- every other column is derived from the LCS, "
            "so they cannot be checked against each other.");
DEFINE_int32(
    osqp_max_iter, 0,
    "If positive, override OSQP's max_iter.  Lowering it forces solver "
    "fallbacks, which is how the solve_fell_back column gets "
    "exercised; 0 keeps the value from the demo's qp settings yaml.");

// Parses "a,b,c" into a fixed-size vector.  Returns false on any mismatch.
bool ParseDoubles(const string& text, int expected_size, VectorXd* out) {
  if (text.empty()) {
    return false;
  }
  vector<double> values;
  std::stringstream stream(text);
  string token;
  while (std::getline(stream, token, ',')) {
    values.push_back(std::stod(token));
  }
  if (static_cast<int>(values.size()) != expected_size) {
    throw std::runtime_error("Expected " + std::to_string(expected_size) +
                             " comma-separated values, got '" + text + "'");
  }
  *out = Eigen::Map<VectorXd>(values.data(), expected_size);
  return true;
}

// A candidate EE position plus the sampler verdicts recorded alongside it.
struct SweptSample {
  Vector3d ee_position;
  bool acceptable = true;        // SampleIsAcceptable, keep-out disabled
  bool inside_keep_out = false;  // diagnostic only, never used to reject
};

// The goal-2 keep-out regions, in a scene of their own, exactly as
// SamplingC3Controller::BuildKeepOutScene builds them.  Only used to record
// which samples the box would have rejected.
class KeepOutScene {
 public:
  KeepOutScene(const vector<string>& keep_out_models, int goal_step) {
    DiagramBuilder<double> builder;
    auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.0);
    const vector<ModelInstanceIndex> indices =
        AddKeepOutModelsToPlant(&plant, &scene_graph, keep_out_models);
    plant.Finalize();
    plant_ = &plant;
    diagram_ = builder.Build();
    diagram_context_ = diagram_->CreateDefaultContext();

    const int step =
        std::clamp(goal_step, 0, static_cast<int>(indices.size()) - 1);
    vector<GeometryId> ids;
    if (!indices.empty() && indices.at(step).is_valid()) {
      for (const auto& body_index : plant.GetBodyIndices(indices.at(step))) {
        for (const GeometryId& id :
             plant.GetCollisionGeometriesForBody(plant.get_body(body_index))) {
          ids.push_back(id);
        }
      }
    }
    has_regions_ = !ids.empty();
    geometries_ = GeometrySet(ids);
  }

  bool has_regions() const { return has_regions_; }

  bool Contains(const Vector3d& ee_position) const {
    if (!has_regions_) {
      return false;
    }
    const auto& plant_context = plant_->GetMyContextFromRoot(*diagram_context_);
    const auto& query_object =
        plant_->get_geometry_query_input_port().Eval<QueryObject<double>>(
            plant_context);
    VectorXd state = VectorXd::Zero(19);
    state.head(3) = ee_position;
    return !systems::SampleAvoidsGeometries(state, query_object, geometries_,
                                            /*clearance=*/0.0);
  }

 private:
  const MultibodyPlant<double>* plant_ = nullptr;
  std::unique_ptr<drake::systems::Diagram<double>> diagram_;
  std::unique_ptr<drake::systems::Context<double>> diagram_context_;
  GeometrySet geometries_;
  bool has_regions_ = false;
};

// Writes one "# <name> v0 v1 ..." comment line holding an LCS state vector.
void WriteStateComment(std::ostream& out, const string& name,
                       const VectorXd& state) {
  out << "# " << name;
  for (int i = 0; i < state.size(); ++i) {
    out << " " << state(i);
  }
  out << "\n";
}

void WriteResults(const string& path, const vector<SweptSample>& samples,
                  const vector<SampleJammingResult>& results,
                  const Vector3d& object_position,
                  const Vector4d& object_orientation, int goal_step,
                  const VectorXd& x_lcs_curr, const VectorXd& x_lcs_des,
                  const VectorXd& x_lcs_final_des,
                  const SamplingC3Options& sampling_c3_options,
                  const vector<systems::GroundTruthLabel>& labels) {
  DRAKE_DEMAND(samples.size() == results.size());
  DRAKE_DEMAND(labels.empty() || labels.size() == results.size());
  std::ofstream out(path);
  if (!out) {
    throw std::runtime_error("Could not open " + path + " for writing");
  }
  out << std::setprecision(10);
  // The frozen scene, so a viewer can place the object exactly where the sweep
  // put it rather than recomputing it and risking drift.
  out << "# scene object_xyz " << object_position(0) << " "
      << object_position(1) << " " << object_position(2) << " object_quat "
      << object_orientation(0) << " " << object_orientation(1) << " "
      << object_orientation(2) << " " << object_orientation(3) << " goal_step "
      << goal_step << "\n";
  // Which plan the force columns describe.  Sweeps written before the raw
  // family was dropped say "raw" or "both" here and carry different columns.
  out << "# metrics_plan retimed\n";
  // The limits this sweep ran under, so a file says which of the
  // baseline/higher-uz/higher-vz configurations produced it.  Both input axes
  // are recorded, not just z: the frac_knots_u_*_at_limit columns are fractions
  // of a bound, so they mean nothing without the bound they were measured
  // against.
  out << "# limits u_vertical " << sampling_c3_options.u_vertical_limits[0]
      << " " << sampling_c3_options.u_vertical_limits[1] << " u_horizontal "
      << sampling_c3_options.u_horizontal_limits[0] << " "
      << sampling_c3_options.u_horizontal_limits[1] << " ee_velocity_vertical "
      << sampling_c3_options.ee_velocity_vertical_limits[0] << " "
      << sampling_c3_options.ee_velocity_vertical_limits[1] << "\n";
  // The three C3 states the solves were run against, so the viewer can draw
  // what C3 was tracking.  Note that c3_state_actual's EE block is a
  // placeholder: every sample overwrites it, and the point cloud *is* the set
  // of actual EE positions.
  WriteStateComment(out, "c3_state_actual", x_lcs_curr);
  WriteStateComment(out, "c3_state_target", x_lcs_des);
  WriteStateComment(out, "c3_state_final_target", x_lcs_final_des);
  out << "# ee_x ee_y ee_z acceptable inside_keep_out solve_fell_back c3_cost";
  for (const string& name : JammingMetricsColumnNames()) {
    out << " " << name;
  }
  if (!labels.empty()) {
    for (const string& name : systems::JammingGroundTruthSim::ColumnNames()) {
      out << " " << name;
    }
  }
  out << "\n";
  for (size_t i = 0; i < samples.size(); ++i) {
    const SweptSample& sample = samples.at(i);
    out << sample.ee_position(0) << " " << sample.ee_position(1) << " "
        << sample.ee_position(2) << " " << (sample.acceptable ? 1 : 0) << " "
        << (sample.inside_keep_out ? 1 : 0) << " "
        << (results.at(i).solve_fell_back ? 1 : 0) << " "
        << results.at(i).c3_cost;
    const VectorXd row = JammingMetricsAsRow(results.at(i).metrics);
    for (int j = 0; j < row.size(); ++j) {
      out << " " << row(j);
    }
    if (!labels.empty()) {
      const VectorXd label_row =
          systems::JammingGroundTruthSim::AsRow(labels.at(i));
      for (int j = 0; j < label_row.size(); ++j) {
        out << " " << label_row(j);
      }
    }
    out << "\n";
  }
  std::cout << "Wrote " << samples.size() << " samples to " << path
            << std::endl;
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
  if (FLAGS_osqp_max_iter > 0) {
    controller_params.osqp_settings.int_options["max_iter"] =
        FLAGS_osqp_max_iter;
    std::cout << "Overriding OSQP max_iter to " << FLAGS_osqp_max_iter
              << std::endl;
  }

  // Read the sampler's copy off the params rather than loading the yaml a
  // second time, so the two cannot disagree.
  const SamplingC3Options& sampling_c3_options =
      controller_params.sampling_c3_options;
  std::cout << "z limits in effect: u_vertical ["
            << sampling_c3_options.u_vertical_limits[0] << ", "
            << sampling_c3_options.u_vertical_limits[1]
            << "] N, ee_velocity_vertical ["
            << sampling_c3_options.ee_velocity_vertical_limits[0] << ", "
            << sampling_c3_options.ee_velocity_vertical_limits[1]
            << "] m/s  (x/y levels are ["
            << sampling_c3_options.u_horizontal_limits[0] << ", "
            << sampling_c3_options.u_horizontal_limits[1] << "] and ["
            << sampling_c3_options.ee_velocity_horizontal_limits[0] << ", "
            << sampling_c3_options.ee_velocity_horizontal_limits[1] << "])"
            << std::endl;

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

  SamplingC3Controller controller(
      plant_lcs, &plant_lcs_context, *plant_lcs_autodiff,
      plant_lcs_context_ad.get(), contact_pairs, controller_params);

  // --- The frozen scene state: the cone at its second-to-last goal. ---
  const int n_x = plant_lcs.num_positions() + plant_lcs.num_velocities();
  const int num_objects = controller_params.num_objects;
  DRAKE_DEMAND(num_objects == 1);  // The cone demo has a single object.

  const int pose_step = std::clamp(
      FLAGS_object_pose_from_goal_step, 0,
      static_cast<int>(goal_params.fixed_target_position_sequence.size()) - 1);
  // The goal sequence's z is already the object's world-frame height for this
  // demo's toppled-cone pose (0 at the build plate, 0.05 at the top of the
  // ramp), so it is used as-is.  Do NOT substitute resting_object_heights here:
  // that is the resting height of the cone standing on its base, and applying
  // it to the on-its-side pose floats the cone above the plate.
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

  VectorXd x_lcs_curr = VectorXd::Zero(n_x);
  x_lcs_curr.head(3) = Vector3d(0.175, 0.175, 0.175);  // overwritten per sample
  x_lcs_curr.segment(3, 4) = object_orientation;
  x_lcs_curr.segment(7, 3) = object_position;

  const int goal_step = std::clamp(
      FLAGS_goal_step, 0,
      static_cast<int>(goal_params.fixed_target_position_sequence.size()) - 1);
  VectorXd x_lcs_des = VectorXd::Zero(n_x);
  x_lcs_des.segment(3, 4) =
      goal_params.fixed_target_orientation_sequence.at(goal_step).at(0);
  x_lcs_des.segment(7, 3) =
      goal_params.fixed_target_position_sequence.at(goal_step).at(0);
  // The EE goal sits a fixed offset above the object, as the goal generator
  // places it.
  x_lcs_des.head(3) = x_lcs_des.segment(7, 3);
  x_lcs_des(2) += goal_params.ee_target_z_offset_above_object;
  // No lookahead sub-goal is applied offline, so C3 tracks the goal directly
  // and the same vector decides the position- vs pose-tracking cost.
  const VectorXd x_lcs_final_des = x_lcs_des;

  std::cout << "Object at " << object_position.transpose() << " quat "
            << object_orientation.transpose() << "\nPursuing goal step "
            << goal_step << " at " << x_lcs_des.segment(7, 3).transpose()
            << std::endl;

  // --- Keep-out regions, recorded but never used to reject. ---
  vector<string> keep_out_models;
  if (controller_params.keep_out_model_sequence.has_value()) {
    keep_out_models = controller_params.keep_out_model_sequence.value();
  }
  KeepOutScene keep_out_scene(keep_out_models, goal_step);
  std::cout << "Goal step " << goal_step << " keep-out regions: "
            << (keep_out_scene.has_regions() ? "present (recorded, not applied)"
                                             : "none")
            << std::endl;

  // --- The pieces SampleIsAcceptable needs, mirroring the controller's ctor.
  const auto& query_object =
      plant_lcs.get_geometry_query_input_port().Eval<QueryObject<double>>(
          plant_lcs_context);
  const auto& inspector = query_object.inspector();
  const GeometryId ee_geometry_id = contact_pairs.at(0).at(0).first();
  std::unordered_set<GeometryId> excluded_geometry_ids{ee_geometry_id};
  for (const string& base_name : controller_params.base_names) {
    for (const GeometryId& id : plant_lcs.GetCollisionGeometriesForBody(
             plant_lcs.GetBodyByName(base_name))) {
      excluded_geometry_ids.insert(id);
    }
  }
  vector<GeometryId> fixed_geometry_ids;
  for (const GeometryId& id : inspector.GetAllGeometryIds(Role::kProximity)) {
    if (!excluded_geometry_ids.count(id)) {
      fixed_geometry_ids.push_back(id);
    }
  }
  const GeometrySet fixed_obstacle_geometries(fixed_geometry_ids);
  const double ee_radius = systems::GetEERadiusFromPlant(
      plant_lcs, plant_lcs_context, contact_pairs);
  auto sampling_params = drake::yaml::LoadYamlFile<SamplingParams>(
      controller_params.sampling_params_file);
  const Eigen::MatrixXd empty_unsuccessful_buffer = Eigen::MatrixXd::Zero(0, 3);

  // Records the sampler's verdicts for a candidate.  Keep-out is passed
  // inactive so it never rejects; whether the box contains the point is
  // recorded separately.
  auto annotate = [&](const Vector3d& ee_position) {
    SweptSample sample;
    sample.ee_position = ee_position;
    VectorXd candidate_state = x_lcs_curr;
    candidate_state.head(3) = ee_position;
    plant_lcs.SetPositionsAndVelocities(&plant_lcs_context, candidate_state);
    sample.acceptable = systems::SampleIsAcceptable(
        candidate_state, sampling_params, sampling_c3_options,
        empty_unsuccessful_buffer, query_object, fixed_obstacle_geometries,
        ee_radius, KeepOutQuery{});
    sample.inside_keep_out = keep_out_scene.Contains(ee_position);
    return sample;
  };

  // --- Draws from the demo's real sampling strategy. ---
  const vector<bool> object_on_target(num_objects, false);
  vector<SweptSample> samples;
  while (static_cast<int>(samples.size()) < FLAGS_num_draws) {
    const vector<Vector3d> drawn =
        controller.GenerateSampleEEPositionsIgnoringKeepOut(
            x_lcs_curr, /*is_doing_c3=*/true, object_on_target);
    if (drawn.empty()) {
      std::cerr << "Sampler returned nothing; stopping draws at "
                << samples.size() << std::endl;
      break;
    }
    for (const Vector3d& ee_position : drawn) {
      samples.push_back(annotate(ee_position));
    }
  }
  std::cout << samples.size() << " samples drawn" << std::endl;

  // --- Solve and score. ---
  vector<Vector3d> ee_positions;
  ee_positions.reserve(samples.size());
  for (const SweptSample& sample : samples) {
    ee_positions.push_back(sample.ee_position);
  }

  // Serial by default: solver fallbacks are attributed per sample either way,
  // but a single-threaded loop is the version to trust.
  const int num_threads = FLAGS_parallel_solves ? 0 : 1;
  std::cout << "Solving C3 for " << ee_positions.size() << " samples ("
            << (FLAGS_parallel_solves ? "parallel" : "serial") << ")..."
            << std::endl;
  const vector<SampleJammingResult> results =
      controller.EvaluateJammingMetricsForSamples(x_lcs_curr, x_lcs_des,
                                                  x_lcs_final_des, ee_positions,
                                                  goal_step, num_threads);

  int num_fell_back = 0;
  for (const SampleJammingResult& result : results) {
    num_fell_back += result.solve_fell_back ? 1 : 0;
  }
  std::cout << num_fell_back << " of " << results.size()
            << " samples had a C3 solve fall back to zero inputs; their force "
               "metrics understate the true effort and are flagged in the "
               "solve_fell_back column."
            << std::endl;

  // --- Ground truth, if asked for. ---
  vector<systems::GroundTruthLabel> labels;
  if (FLAGS_label_with_sim) {
    const auto sim_params = drake::yaml::LoadYamlFile<RobotSimParams>(
        "examples/sampling_c3/three_d_printer/" + FLAGS_demo_name +
        "/parameters/sim_params.yaml");
    systems::JammingGroundTruthSim ground_truth(sim_params.object_models,
                                                sim_params.dt);
    std::cout << "Labelling " << results.size()
              << " samples in the real sim (EE tip sits "
              << ground_truth.ee_to_joint_offset().transpose()
              << " from the printer origin at zero travel)..." << std::endl;

    labels.resize(results.size());
    int num_jammed = 0;
    for (size_t i = 0; i < results.size(); ++i) {
      const SampleJammingResult& result = results.at(i);
      // A solve that produced nothing to execute has nothing to replay, and
      // cannot be jammed by a push it never made.
      const int plan_is_real = std::isnan(result.metrics.no_op_plan)
                                   ? -1
                                   : (result.metrics.no_op_plan > 0.5 ? 0 : 1);
      labels.at(i) = ground_truth.Label(object_orientation, object_position,
                                        result.retimed_ee_plan,
                                        result.planning_dt, plan_is_real);
      num_jammed += labels.at(i).jammed > 0.5 ? 1 : 0;
      if ((i + 1) % 100 == 0) {
        std::cout << "  " << (i + 1) << " / " << results.size() << std::endl;
      }
    }
    std::cout << num_jammed << " of " << results.size()
              << " samples are labelled jammed: the plan commanded real effort "
                 "and moved the object no further than holding still would."
              << std::endl;
  }

  WriteResults(FLAGS_out_dir + "/jamming_sweep_random.txt", samples, results,
               object_position, object_orientation, goal_step, x_lcs_curr,
               x_lcs_des, x_lcs_final_des, sampling_c3_options, labels);

  return 0;
}

}  // namespace
}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::DoMain(argc, argv); }
