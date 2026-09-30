// Is the loop-to-loop swing in sample costs a bug, or the cost's honest
// response to its inputs?
//
// In the 2026-09-24 realistic-sim logs the whole batch of sample costs -- the
// current location included -- moves together by up to 10-30x between
// consecutive control loops (e.g. simlog-000010 at t = 167.2-167.3 s: 1.72e4,
// 1.13e3, 1.51e4).  The sample buffer stores each sample with the cost of the
// loop that scored it, so a buffered sample from a lucky low loop later
// outbids every freshly scored one; that is what nearly every C3 -> repos
// "found good sample" switch turned out to be.  Before the buffer is changed to
// work around the swing, this separates its possible sources.  Only index 0,
// the current location, is compared across calls:  its EE position is fixed by
// the state, whereas the other samples are redrawn at random every loop.
//
//   1. frozen:  the same logged state fed repeatedly with predicted x0 off.
//      Any spread here is nondeterminism in the solve or cost -- a bug.  Run at
//      one thread and at the configured count, to expose races in the OpenMP
//      sample loop.
//   2. frozen, predicted x0 on (as shipped):  the same state, but each loop's
//      plan feeds the EE state of the next.  A swing here that (1) lacks is
//      the prediction feedback, not the inputs.
//   3. window replay:  the logged inputs of the seconds leading up to the
//      fixture, fed in order with the shipped settings, printing the
//      recomputed index-0 cost beside the logged one.  Validates the harness;
//      the predicted-x0 clamp uses the wall-clock solve time, so agreement is
//      approximate rather than exact.
//   4. attribution:  from the fixture, swap in the object pose, EE position or
//      EE velocity of --t_other one at a time (predicted x0 off).
//   5. noise:  white noise at the sim injector's per-axis std on the fixture's
//      object pose (predicted x0 off).
//
// Two standalone modes replace the experiments above when their flag is set:
// the sample census (--census_times), and the plan census
// (--plan_census_times).  The plan census asks whether the current-location C3
// plan moves the object with the end effector or for free:  per fixture and
// per parameter variant it re-solves the plan and splits the planned contact
// forces on the object by contact group (finger, ground, ramp), with each
// group's torque about the object's CoM projected on the plan's rotation axis.
// A plan that tilts the object while its finger group carries no torque is
// getting that tilt from the environment contacts alone.
//
// Usage:
//   bazel build //examples/sampling_c3/three_d_printer/test:sample_cost_repeatability
//   ./bazel-bin/examples/sampling_c3/three_d_printer/test/sample_cost_repeatability \
//       --log=/home/bibit/3d_printer/logs/2026/09_24_26/000010/simlog-000010 \
//       --t=167.27 --t_other=167.21

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <dairlib/lcmt_c3_state.hpp>
#include <dairlib/lcmt_object_state.hpp>
#include <dairlib/lcmt_radio_out.hpp>
#include <dairlib/lcmt_sampling_c3_debug.hpp>
#include <dairlib/lcmt_timestamped_saved_traj.hpp>
#include <gflags/gflags.h>
#include <lcm/lcm-cpp.hpp>

#include "examples/sampling_c3/parameter_headers/sampling_c3_controller_params.h"
#include "examples/sampling_c3/sampling_c3_utils.h"
#include "systems/controllers/sampling_based_c3_controller.h"

#include "c3/core/traj_eval.h"
#include "common/quaternion_axis_alignment.h"
#include "systems/framework/timestamped_vector.h"

#include "drake/common/yaml/yaml_io.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"

DEFINE_string(log, "", "Path to the source simlog/hwlog.");
DEFINE_string(demo_name, "cone", "Demo whose parameters to load.");
DEFINE_double(t, 167.27,
              "Log-relative time [s] of the fixture loop (the loop nearest it "
              "is used).  Time 0 is the first SAMPLING_C3_DEBUG message.");
DEFINE_double(t_other, 167.21,
              "Log-relative time of the loop whose inputs are swapped in for "
              "the attribution experiment.");
DEFINE_int32(repeats, 30, "Calls per frozen-state experiment.");
DEFINE_int32(noise_trials, 50, "Calls in the noise experiment.");
DEFINE_double(window_seconds, 3.0,
              "How much logged history the window replay feeds before the "
              "fixture.");
DEFINE_int32(seed, 0, "Seed for the noise experiment.");
DEFINE_string(census_times, "",
              "Comma-separated fixture times.  When set, run only the sample "
              "census:  at each fixture, ComputePlan is called "
              "--census_repeats times on the frozen logged state with the "
              "shipped sampler, and every candidate it scores is written to "
              "--census_csv with its cost split by state block.");
DEFINE_int32(census_repeats, 40, "ComputePlan calls per census fixture.");
DEFINE_string(census_csv, "/tmp/sample_census.csv", "Census output path.");
DEFINE_bool(ignore_twist, false,
            "Turn on sampling_c3_options.cost_ignores_tracked_axis_twist.");
DEFINE_double(census_warmup_t, -1,
              "If >= 0, also feed the logged loop nearest this time after the "
              "goal warm-up, e.g. the loop where the live controller latched "
              "pose tracking, so the census inherits that latch.");
DEFINE_string(plan_census_times, "",
              "Comma-separated fixture times.  When set, run only the plan "
              "census:  at each fixture, the current-location C3 plan is "
              "re-solved once per --plan_variants entry (predicted x0 off), "
              "and its object motion, EE motion and contact forces by group "
              "are printed and written to --plan_census_csv.");
DEFINE_string(plan_variants, "shipped",
              "Semicolon-separated variants, each 'name' or "
              "'name:key=value,key=value'.  Keys:  object_pose=clean, "
              "object_dz (m, applied after object_pose), scale_lcs, "
              "aug_rows=<groups> and aug_scaling (the final C3+ solve's "
              "final_augmented_cost_contact_indices/_scaling), "
              "u_ratio_<groups> (u_eta = ratio * u_lambda; groups may be "
              "'+'-joined, or env/all), "
              "num_contacts_index, admm_iter, rho_scale, w_G, w_U, "
              "w_G_position, w_U_position, end_on_qp_step, contact_model, "
              "planning_dt_pose, "
              "planning_dt_position, mu_<group>, and <group>_<weight> (a "
              "scale on that group's rows of the planning g_lambda, g_eta, "
              "u_lambda or u_eta lists, pose and position alike; Anitescu "
              "only).  Groups:  ee_ground, finger, ground, ramp.");
DEFINE_string(plan_census_csv, "/tmp/plan_census.csv",
              "Plan census output path.");
DEFINE_string(goal_params, "",
              "Plan census:  a goal_params yaml to use instead of the log "
              "folder's, so an old fixture can be re-solved against a new goal "
              "sequence.  Each loop logged while pursuing the log's goal k is "
              "re-targeted to this file's goal k (sub-goal lookahead and "
              "re-twisted orientation as the goal generator builds them).");
DEFINE_string(progress_params, "",
              "Plan census:  a progress_params yaml to use instead of the log "
              "folder's (e.g. for its per-goal cost-switching thresholds).");

namespace dairlib {
namespace {

using drake::SortedPair;
using drake::geometry::GeometryId;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using drake::systems::DiagramBuilder;
using Eigen::Vector3d;
using Eigen::VectorXd;
using systems::SamplingC3Controller;
using systems::TimestampedVector;

// The per-axis white noise the sim's object-state error injector adds, from
// the cone demo's sim_params.yaml (position in m, orientation in degrees).
const Vector3d kNoisePositionStd(0.00018, 0.00032, 0.00043);
const Vector3d kNoiseOrientationStdDeg(0.55, 0.31, 0.38);

// Everything the controller consumed on one logged control loop, plus what it
// scored the current location at.
struct LoggedLoop {
  int64_t utime = 0;
  double t = 0;  // log-relative
  VectorXd x_actual, x_target, x_final_target;
  dairlib::lcmt_radio_out radio{};
  double logged_cost0 = NAN;
  int goal = 0;
  bool is_c3 = false;
  // The simulator's error-free object pose (quaternion wxyz, position) at the
  // loop, when the log has OBJECT_STATE_SIMULATION_CLEAN; empty otherwise.
  VectorXd clean_object_pose;
};

// The sample_costs block of a SAMPLE_COSTS message.  Found by name:  the
// message also carries the jam label blocks, in an order not to be relied on.
double CostOfSample0(const dairlib::lcmt_timestamped_saved_traj& msg) {
  const auto& saved = msg.saved_traj;
  for (int i = 0; i < saved.num_trajectories; ++i) {
    if (saved.trajectory_names[i] == "sample_costs") {
      return saved.trajectories[i].datapoints.at(0).at(0);
    }
  }
  throw std::runtime_error("No sample_costs block in SAMPLE_COSTS");
}

bool DecodeState(const lcm::LogEvent& event, int n_x, VectorXd* out) {
  dairlib::lcmt_c3_state msg;
  if (msg.decode(event.data, 0, event.datalen) < 0) return false;
  if (msg.num_states != n_x) return false;
  out->resize(n_x);
  for (int i = 0; i < n_x; ++i) (*out)(i) = msg.state[i];
  return true;
}

// Groups every channel the controller reads or publishes by utime, keeping
// only loops that carry all of them.  All of these are published by the same
// forced-publish event, so their utimes agree exactly.
std::vector<LoggedLoop> ReadLog(const std::string& path, int n_x) {
  lcm::LogFile log(path, "r");
  if (!log.good()) throw std::runtime_error("Could not open log " + path);
  std::map<int64_t, LoggedLoop> by_utime;
  std::map<int64_t, int> have;  // bitmask of the five channels below
  dairlib::lcmt_radio_out latest_radio{};
  VectorXd latest_clean_pose;
  const lcm::LogEvent* event;
  while ((event = log.readNextEvent()) != nullptr) {
    const std::string& ch = event->channel;
    if (ch == "SAMPLING_C3_RADIO") {
      latest_radio.decode(event->data, 0, event->datalen);
      continue;
    }
    if (ch == "OBJECT_STATE_SIMULATION_CLEAN") {
      dairlib::lcmt_object_state msg;
      if (msg.decode(event->data, 0, event->datalen) < 0) continue;
      if (msg.num_positions < 7) continue;
      latest_clean_pose = Eigen::Map<const VectorXd>(msg.position.data(), 7);
      continue;
    }
    VectorXd x;
    if (ch == "C3_ACTUAL" || ch == "C3_TARGET" || ch == "C3_FINAL_TARGET") {
      dairlib::lcmt_c3_state msg;
      if (msg.decode(event->data, 0, event->datalen) < 0) continue;
      if (!DecodeState(*event, n_x, &x)) continue;
      LoggedLoop& loop = by_utime[msg.utime];
      loop.utime = msg.utime;
      if (ch == "C3_ACTUAL") {
        loop.x_actual = x;
        have[msg.utime] |= 1;
      } else if (ch == "C3_TARGET") {
        loop.x_target = x;
        have[msg.utime] |= 2;
      } else {
        loop.x_final_target = x;
        have[msg.utime] |= 4;
      }
    } else if (ch == "SAMPLE_COSTS") {
      dairlib::lcmt_timestamped_saved_traj msg;
      if (msg.decode(event->data, 0, event->datalen) < 0) continue;
      LoggedLoop& loop = by_utime[msg.utime];
      loop.logged_cost0 = CostOfSample0(msg);
      have[msg.utime] |= 8;
    } else if (ch == "SAMPLING_C3_DEBUG") {
      dairlib::lcmt_sampling_c3_debug msg;
      if (msg.decode(event->data, 0, event->datalen) < 0) continue;
      LoggedLoop& loop = by_utime[msg.utime];
      loop.goal = msg.detected_goal_changes;
      loop.is_c3 = msg.is_c3_mode;
      loop.radio = latest_radio;
      loop.clean_object_pose = latest_clean_pose;
      have[msg.utime] |= 16;
    }
  }
  std::vector<LoggedLoop> loops;
  for (auto& [utime, loop] : by_utime) {
    if (have[utime] == 31) loops.push_back(loop);
  }
  if (loops.empty()) throw std::runtime_error("No complete loops in " + path);
  const int64_t t0 = loops.front().utime;
  for (LoggedLoop& loop : loops) loop.t = (loop.utime - t0) * 1e-6;
  return loops;
}

int NearestLoop(const std::vector<LoggedLoop>& loops, double t) {
  int best = 0;
  for (int i = 0; i < static_cast<int>(loops.size()); ++i) {
    if (std::abs(loops[i].t - t) < std::abs(loops[best].t - t)) best = i;
  }
  return best;
}

// The plants the controller references.  Built once and shared by every
// controller instance below; the instances are only ever stepped one at a
// time.
struct Plants {
  DiagramBuilder<double> builder;
  MultibodyPlant<double>* plant_lcs = nullptr;
  std::unique_ptr<MultibodyPlant<drake::AutoDiffXd>> plant_lcs_ad;
  std::unique_ptr<drake::systems::Diagram<double>> diagram;
  std::unique_ptr<Context<double>> diagram_context;
  Context<double>* plant_lcs_context = nullptr;
  std::unique_ptr<Context<drake::AutoDiffXd>> plant_lcs_context_ad;
  std::vector<std::vector<SortedPair<GeometryId>>> contact_pairs;

  explicit Plants(const SamplingC3ControllerParams& params) {
    auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.0);
    plant_lcs = &plant;
    AddLCSModelsTo3DPrinterPlant(plant_lcs, &scene_graph,
                                 params.object_models);
    plant_lcs->Finalize();
    plant_lcs_ad = drake::systems::System<double>::ToAutoDiffXd(*plant_lcs);
    diagram = builder.Build();
    diagram_context = diagram->CreateDefaultContext();
    plant_lcs_context = &diagram->GetMutableSubsystemContext(
        *plant_lcs, diagram_context.get());
    plant_lcs_context_ad = plant_lcs_ad->CreateDefaultContext();
    contact_pairs = BuildConeContactPairs(*plant_lcs, params.base_names);
  }
};

// One controller instance with its own context, stepped exactly as the live
// diagram steps it:  fix the four input ports, run the forced discrete update
// (ComputePlan), read the sample costs back off the output port.
// --goal_params re-targeting.  The replay feeds the controller the LOGGED
// target and final target, so a new goal sequence would otherwise change only
// the per-goal settings.  When set, each loop whose logged final target is the
// log's goal k gets targets rebuilt for the override's goal k the way the goal
// generator builds them:  the tracked axis turned onto the goal's by the
// smallest rotation (keeping the current twist), the object sub-goal
// lookahead_step_size toward the goal, and the EE targets left as logged (they
// sit a fixed offset above the current object, not the goal).
struct Retarget {
  SamplingC3GoalParams from;  // the log's own goals
  SamplingC3GoalParams to;    // --goal_params
};
std::optional<Retarget> g_retarget;

Eigen::Vector4d RetwistedGoal(const Eigen::Ref<const VectorXd>& q_current_wxyz,
                              const Eigen::Vector4d& q_goal_wxyz,
                              const Vector3d& axis_body) {
  const Eigen::Quaterniond q_cur =
      Eigen::Quaterniond(q_current_wxyz(0), q_current_wxyz(1),
                         q_current_wxyz(2), q_current_wxyz(3))
          .normalized();
  const Eigen::Quaterniond q_goal =
      Eigen::Quaterniond(q_goal_wxyz(0), q_goal_wxyz(1), q_goal_wxyz(2),
                         q_goal_wxyz(3))
          .normalized();
  Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(
                             q_cur * axis_body, q_goal * axis_body) *
                         q_cur;
  if (q.coeffs().dot(q_goal.coeffs()) < 0) q.coeffs() *= -1;
  return Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());
}

// Returns {target, final target} for this loop, re-targeted if requested.
std::pair<VectorXd, VectorXd> TargetsFor(const struct LoggedLoop& loop);

class Stepper {
 public:
  Stepper(Plants& plants, SamplingC3ControllerParams params)
      : controller_(*plants.plant_lcs, plants.plant_lcs_context,
                    *plants.plant_lcs_ad, plants.plant_lcs_context_ad.get(),
                    plants.contact_pairs, params),
        context_(controller_.CreateDefaultContext()) {}

  // Returns the index-0 (current location) cost.
  double Step(const LoggedLoop& loop, const VectorXd& x_actual) {
    const int n_x = x_actual.size();
    TimestampedVector<double> x_lcs(n_x);
    x_lcs.SetDataVector(x_actual);
    x_lcs.set_timestamp(loop.utime * 1e-6);
    context_->SetTime(loop.utime * 1e-6);
    controller_.get_input_port_lcs_state().FixValue(context_.get(), x_lcs);
    const auto [x_target, x_final_target] = TargetsFor(loop);
    controller_.get_input_port_target().FixValue(context_.get(), x_target);
    controller_.get_input_port_final_target().FixValue(context_.get(),
                                                       x_final_target);
    controller_.get_input_port_radio().FixValue(context_.get(), loop.radio);
    controller_.CalcForcedDiscreteVariableUpdate(
        *context_, &context_->get_mutable_discrete_state());
    const auto& port = controller_.get_output_port_all_sample_costs();
    auto value = port.Allocate();
    port.Calc(*context_, value.get());
    return CostOfSample0(
        value->get_value<dairlib::lcmt_timestamped_saved_traj>());
  }

  const SamplingC3Controller& controller() const { return controller_; }
  double Step(const LoggedLoop& loop) { return Step(loop, loop.x_actual); }

  // The last Step's current-location C3 plan, and the contact descriptions
  // (one per lambda row) it was solved with -- the pair C3_FORCES_CURR is
  // built from.  Calc rather than Eval:  the plan lives in the controller, not
  // in the context, so a cached value could be stale.
  c3::systems::C3Output::C3Solution CurrPlanSolution() const {
    const auto& port = controller_.get_output_port_c3_solution_curr_plan();
    auto value = port.Allocate();
    port.Calc(*context_, value.get());
    return value->get_value<c3::systems::C3Output::C3Solution>();
  }
  std::vector<c3::multibody::LCSContactDescription> CurrPlanContacts() const {
    const auto& port = controller_.get_output_port_lcs_contact_jacobian_curr_plan();
    auto value = port.Allocate();
    port.Calc(*context_, value.get());
    return value
        ->get_value<std::vector<c3::multibody::LCSContactDescription>>();
  }

  // Brings a fresh controller to the fixture's goal step:  the controller
  // counts goal changes itself, off changes in the final target, so it is fed
  // one loop from each earlier goal in order.
  void WarmUpToGoal(const std::vector<LoggedLoop>& loops, int goal) {
    for (int g = 0; g <= goal; ++g) {
      for (const LoggedLoop& loop : loops) {
        if (loop.goal == g) {
          Step(loop);
          break;
        }
      }
    }
  }

 private:
  SamplingC3Controller controller_;
  std::unique_ptr<Context<double>> context_;
};

void PrintSpread(const std::string& name, const std::vector<double>& costs) {
  const auto [lo, hi] = std::minmax_element(costs.begin(), costs.end());
  std::vector<double> dlog;
  for (size_t i = 1; i < costs.size(); ++i) {
    dlog.push_back(std::abs(std::log10(costs[i] / costs[i - 1])));
  }
  std::sort(dlog.begin(), dlog.end());
  auto pct = [&](double p) {
    return dlog.empty() ? 0.0 : dlog[std::min(dlog.size() - 1,
                                              size_t(p * dlog.size()))];
  };
  int over3 = 0, over10 = 0;
  for (double d : dlog) {
    over3 += d > std::log10(3.0);
    over10 += d > 1.0;
  }
  std::cout << std::setprecision(4) << name << ":  n=" << costs.size()
            << "  min " << *lo << "  max " << *hi << "  max/min "
            << *hi / *lo << "  |dlog10| p50 " << pct(0.5) << " p90 "
            << pct(0.9) << "  jumps >3x " << over3 << " >10x " << over10
            << "\n    costs:";
  for (double c : costs) std::cout << " " << c;
  std::cout << std::endl;
}

// The object pose perturbed by one draw of the injector's white noise:
// position per axis, orientation as a body-frame rotation vector.
VectorXd PerturbObjectPose(const VectorXd& x, std::mt19937& rng) {
  std::normal_distribution<double> normal(0.0, 1.0);
  VectorXd out = x;
  for (int i = 0; i < 3; ++i) out(7 + i) += kNoisePositionStd(i) * normal(rng);
  Vector3d rotvec;
  for (int i = 0; i < 3; ++i) {
    rotvec(i) = kNoiseOrientationStdDeg(i) * M_PI / 180.0 * normal(rng);
  }
  Eigen::Quaterniond q(x(3), x(4), x(5), x(6));
  Eigen::Quaterniond dq(Eigen::AngleAxisd(rotvec.norm(),
                                          rotvec.norm() > 0
                                              ? Vector3d(rotvec.normalized())
                                              : Vector3d::UnitX()));
  q = (q * dq).normalized();
  out(3) = q.w();
  out(4) = q.x();
  out(5) = q.y();
  out(6) = q.z();
  return out;
}

// The sample census:  are samples on one side of the object scored worse, and
// by which term?  The candidates are the shipped sampler's own draws (redrawn
// on every call), so their distances to the object surface are the ones the
// live controller sees.  Each row is one candidate of one call.  The cost is
// split by state block using the weights ComputePlan used, with the EE blocks
// zeroed as the object-only cost types do; obj_* summing to the cost confirms
// the split.
void RunCensus(Plants& plants, const SamplingC3ControllerParams& params,
               const std::vector<LoggedLoop>& loops) {
  const MultibodyPlant<double>& plant = *plants.plant_lcs;
  const int n_q = plant.num_positions();
  const GeometryId cone_hull = plant.GetCollisionGeometriesForBody(
      plant.GetBodyByName(params.base_names.at(0)))[0];

  std::vector<double> times;
  std::stringstream stream(FLAGS_census_times);
  for (std::string item; std::getline(stream, item, ',');) {
    times.push_back(std::stod(item));
  }

  std::ofstream csv(FLAGS_census_csv);
  csv << "t,repeat,is_c3,index,x,y,z,dx,dy,dz,center_to_hull,cost,cost0,"
         "obj_quat,obj_pos,obj_w,obj_v,final_dx,final_dy,final_dz,"
         "final_rot_deg,final_pos_err,start_pos_err,final_quat_err,"
         "twist_deg,swing_deg,mis_start_deg,mis_end_deg,swing_sq_sum,"
         "obj_quat_retwist,obj_quat_project\n";

  SamplingC3ControllerParams p = params;
  p.sampling_c3_options.use_predicted_x0_c3 = false;
  p.sampling_c3_options.use_predicted_x0_repos = false;
  for (double t : times) {
    const LoggedLoop& fixture = loops[NearestLoop(loops, t)];
    Stepper stepper(plants, p);
    stepper.WarmUpToGoal(loops, fixture.goal);
    if (FLAGS_census_warmup_t >= 0) {
      stepper.Step(loops[NearestLoop(loops, FLAGS_census_warmup_t)]);
    }
    const VectorXd& x = fixture.x_actual;
    const Vector3d object_position = x.segment(7, 3);
    const Eigen::Quaterniond object_quat(x(3), x(4), x(5), x(6));
    const Vector3d target_position = fixture.x_target.segment(7, 3);
    const Eigen::Quaterniond target_quat(fixture.x_target(3),
                                         fixture.x_target(4),
                                         fixture.x_target(5),
                                         fixture.x_target(6));

    plant.SetPositions(plants.plant_lcs_context, x.head(n_q));
    const auto& query_object =
        plant.get_geometry_query_input_port()
            .Eval<drake::geometry::QueryObject<double>>(
                *plants.plant_lcs_context);
    auto center_to_hull = [&](const Vector3d& point) {
      const auto results = query_object.ComputeSignedDistanceGeometryToPoint(
          point, drake::geometry::GeometrySet(cone_hull));
      return results.empty() ? NAN : results[0].distance;
    };

    // The tracked axis, and the unit twist tangent about it at the fixture's
    // orientation:  d/d(delta) of q (x) [cos(delta/2), sin(delta/2) a].
    const Vector3d axis = params.goal_params.tracked_orientation_axis.at(0);
    const Eigen::Quaterniond twist_tangent =
        object_quat * Eigen::Quaterniond(0, axis.x(), axis.y(), axis.z());
    const Eigen::Vector4d twist_dir(twist_tangent.w(), twist_tangent.x(),
                            twist_tangent.y(), twist_tangent.z());
    const Eigen::Matrix4d P =
        Eigen::Matrix4d::Identity() -
        twist_dir.normalized() * twist_dir.normalized().transpose();

    int n_rows = 0;
    for (int r = 0; r < FLAGS_census_repeats; ++r) {
      stepper.Step(fixture, x);
      const SamplingC3Controller& c = stepper.controller();
      const auto& locations = c.sample_locations_for_testing();
      const auto& costs = c.sample_costs_for_testing();
      const auto& rollouts = c.sample_cost_rollouts_for_testing();
      std::vector<Eigen::MatrixXd> Q = c.state_cost_weights_for_testing();
      for (auto& Qi : Q) {
        Qi.block(0, 0, 3, 3).setZero();
        Qi.block(n_q, n_q, 3, 3).setZero();
      }
      const std::vector<VectorXd> x_des(Q.size(), fixture.x_target);
      for (size_t i = 0; i < locations.size() && i < rollouts.size(); ++i) {
        const auto& XX = rollouts[i];
        if (XX.size() != Q.size()) continue;
        auto term = [&](int start, int size) {
          return c3::traj_eval::TrajectoryEvaluator::ComputeQuadraticTrajectoryCost(
              start, start + size, XX, x_des, Q);
        };
        const VectorXd& x_end = XX.back();
        const Eigen::Quaterniond end_quat(x_end(3), x_end(4), x_end(5),
                                          x_end(6));
        // Twist about the tracked axis and swing of the axis, end vs start.
        const Eigen::Quaterniond start_quat(XX.front()(3), XX.front()(4),
                                            XX.front()(5), XX.front()(6));
        Eigen::Quaterniond relative = start_quat.inverse() * end_quat;
        if (relative.w() < 0) relative.coeffs() *= -1;
        const double twist_deg =
            180.0 / M_PI * 2 *
            std::atan2(Vector3d(relative.x(), relative.y(), relative.z())
                           .dot(axis),
                       relative.w());
        const double swing_deg =
            180.0 / M_PI * ComputeAxisMisalignmentAngle(end_quat, start_quat,
                                                        axis);
        // Orientation term re-priced three ways:  the exact swing-only
        // metric (unweighted), the target re-twisted to each knot (R), and
        // the block with the twist tangent projected out (P).
        double swing_sq_sum = 0, retwist = 0, project = 0;
        Vector3d hysteresis_state = Vector3d::Zero();
        for (size_t k = 0; k < XX.size(); ++k) {
          const Eigen::Quaterniond qk(XX[k](3), XX[k](4), XX[k](5), XX[k](6));
          const double mis =
              ComputeAxisMisalignmentAngle(qk, target_quat, axis);
          swing_sq_sum += mis * mis;
          const Eigen::Quaterniond qdk = ComputeAxisAlignedGoalQuaternion(
              qk, target_quat, axis, params.goal_params.angle_hysteresis,
              &hysteresis_state);
          Eigen::Vector4d qk_v(qk.w(), qk.x(), qk.y(), qk.z());
          Eigen::Vector4d qdk_v(qdk.w(), qdk.x(), qdk.y(), qdk.z());
          if (qk_v.dot(qdk_v) < 0) qdk_v *= -1;
          const Eigen::Vector4d e_old = XX[k].segment(3, 4) -
                                        fixture.x_target.segment(3, 4);
          const Eigen::Matrix4d Qk = Q[k].block(3, 3, 4, 4);
          retwist += (qk_v - qdk_v).dot(Qk * (qk_v - qdk_v));
          project += e_old.dot(P * Qk * P * e_old);
        }
        const Vector3d d = locations[i] - object_position;
        const Vector3d travel = x_end.segment(7, 3) - XX.front().segment(7, 3);
        csv << fixture.t << "," << r << "," << c.is_doing_c3_for_testing()
            << "," << i << "," << locations[i].x() << "," << locations[i].y()
            << "," << locations[i].z() << "," << d.x() << "," << d.y() << ","
            << d.z() << "," << center_to_hull(locations[i]) << "," << costs[i]
            << "," << costs[0] << "," << term(3, 4) << "," << term(7, 3)
            << "," << term(n_q + 3, 3) << "," << term(n_q + 6, 3) << ","
            << travel.x() << "," << travel.y() << "," << travel.z() << ","
            << 180.0 / M_PI *
                   end_quat.angularDistance(Eigen::Quaterniond(
                       XX.front()(3), XX.front()(4), XX.front()(5),
                       XX.front()(6)))
            << "," << (x_end.segment(7, 3) - target_position).norm() << ","
            << (object_position - target_position).norm() << ","
            << 180.0 / M_PI * end_quat.angularDistance(target_quat) << ","
            << twist_deg << "," << swing_deg << ","
            << 180.0 / M_PI *
                   ComputeAxisMisalignmentAngle(start_quat, target_quat, axis)
            << ","
            << 180.0 / M_PI *
                   ComputeAxisMisalignmentAngle(end_quat, target_quat, axis)
            << "," << swing_sq_sum << "," << retwist << "," << project
            << "\n";
        ++n_rows;
      }
    }
    std::cout << "census t=" << fixture.t
              << (fixture.is_c3 ? " (C3)" : " (repos)") << ": " << n_rows
              << " rows; object at " << 1e3 * object_position.transpose()
              << " mm, target at " << 1e3 * target_position.transpose()
              << " mm, " << 180.0 / M_PI * object_quat.angularDistance(target_quat)
              << " deg off target orientation; logged cost[0] "
              << fixture.logged_cost0 << std::endl;
  }
  std::cout << "wrote " << FLAGS_census_csv << std::endl;
}

// The contact groups, in the order GetResolvedContactPairs stacks them (the
// order of resolve_contacts_to_lists' entries).
const std::vector<std::string> kGroupNames = {"ee_ground", "finger", "ground",
                                              "ramp"};

int GroupIndex(const std::string& name) {
  const auto it = std::find(kGroupNames.begin(), kGroupNames.end(), name);
  if (it == kGroupNames.end()) {
    throw std::runtime_error("Unknown contact group '" + name + "'");
  }
  return it - kGroupNames.begin();
}

struct Variant {
  std::string name;
  std::vector<std::pair<std::string, std::string>> overrides;
};

std::vector<Variant> ParseVariants(const std::string& spec) {
  std::vector<Variant> variants;
  std::stringstream stream(spec);
  for (std::string item; std::getline(stream, item, ';');) {
    if (item.empty()) continue;
    Variant variant;
    const size_t colon = item.find(':');
    variant.name = item.substr(0, colon);
    if (colon != std::string::npos) {
      std::stringstream overrides(item.substr(colon + 1));
      for (std::string kv; std::getline(overrides, kv, ',');) {
        const size_t eq = kv.find('=');
        if (eq == std::string::npos) {
          throw std::runtime_error("Override '" + kv + "' in variant " +
                                   variant.name + " has no '='");
        }
        variant.overrides.emplace_back(kv.substr(0, eq), kv.substr(eq + 1));
      }
    }
    variants.push_back(variant);
  }
  return variants;
}

// Lambda rows per contact of group @p group under the Anitescu model.
int AnitescuRowsPerContact(const SamplingC3Options& o, int group) {
  return o.resolve_as_planar_contacts_list.at(group)
             ? 2
             : 2 * o.num_friction_directions.value();
}

// The planning entry's lambda rows [start, start + n) of contact group
// @p group, under the Anitescu model.
std::pair<int, int> GroupLambdaRows(const SamplingC3Options& o, int group) {
  const std::vector<int>& budget =
      o.resolve_contacts_to_lists.at(o.num_contacts_index);
  int start = 0;
  for (int g = 0; g < group; ++g) {
    start += budget[g] * AnitescuRowsPerContact(o, g);
  }
  return {start, budget[group] * AnitescuRowsPerContact(o, group)};
}

// The groups a '+'-joined list names; "env" is ground+ramp, "all" every group.
std::vector<int> ParseGroups(const std::string& value) {
  if (value == "all") return {0, 1, 2, 3};
  std::vector<int> groups;
  std::stringstream stream(value);
  for (std::string name; std::getline(stream, name, '+');) {
    if (name == "env") {
      groups.insert(groups.end(), {GroupIndex("ground"), GroupIndex("ramp")});
    } else {
      groups.push_back(GroupIndex(name));
    }
  }
  return groups;
}

// Applies @p variant's overrides, then round-trips the options through YAML so
// that every field derived from them (per-contact mu and friction directions,
// the per-mode C3 cost matrices) is recomputed exactly as a load would.
SamplingC3ControllerParams ApplyVariant(const SamplingC3ControllerParams& params,
                                        const Variant& variant) {
  SamplingC3ControllerParams p = params;
  SamplingC3Options& o = p.sampling_c3_options;
  // Overrides that index the planning entry's lambda rows, applied after any
  // num_contacts_index override.
  std::vector<std::pair<std::string, double>> row_scales;
  std::vector<std::pair<int, double>> u_ratios;
  std::optional<std::string> aug_rows;
  for (const auto& [key, value] : variant.overrides) {
    if (key == "object_pose" || key == "object_dz") {
      continue;  // read by RunPlanCensus
    } else if (key == "num_contacts_index") {
      o.num_contacts_index = std::stoi(value);
    } else if (key == "admm_iter") {
      o.admm_iter = std::stoi(value);
    } else if (key == "rho_scale") {
      o.rho_scale = std::stod(value);
    } else if (key == "w_G") {
      o.w_G = std::stod(value);
    } else if (key == "w_U") {
      o.w_U = std::stod(value);
    } else if (key == "w_G_position") {
      o.w_G_position = std::stod(value);
    } else if (key == "w_U_position") {
      o.w_U_position = std::stod(value);
    } else if (key == "q_orientation_scale") {
      // Object quaternion weights in both modes' Q, and the
      // quaternion-dependent term.
      for (auto* q : {&o.q_vector, &o.q_vector_position}) {
        for (int j = 3; j < 7; ++j) (*q)[j] *= std::stod(value);
      }
      o.q_quaternion_dependent_weight *= std::stod(value);
    } else if (key == "end_on_qp_step") {
      o.end_on_qp_step = value == "true" || value == "1";
    } else if (key == "scale_lcs") {
      o.scale_lcs = value == "true" || value == "1";
    } else if (key == "aug_rows") {
      aug_rows = value;
    } else if (key == "aug_scaling") {
      o.final_augmented_cost_contact_scaling = std::stod(value);
    } else if (key == "contact_model") {
      o.contact_model = value;
    } else if (key == "planning_dt_pose") {
      o.planning_dt_pose = std::stod(value);
    } else if (key == "planning_dt_position") {
      o.planning_dt_position = std::stod(value);
    } else if (key.rfind("mu_", 0) == 0) {
      o.mu_per_pair_type.at(GroupIndex(key.substr(3))) = std::stod(value);
    } else if (key.rfind("u_ratio_", 0) == 0) {
      for (int g : ParseGroups(key.substr(8))) {
        u_ratios.emplace_back(g, std::stod(value));
      }
    } else {
      row_scales.emplace_back(key, std::stod(value));
    }
  }
  const bool row_overrides =
      !row_scales.empty() || !u_ratios.empty() || aug_rows.has_value();
  if (row_overrides && o.contact_model != "anitescu") {
    throw std::runtime_error(
        "Lambda-row overrides are only defined for the Anitescu model");
  }
  for (const auto& [key, scale] : row_scales) {
    std::string weight;
    std::vector<std::vector<std::vector<double>>*> lists;
    for (const std::string suffix :
         {"_g_lambda", "_g_eta", "_u_lambda", "_u_eta"}) {
      if (key.size() > suffix.size() &&
          key.compare(key.size() - suffix.size(), suffix.size(), suffix) ==
              0) {
        weight = suffix.substr(1);
      }
    }
    if (weight == "g_lambda") {
      lists = {&o.g_lambda_list, &o.g_lambda_position_list};
    } else if (weight == "g_eta") {
      lists = {&o.g_eta_list, &o.g_eta_position_list};
    } else if (weight == "u_lambda") {
      lists = {&o.u_lambda_list, &o.u_lambda_position_list};
    } else if (weight == "u_eta") {
      lists = {&o.u_eta_list, &o.u_eta_position_list};
    } else {
      throw std::runtime_error("Unknown override '" + key + "'");
    }
    const auto [start, n] = GroupLambdaRows(
        o, GroupIndex(key.substr(0, key.size() - weight.size() - 1)));
    for (auto* list : lists) {
      std::vector<double>& row = list->at(o.num_contacts_index);
      if (start + n > static_cast<int>(row.size())) {
        throw std::runtime_error(key + ":  rows " + std::to_string(start) +
                                 "-" + std::to_string(start + n) +
                                 " overrun a list entry of length " +
                                 std::to_string(row.size()));
      }
      for (int r = start; r < start + n; ++r) row[r] *= scale;
    }
  }
  // u_eta = ratio * u_lambda on a group's rows:  the C3+ projection keeps a
  // force only while lambda sqrt(u_lambda) >= eta sqrt(u_eta).
  for (const auto& [group, ratio] : u_ratios) {
    const auto [start, n] = GroupLambdaRows(o, group);
    for (auto [lambdas, etas] :
         {std::pair{&o.u_lambda_list, &o.u_eta_list},
          std::pair{&o.u_lambda_position_list, &o.u_eta_position_list}}) {
      const std::vector<double>& u_lambda = lambdas->at(o.num_contacts_index);
      std::vector<double>& u_eta = etas->at(o.num_contacts_index);
      for (int r = start; r < start + n; ++r) u_eta.at(r) = ratio * u_lambda.at(r);
    }
  }
  // The final C3+ solve's extra consistency weight, on these groups' rows.
  if (aug_rows.has_value()) {
    std::vector<int> rows;
    for (int g : ParseGroups(*aug_rows)) {
      const auto [start, n] = GroupLambdaRows(o, g);
      for (int r = start; r < start + n; ++r) rows.push_back(r);
    }
    std::sort(rows.begin(), rows.end());
    o.final_augmented_cost_contact_indices = rows;
  }
  o = drake::yaml::LoadYamlString<SamplingC3Options>(
      drake::yaml::SaveYamlString(o));
  return p;
}

// Swaps in the log folder's own goal and progress parameters, so a fixture
// from a run with a different goal sequence (e.g. the 4-goal toe-tip runs)
// gets its own per-goal settings.  The options' per-goal position cost is
// padded with "use q_vector_position" entries to the log's goal count.
void UseLogGoalParams(const std::string& log_path,
                      SamplingC3ControllerParams* params) {
  const std::filesystem::path folder =
      std::filesystem::path(log_path).parent_path();
  for (const auto& entry : std::filesystem::directory_iterator(folder)) {
    const std::string name = entry.path().filename().string();
    if (name.rfind("goal_params_", 0) == 0) {
      params->goal_params = drake::yaml::LoadYamlFile<SamplingC3GoalParams>(
          entry.path().string());
      std::cout << "goal params from " << name << std::endl;
    } else if (name.rfind("progress_params_", 0) == 0) {
      params->progress_params =
          drake::yaml::LoadYamlFile<SamplingC3ProgressParams>(
              entry.path().string());
      std::cout << "progress params from " << name << std::endl;
    }
  }
  if (!FLAGS_goal_params.empty()) {
    const SamplingC3GoalParams log_goals = params->goal_params;
    params->goal_params =
        drake::yaml::LoadYamlFile<SamplingC3GoalParams>(FLAGS_goal_params);
    g_retarget = Retarget{log_goals, params->goal_params};
    std::cout << "goal params from " << FLAGS_goal_params << std::endl;
  }
  if (!FLAGS_progress_params.empty()) {
    params->progress_params =
        drake::yaml::LoadYamlFile<SamplingC3ProgressParams>(
            FLAGS_progress_params);
    std::cout << "progress params from " << FLAGS_progress_params
              << std::endl;
  }
  auto& sequence = params->sampling_c3_options.q_vector_position_sequence;
  const size_t num_goals =
      params->goal_params.fixed_target_position_sequence.size();
  if (sequence.has_value() && sequence->size() != num_goals) {
    sequence->resize(num_goals);
    params->sampling_c3_options =
        drake::yaml::LoadYamlString<SamplingC3Options>(
            drake::yaml::SaveYamlString(params->sampling_c3_options));
  }
  // One keep-out entry per goal, as startup requires; "" means none.
  if (params->keep_out_model_sequence.has_value()) {
    params->keep_out_model_sequence->resize(num_goals);
  }
}

// The unit tracked axis of the object, in world, from a wxyz quaternion.
std::pair<VectorXd, VectorXd> TargetsFor(const LoggedLoop& loop) {
  if (!g_retarget.has_value()) return {loop.x_target, loop.x_final_target};
  const auto& from = g_retarget->from.fixed_target_position_sequence;
  const auto& to = g_retarget->to;
  for (size_t k = 0; k < from.size() && k < to.fixed_target_position_sequence.size();
       ++k) {
    if ((loop.x_final_target.segment(7, 2) - from[k].at(0).head(2)).norm() >
        0.002) {
      continue;
    }
    const Vector3d p_goal = to.fixed_target_position_sequence[k].at(0);
    const Eigen::Vector4d q_goal = RetwistedGoal(
        loop.x_actual.segment(3, 4),
        to.fixed_target_orientation_sequence[k].at(0),
        to.tracked_orientation_axis.at(0));
    VectorXd x_final = loop.x_final_target;
    x_final.segment(3, 4) = q_goal;
    x_final.segment(7, 3) = p_goal;
    const Vector3d p_now = loop.x_actual.segment(7, 3);
    const Vector3d d = p_goal - p_now;
    VectorXd x_sub = loop.x_target;
    x_sub.segment(3, 4) = q_goal;
    x_sub.segment(7, 3) =
        d.norm() < 1e-9
            ? p_goal
            : Vector3d(p_now + std::min(to.lookahead_step_size, d.norm()) *
                                   d.normalized());
    return {x_sub, x_final};
  }
  return {loop.x_target, loop.x_final_target};
}

Vector3d TrackedAxis(const Eigen::Ref<const VectorXd>& quat_wxyz,
                     const Vector3d& axis_body) {
  const Eigen::Quaterniond q(quat_wxyz(0), quat_wxyz(1), quat_wxyz(2),
                             quat_wxyz(3));
  return q.normalized() * axis_body;
}

// The force per unit lambda that each lambda row applies to its pair's
// geometry B, as the LCS applies it.  A contact description's force basis is
// that direction (c3 builds it as the negation of the Jacobian's basis), but an
// Anitescu row's is normalized:  the LCS edge n + mu t has length
// sqrt(1 + mu^2).  Rows are laid out per @p row_contact (-1 = Stewart-Trinkle
// slack).
std::vector<Vector3d> LcsForcesOnB(
    const std::vector<c3::multibody::LCSContactDescription>& contacts,
    const std::vector<int>& row_contact, const std::vector<double>& mu,
    bool anitescu) {
  std::vector<Vector3d> forces(contacts.size(), Vector3d::Zero());
  for (size_t r = 0; r < contacts.size(); ++r) {
    const int i = row_contact[r];
    if (i < 0) continue;
    forces[r] = contacts[r].force_basis *
                (anitescu ? std::sqrt(1 + mu[i] * mu[i]) : 1.0);
  }
  return forces;
}

// The plan census; see the file comment.
void RunPlanCensus(Plants& plants, const SamplingC3ControllerParams& params,
                   const std::vector<LoggedLoop>& loops) {
  const MultibodyPlant<double>& plant = *plants.plant_lcs;
  const int n_q = plant.num_positions();
  const auto& object_body = plant.GetBodyByName(params.base_names.at(0));
  const drake::geometry::FrameId object_frame =
      plant.GetBodyFrameIdOrThrow(object_body.index());
  const Vector3d com_body = object_body.default_com();
  const Vector3d axis_body =
      params.goal_params.tracked_orientation_axis.at(0);

  std::vector<double> times;
  std::stringstream stream(FLAGS_plan_census_times);
  for (std::string item; std::getline(stream, item, ',');) {
    times.push_back(std::stod(item));
  }
  const std::vector<Variant> variants = ParseVariants(FLAGS_plan_variants);

  std::ofstream csv(FLAGS_plan_census_csv);
  csv << "t,variant,is_c3,obj_x,obj_y,obj_z,tilt0_deg,finger_gap_mm,"
         "plan_tilt_deg,plan_rot_deg,travel_x_mm,travel_y_mm,travel_z_mm,"
         "ee_dx_mm,ee_dy_mm,ee_dz_mm";
  for (const std::string& g : kGroupNames) {
    csv << ",lam_" << g << ",tau_" << g;
  }
  csv << ",ground_fz0,rollout_tilt_deg,rollout_travel_mm,cost0,step_ms,"
         "phantom_tilt_deg,phantom_travel_mm,plan_axis_z0,plan_axis_z1,"
         "rollout_axis_z1\n";

  for (double t : times) {
    const LoggedLoop& fixture = loops[NearestLoop(loops, t)];
    auto tilt_deg = [&](const Eigen::Ref<const VectorXd>& quat_wxyz) {
      return 180.0 / M_PI *
             std::acos(std::clamp(TrackedAxis(quat_wxyz, axis_body).z(),
                                  -1.0, 1.0));
    };
    std::cout << "\n=== t=" << std::fixed << std::setprecision(2)
              << fixture.t << " goal " << fixture.goal
              << (fixture.is_c3 ? " (C3)" : " (repos)") << "  object "
              << std::setprecision(1)
              << 1e3 * fixture.x_actual.segment(7, 3).transpose()
              << " mm, tilt " << tilt_deg(fixture.x_actual.segment(3, 4))
              << " deg";
    if (fixture.clean_object_pose.size() == 7) {
      std::cout << " (clean "
                << 1e3 * fixture.clean_object_pose.tail(3).transpose()
                << " mm, tilt " << tilt_deg(fixture.clean_object_pose.head(4))
                << " deg)";
    }
    std::cout << "  EE " << 1e3 * fixture.x_actual.head(3).transpose()
              << " mm\n"
              << "  variant           tilt   rot  travel xyz [mm]    "
                 "EE disp xyz [mm]    gap[mm] | sum lambda  finger/ground/"
                 "ramp | rotation-weighted torque  finger/ground/ramp (net)\n";

    for (const Variant& variant : variants) {
      SamplingC3ControllerParams p = ApplyVariant(params, variant);
      p.sampling_c3_options.use_predicted_x0_c3 = false;
      p.sampling_c3_options.use_predicted_x0_repos = false;
      VectorXd x = fixture.x_actual;
      for (const auto& [key, value] : variant.overrides) {
        if (key != "object_pose") continue;
        if (value != "clean" || fixture.clean_object_pose.size() != 7) {
          throw std::runtime_error("object_pose=" + value +
                                   " is not available at this fixture");
        }
        x.segment(3, 7) = fixture.clean_object_pose;
      }
      for (const auto& [key, value] : variant.overrides) {
        if (key == "object_dz") x(9) += std::stod(value);
      }
      const Vector3d object_position = x.segment(7, 3);
      Stepper stepper(plants, p);
      stepper.WarmUpToGoal(loops, fixture.goal);
      // The median wall time of three ComputePlan calls on the fixture, then
      // the one whose plan is read below.
      std::vector<double> step_ms;
      for (int i = 0; i < 3; ++i) {
        const auto t0 = std::chrono::steady_clock::now();
        stepper.Step(fixture, x);
        step_ms.push_back(std::chrono::duration<double, std::milli>(
                              std::chrono::steady_clock::now() - t0)
                              .count());
      }
      std::sort(step_ms.begin(), step_ms.end());
      stepper.Step(fixture, x);
      const auto solution = stepper.CurrPlanSolution();
      const std::vector<VectorXd> qp_forces =
          stepper.controller().curr_plan_qp_forces_for_testing();
      const auto contacts = stepper.CurrPlanContacts();
      const SamplingC3Options& o = p.sampling_c3_options;

      // The pairs the plan was solved with, in LCS order, to tell which
      // witness is on the object and which contact group each belongs to.
      plant.SetPositions(plants.plant_lcs_context, x.head(n_q));
      const auto pairs = SamplingC3Controller::GetResolvedContactPairs(
          plant, *plants.plant_lcs_context, plants.contact_pairs,
          o.resolve_contacts_to, o.max_contacts_per_object_geometry,
          o.contact_dedup_witness_radius.value_or(0.0),
          o.num_friction_directions_per_contact.value());
      const auto& query_object =
          plant.get_geometry_query_input_port()
              .Eval<drake::geometry::QueryObject<double>>(
                  *plants.plant_lcs_context);
      const auto& inspector = query_object.inspector();
      std::vector<int> contact_group;
      for (int g = 0; g < static_cast<int>(o.resolve_contacts_to.size());
           ++g) {
        contact_group.insert(contact_group.end(), o.resolve_contacts_to[g], g);
      }
      // Lambda row -> contact index (-1 for Stewart-Trinkle slacks).
      const std::vector<int>& nfd = o.num_friction_directions_per_contact.value();
      const int n_contacts = pairs.size();
      std::vector<int> row_contact;
      if (o.contact_model == "anitescu") {
        for (int i = 0; i < n_contacts; ++i) {
          row_contact.insert(row_contact.end(), 2 * nfd[i], i);
        }
      } else {
        row_contact.insert(row_contact.end(), n_contacts, -1);
        for (int i = 0; i < n_contacts; ++i) row_contact.push_back(i);
        for (int i = 0; i < n_contacts; ++i) {
          row_contact.insert(row_contact.end(), 2 * nfd[i], i);
        }
      }
      if (row_contact.size() != contacts.size() ||
          static_cast<int>(contact_group.size()) != n_contacts ||
          solution.lambda_sol_.rows() != static_cast<int>(contacts.size())) {
        throw std::runtime_error("Contact bookkeeping mismatch");
      }

      // The plan's object motion, and its rotation axis in world.
      const int N = solution.x_sol_.cols();
      const VectorXd x_start = solution.x_sol_.col(0).cast<double>();
      const VectorXd x_end = solution.x_sol_.col(N - 1).cast<double>();
      const Eigen::Quaterniond q_start(x_start(3), x_start(4), x_start(5),
                                       x_start(6));
      const Eigen::Quaterniond q_end(x_end(3), x_end(4), x_end(5), x_end(6));
      const Eigen::AngleAxisd rotation(q_end.normalized() *
                                       q_start.normalized().inverse());
      const Vector3d plan_axis = rotation.axis();
      const double plan_tilt =
          180.0 / M_PI *
          std::acos(std::clamp(TrackedAxis(x_start.segment(3, 4), axis_body)
                                   .dot(TrackedAxis(x_end.segment(3, 4),
                                                    axis_body)),
                               -1.0, 1.0));
      const Vector3d com =
          object_position +
          Eigen::Quaterniond(x(3), x(4), x(5), x(6)).normalized() * com_body;

      // Forces on the object by group, summed over knots with the knot-0
      // contact geometry.  A description's force basis is the force its
      // geometry A exerts on geometry B.
      std::vector<double> lam(kGroupNames.size(), 0.0);
      std::vector<double> tau(kGroupNames.size(), 0.0);
      double ground_fz0 = 0;
      double finger_gap = NAN;
      for (int i = 0; i < n_contacts; ++i) {
        if (kGroupNames[contact_group[i]] == "finger") {
          finger_gap = query_object
                           .ComputeSignedDistancePairClosestPoints(
                               pairs[i].first(), pairs[i].second())
                           .distance;
        }
      }
      const std::vector<Vector3d> force_on_b =
          LcsForcesOnB(contacts, row_contact, o.mu_per_contact.value(),
                       o.contact_model == "anitescu");
      for (size_t r = 0; r < contacts.size(); ++r) {
        const int i = row_contact[r];
        if (i < 0 || contacts[r].is_slack) continue;
        const bool object_is_b =
            inspector.GetFrameId(pairs[i].second()) == object_frame;
        const bool object_is_a =
            inspector.GetFrameId(pairs[i].first()) == object_frame;
        if (!object_is_a && !object_is_b) continue;  // EE-ground
        const double sign = object_is_b ? 1.0 : -1.0;
        const Vector3d point = object_is_b ? contacts[r].witness_point_B
                                           : contacts[r].witness_point_A;
        const int g = contact_group[i];
        // Knots 0..N-2 roll the published states out to knot N-1.
        for (int k = 0; k + 1 < N; ++k) {
          const double lambda = qp_forces[k](r);
          const Vector3d force = sign * lambda * force_on_b[r];
          lam[g] += std::abs(lambda);
          // Weighted by the steps left:  an angular impulse at knot k has
          // turned the object for N-1-k steps by the last knot.
          tau[g] += (N - 1 - k) * (point - com).cross(force).dot(plan_axis);
          if (k == 0 && kGroupNames[g] == "ground") ground_fz0 += force.z();
        }
      }

      const Vector3d travel = 1e3 * (x_end.segment(7, 3) - x_start.segment(7, 3));
      // What the cost's own rollout of this plan (index 0, the current
      // location) says the object does.
      double rollout_tilt = NAN, rollout_travel = NAN, cost0 = NAN;
      double rollout_axis_z1 = NAN;
      Vector3d rollout_ee = Vector3d::Constant(NAN);
      const auto& rollouts = stepper.controller().sample_cost_rollouts_for_testing();
      if (!rollouts.empty() && !rollouts[0].empty()) {
        const VectorXd& r0 = rollouts[0].front();
        const VectorXd& r1 = rollouts[0].back();
        rollout_tilt =
            180.0 / M_PI *
            std::acos(std::clamp(TrackedAxis(r0.segment(3, 4), axis_body)
                                     .dot(TrackedAxis(r1.segment(3, 4),
                                                      axis_body)),
                                 -1.0, 1.0));
        rollout_travel = 1e3 * (r1.segment(7, 3) - r0.segment(7, 3)).norm();
        rollout_axis_z1 = TrackedAxis(r1.segment(3, 4), axis_body).z();
        rollout_ee = 1e3 * (r1.head(3) - r0.head(3));
        cost0 = stepper.controller().sample_costs_for_testing().at(0);
      }
      const Vector3d ee_disp = 1e3 * (x_end.head(3) - x_start.head(3));
      // The tracked axis's world z at the plan's first and last knot:  for the
      // cone, how far the apex points up (+) or down (-).
      const double plan_axis_z0 =
          TrackedAxis(x_start.segment(3, 4), axis_body).z();
      const double plan_axis_z1 = TrackedAxis(x_end.segment(3, 4), axis_body).z();
      std::cout << "  " << std::left << std::setw(16) << variant.name
                << std::right << std::setprecision(1) << std::setw(6)
                << plan_tilt << std::setw(6)
                << 180.0 / M_PI * rotation.angle() << "  " << std::setw(5)
                << travel.x() << std::setw(6) << travel.y() << std::setw(6)
                << travel.z() << "    " << std::setw(5) << ee_disp.x()
                << std::setw(6) << ee_disp.y() << std::setw(6) << ee_disp.z()
                << "  " << std::setw(7) << 1e3 * finger_gap << " | "
                << std::setprecision(2) << std::setw(7) << lam[1]
                << std::setw(8) << lam[2] << std::setw(8) << lam[3] << " | "
                << std::setprecision(4) << std::setw(9) << tau[1]
                << std::setw(9) << tau[2] << std::setw(9) << tau[3]
                << "  net " << std::setw(8) << tau[1] + tau[2] + tau[3]
                << " | rollout " << std::setprecision(1) << std::setw(5)
                << rollout_tilt << " deg " << std::setw(5) << rollout_travel
                << " mm  EE " << std::setprecision(0)
                << rollout_ee.transpose() << "  cost0 " << cost0 << "  "
                << step_ms[1] << " ms  axis z " << std::setprecision(2)
                << std::showpos << plan_axis_z0 << " -> plan " << plan_axis_z1
                << " / rollout " << rollout_axis_z1 << std::noshowpos
                << std::endl;
      csv << fixture.t << "," << variant.name << "," << fixture.is_c3 << ","
          << object_position.x() << "," << object_position.y() << ","
          << object_position.z() << "," << tilt_deg(x.segment(3, 4)) << ","
          << 1e3 * finger_gap << "," << plan_tilt << ","
          << 180.0 / M_PI * rotation.angle() << "," << travel.x() << ","
          << travel.y() << "," << travel.z() << "," << ee_disp.x() << ","
          << ee_disp.y() << "," << ee_disp.z();
      for (size_t g = 0; g < kGroupNames.size(); ++g) {
        csv << "," << lam[g] << "," << tau[g];
      }
      csv << "," << ground_fz0 << "," << rollout_tilt << ","
          << rollout_travel << "," << cost0 << "," << step_ms[1] << ","
          << plan_tilt - rollout_tilt << "," << travel.norm() - rollout_travel
          << "," << plan_axis_z0 << "," << plan_axis_z1 << ","
          << rollout_axis_z1 << "\n";
    }
  }
  std::cout << "\nwrote " << FLAGS_plan_census_csv << std::endl;
}

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  auto params = drake::yaml::LoadYamlFile<SamplingC3ControllerParams>(
      "examples/sampling_c3/three_d_printer/" + FLAGS_demo_name +
      "/parameters/sampling_c3_controller_params.yaml");
  if (FLAGS_ignore_twist) {
    params.sampling_c3_options.cost_ignores_tracked_axis_twist = true;
  }
  if (!FLAGS_plan_census_times.empty()) UseLogGoalParams(FLAGS_log, &params);
  Plants plants(params);
  const int n_x =
      plants.plant_lcs->num_positions() + plants.plant_lcs->num_velocities();
  const int n_q = plants.plant_lcs->num_positions();

  const std::vector<LoggedLoop> loops = ReadLog(FLAGS_log, n_x);
  if (!FLAGS_plan_census_times.empty()) {
    RunPlanCensus(plants, params, loops);
    return 0;
  }
  if (!FLAGS_census_times.empty()) {
    RunCensus(plants, params, loops);
    return 0;
  }
  const int k = NearestLoop(loops, FLAGS_t);
  const int k_other = NearestLoop(loops, FLAGS_t_other);
  const LoggedLoop& fixture = loops[k];
  const LoggedLoop& other = loops[k_other];
  std::cout << "Fixture loop t=" << fixture.t << " goal " << fixture.goal
            << (fixture.is_c3 ? " C3" : " repos") << ", logged cost[0] "
            << fixture.logged_cost0 << "; other loop t=" << other.t
            << ", logged cost[0] " << other.logged_cost0 << std::endl;

  auto with = [&](bool predict, int threads) {
    SamplingC3ControllerParams p = params;
    p.sampling_c3_options.use_predicted_x0_c3 = predict;
    p.sampling_c3_options.use_predicted_x0_repos = predict;
    if (threads >= 0) p.sampling_c3_options.num_outer_threads = threads;
    return p;
  };
  auto repeat = [&](Stepper& stepper, const LoggedLoop& loop,
                    const VectorXd& x, int n) {
    std::vector<double> costs;
    for (int i = 0; i < n; ++i) costs.push_back(stepper.Step(loop, x));
    return costs;
  };

  // 1. Frozen, predicted x0 off, at one thread and the configured count.
  for (int threads : {1, -1}) {
    Stepper stepper(plants, with(false, threads));
    stepper.WarmUpToGoal(loops, fixture.goal);
    PrintSpread(std::string("1. frozen, predict off, threads ") +
                    (threads == 1 ? "1" : "configured"),
                repeat(stepper, fixture, fixture.x_actual, FLAGS_repeats));
  }

  // 2. Frozen, predicted x0 on.
  {
    Stepper stepper(plants, with(true, -1));
    stepper.WarmUpToGoal(loops, fixture.goal);
    PrintSpread("2. frozen, predict on",
                repeat(stepper, fixture, fixture.x_actual, FLAGS_repeats));
  }

  // 3. Window replay with the shipped settings.
  {
    Stepper stepper(plants, params);
    stepper.WarmUpToGoal(loops, fixture.goal);
    std::cout << "3. window replay (shipped settings), last loops: t, "
                 "recomputed cost[0], logged cost[0], ratio"
              << std::endl;
    const int last = std::min<int>(loops.size() - 1, k + 3);
    for (int i = 0; i <= last; ++i) {
      if (loops[i].t < fixture.t - FLAGS_window_seconds) continue;
      const double c = stepper.Step(loops[i]);
      if (loops[i].t >= fixture.t - 0.5) {
        std::cout << "    " << std::setprecision(5) << loops[i].t << "  " << c
                  << "  " << loops[i].logged_cost0 << "  "
                  << c / loops[i].logged_cost0
                  << (i == k ? "   <- fixture" : "") << std::endl;
      }
    }
  }

  // 4. Attribution:  swap in blocks of the other loop's state.
  {
    Stepper stepper(plants, with(false, -1));
    stepper.WarmUpToGoal(loops, fixture.goal);
    struct Swap {
      std::string name;
      int start, size;
    };
    const std::vector<Swap> swaps = {{"none", 0, 0},
                                     {"object pose", 3, 7},
                                     {"EE position", 0, 3},
                                     {"EE velocity", n_q, 3},
                                     {"everything", 0, n_x}};
    std::cout << "4. attribution (predict off), fixture with the other loop's"
              << std::endl;
    for (const Swap& swap : swaps) {
      VectorXd x = fixture.x_actual;
      x.segment(swap.start, swap.size) =
          other.x_actual.segment(swap.start, swap.size);
      const std::vector<double> costs = repeat(stepper, fixture, x, 3);
      std::cout << "    " << std::setw(12) << swap.name << ":";
      for (double c : costs) std::cout << " " << c;
      std::cout << std::endl;
    }
    std::cout << "    object pose delta:  position "
              << 1e3 * (fixture.x_actual.segment(7, 3) -
                        other.x_actual.segment(7, 3))
                           .norm()
              << " mm, orientation "
              << 180.0 / M_PI *
                     Eigen::Quaterniond(fixture.x_actual(3),
                                        fixture.x_actual(4),
                                        fixture.x_actual(5),
                                        fixture.x_actual(6))
                         .angularDistance(Eigen::Quaterniond(
                             other.x_actual(3), other.x_actual(4),
                             other.x_actual(5), other.x_actual(6)))
              << " deg;  EE velocity fixture "
              << fixture.x_actual.segment(n_q, 3).transpose() << " other "
              << other.x_actual.segment(n_q, 3).transpose() << std::endl;
  }

  // 5. Injector-sized noise on the object pose.
  {
    Stepper stepper(plants, with(false, -1));
    stepper.WarmUpToGoal(loops, fixture.goal);
    std::mt19937 rng(FLAGS_seed);
    std::vector<double> costs;
    for (int i = 0; i < FLAGS_noise_trials; ++i) {
      costs.push_back(
          stepper.Step(fixture, PerturbObjectPose(fixture.x_actual, rng)));
    }
    PrintSpread("5. injector noise on object pose, predict off", costs);
  }
  return 0;
}

}  // namespace
}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::DoMain(argc, argv); }
