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
// Usage:
//   bazel build //examples/sampling_c3/three_d_printer/test:sample_cost_repeatability
//   ./bazel-bin/examples/sampling_c3/three_d_printer/test/sample_cost_repeatability \
//       --log=/home/bibit/3d_printer/logs/2026/09_24_26/000010/simlog-000010 \
//       --t=167.27 --t_other=167.21

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <dairlib/lcmt_c3_state.hpp>
#include <dairlib/lcmt_radio_out.hpp>
#include <dairlib/lcmt_sampling_c3_debug.hpp>
#include <dairlib/lcmt_timestamped_saved_traj.hpp>
#include <gflags/gflags.h>
#include <lcm/lcm-cpp.hpp>

#include "examples/sampling_c3/parameter_headers/sampling_c3_controller_params.h"
#include "examples/sampling_c3/sampling_c3_utils.h"
#include "systems/controllers/sampling_based_c3_controller.h"
#include "systems/framework/timestamped_vector.h"

#include "drake/common/yaml/yaml_io.h"
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
  const lcm::LogEvent* event;
  while ((event = log.readNextEvent()) != nullptr) {
    const std::string& ch = event->channel;
    if (ch == "SAMPLING_C3_RADIO") {
      latest_radio.decode(event->data, 0, event->datalen);
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
    controller_.get_input_port_target().FixValue(context_.get(),
                                                 loop.x_target);
    controller_.get_input_port_final_target().FixValue(context_.get(),
                                                       loop.x_final_target);
    controller_.get_input_port_radio().FixValue(context_.get(), loop.radio);
    controller_.CalcForcedDiscreteVariableUpdate(
        *context_, &context_->get_mutable_discrete_state());
    const auto& port = controller_.get_output_port_all_sample_costs();
    auto value = port.Allocate();
    port.Calc(*context_, value.get());
    return CostOfSample0(
        value->get_value<dairlib::lcmt_timestamped_saved_traj>());
  }
  double Step(const LoggedLoop& loop) { return Step(loop, loop.x_actual); }

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

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  auto params = drake::yaml::LoadYamlFile<SamplingC3ControllerParams>(
      "examples/sampling_c3/three_d_printer/" + FLAGS_demo_name +
      "/parameters/sampling_c3_controller_params.yaml");
  Plants plants(params);
  const int n_x =
      plants.plant_lcs->num_positions() + plants.plant_lcs->num_velocities();
  const int n_q = plants.plant_lcs->num_positions();

  const std::vector<LoggedLoop> loops = ReadLog(FLAGS_log, n_x);
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
