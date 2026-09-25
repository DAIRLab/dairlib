// Hybrid replay:  does the live jam watchdog change what happened in a log?
//
// The two hardware logs behind the "Cone Wedge at the Ramp Lip" post-mortem
// each contain a stretch where the end effector drove the cone into the ramp
// step and held it there.  The controller that flew those logs had nothing that
// could notice; the one at HEAD has the watchdog, the C3 -> repositioning
// transition it triggers, and the retreat it prepends to a repositioning plan.
// This tool splices the two together into one playable log:
//
//   Phase A, up to the trip.  Every event in the source log is copied through
//     verbatim, so what plays back is exactly what happened.  Alongside it the
//     HEAD controller is stepped once per logged control loop on the logged
//     state -- open loop, its outputs discarded -- purely so its watchdog sees
//     the same approach the real one did and latches at its own moment.  This
//     is honest because the watchdog is the only behavioural difference until
//     it trips:  before that the new controller *is* the old one.
//
//   Phase B, from the trip on.  The controller keeps running, but now closed
//     loop:  the object pose still comes from the log's vision estimates, while
//     the end effector follows the controller's own published
//     TRACKING_TRAJECTORY_ACTOR.  Its outputs are what get written, on the real
//     channel names, so the retreat and the return to C3 play back in place of
//     the jam that actually happened.  Runs until the controller is back in C3
//     mode (plus --hold_after_c3_seconds), or --max_closed_loop_seconds.
//
// The result is one LCM log playable with lcm-logplayer against
// three_d_printer_visualizer, plus a CSV of the watchdog's two signals.
//
// What this is NOT.  Phase A is evidence:  the controller sees the real states
// and nothing is modelled, so when and on which term the watchdog fires is
// simply measured.  Phase B is an illustration of what it then commands, and
// rests on two approximations.  The end effector model is "the commanded
// trajectory is achieved by the next control loop", where the real rig tracks
// it with an OSC and would lag; retreating from a wedged cone is unobstructed
// motion, the case that model is least wrong for, but phase B timings are
// optimistic by roughly one tracking time constant.  And the object trace is
// the *jammed* run's, which stands in for the first seconds of a retreat and no
// longer.  In particular the stall itself is not reproducible here:  on the
// hardware the gantry was physically pinned and its measured position stopped
// changing, whereas nothing in phase B can hold the end effector back.  That is
// why --jam_guard=false is worth running as the control -- what it shows is the
// *decision* the controller would have made without the guard (it goes straight
// back to C3 and keeps closing), not the motion that would have resulted.
// Phase B is deliberately short for the same reason.
//
// Usage (see the two windows called out in the post-mortem):
//   bazel build //examples/sampling_c3/three_d_printer/test:hybrid_jam_replay
//   ./bazel-bin/examples/sampling_c3/three_d_printer/test/hybrid_jam_replay \
//       --log=/home/bibit/3d_printer/logs/2026/09_01_26/000003/hwlog-000003 \
//       --start_time=100 --out=/tmp/hybrid-000003
//
// Run it a second time with --jam_guard=false for the A/B baseline:  same
// window, same states, watchdog off, so the difference in the two output logs
// is the change and nothing else.
//
// To re-derive the thresholds rather than exercise them, use --observe_only:
// the guard is evaluated and logged but never acted on, so the whole window
// stays phase A and the csv is an uninterrupted trace of both signals across
// the jam and -- more importantly -- across the productive pushes that are the
// real false-positive risk.  Phase B cannot bound a false-positive rate.

#include <algorithm>
#include <cmath>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <archive/dairlib/lcmt_sampling_c3_debug.hpp>
#include <archive/dairlib/lcmt_sampling_c3_debug_v2.hpp>
#include <archive/dairlib/lcmt_sampling_c3_debug_v3.hpp>
#include <dairlib/lcmt_object_state.hpp>
#include <dairlib/lcmt_radio_out.hpp>
#include <dairlib/lcmt_robot_output.hpp>
#include <dairlib/lcmt_sample_buffer.hpp>
#include <dairlib/lcmt_sampling_c3_debug.hpp>
#include <dairlib/lcmt_timestamped_saved_traj.hpp>
#include <gflags/gflags.h>
#include <lcm/lcm-cpp.hpp>

#include "c3/systems/lcmt_generators/c3_output_generator.h"
#include "c3/systems/lcmt_generators/contact_force_generator.h"
#include "examples/sampling_c3/parameter_headers/lcm_channels.h"
#include "examples/sampling_c3/parameter_headers/sampling_c3_controller_params.h"
#include "examples/sampling_c3/sampling_c3_utils.h"
#include "lcm/lcm_trajectory.h"
#include "systems/controllers/sampling_based_c3_controller.h"
#include "systems/robot_lcm_systems.h"
#include "systems/senders/c3_state_sender.h"
#include "systems/senders/sample_buffer_sender.h"
#include "systems/three_d_printer_kinematics.h"

#include "drake/common/yaml/yaml_io.h"
#include "drake/lcm/drake_lcm_log.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"

namespace dairlib {
namespace {

using c3::systems::lcmt_generators::C3OutputGenerator;
using c3::systems::lcmt_generators::ContactForceGenerator;
using drake::SortedPair;
using drake::geometry::GeometryId;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::systems::DiagramBuilder;
using drake::systems::TriggerType;
using drake::systems::TriggerTypeSet;
using drake::systems::lcm::LcmPublisherSystem;
using drake::trajectories::PiecewisePolynomial;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

DEFINE_string(log, "", "Path to the source hardware log (hwlog-XXXXXX).");
DEFINE_string(out, "/tmp/hybrid_jam_replay",
              "Path to write the hybrid log to.  A .csv of the watchdog "
              "signals is written alongside it.");
DEFINE_double(
    start_time, 0.0,
    "Log-relative seconds at which to start stepping the controller "
    "on the logged state.  Pick a few seconds before the approach "
    "that jams:  early enough that the watchdog sees the whole press, "
    "late enough that the controller is not asked to reproduce the "
    "whole run.");
DEFINE_double(give_up_time, -1.0,
              "Log-relative second to stop looking for a trip.  Negative means "
              "the end of the log.");
DEFINE_double(context_seconds, 12.0,
              "How much of the source log, ending at the trip, to copy into "
              "the output verbatim.  This is the 'what actually happened' half "
              "of the hybrid log.");
DEFINE_double(max_closed_loop_seconds, 12.0,
              "Hard stop on phase B.  The object trace it replays is the "
              "jammed system's, which stops meaning anything once the real run "
              "was rescued, so keep this short.");
DEFINE_double(hold_after_c3_seconds, 1.5,
              "Once the controller is back in C3 mode, keep running this much "
              "longer before stopping.");
DEFINE_bool(jam_guard, true,
            "False drops the jam_guard block from the parameters, which is the "
            "A/B baseline:  identical window, identical states, watchdog off.  "
            "In that mode there is no trip, so phase B starts at "
            "--baseline_switch_time instead.");
DEFINE_bool(observe_only, false,
            "Evaluate and record the watchdog but never act on it:  the whole "
            "window stays open loop on the logged states, so the csv is an "
            "uninterrupted trace of both guard signals and the latch across "
            "the jam and everything around it.  This is the mode to re-derive "
            "thresholds from -- phase B cannot bound a false-positive rate, "
            "since it teleports the end effector onto its own plan and keeps "
            "replaying the jammed run's object estimates.  Unlike "
            "--jam_guard=false, which disables the guard and therefore leaves "
            "the signals unpopulated, this keeps computing them.  CAVEAT for "
            "the gap term:  the replayed gap runs 5-15 mm shallower than the "
            "live signal precisely while in contact -- over hwlog-000000 it "
            "reproduces 63 of the 479 loops the live run spent at a gap <= "
            "-1 mm and 0 of the 127 at <= -10 mm, the predicted-x0 path being "
            "the likely cause since it diverges exactly during a press.  The "
            "object-travel term does not have this problem (it matches a hand "
            "reconstruction to 0.0000 mm).  So derive gap thresholds from the "
            "logged SAMPLING_C3_DEBUG signal, not from this trace.");
DEFINE_double(baseline_switch_time, -1.0,
              "With --jam_guard=false, the log-relative second at which to "
              "switch to closed loop anyway, so the baseline covers the same "
              "window as the run being compared to it.");
DEFINE_bool(freeze_object_after_trip, false,
            "Hold the object at its trip-time pose through phase B instead of "
            "following the log.  Use when phase B would otherwise run past the "
            "point where a human rescued the real run.");
DEFINE_double(force_trip, -1.0,
              "Override the yaml's jam_guard.force_trip [N].  Negative keeps "
              "the yaml value.  To record both signals across a whole window "
              "without acting on them -- which is how you re-check the "
              "thresholds against a log -- use --observe_only instead of "
              "setting this out of reach.");
DEFINE_double(force_release, -1.0, "Override jam_guard.force_release [N].");
DEFINE_double(gap_trip, 1.0,
              "Override jam_guard.gap_trip [m].  Positive keeps the yaml "
              "value, since a real trip threshold is negative.");
DEFINE_double(gap_release, 99.0,
              "Override jam_guard.gap_release [m].  99 keeps the yaml value; "
              "unlike gap_trip this one is legitimately positive, so it cannot "
              "use the sign as its sentinel.");
DEFINE_double(force_gate_gap, 99.0,
              "Override jam_guard.force_gate_gap [m] -- the gap within which "
              "the force term is allowed to arm at all.  99 keeps the yaml "
              "value; pass a huge value for the old ungated behaviour.");
DEFINE_double(object_travel_trip, -1.0,
              "Override jam_guard.object_travel_trip [m] -- how far the object "
              "estimate may move over the window and still let the gap term "
              "arm.  Negative keeps the yaml value.  To disable the term "
              "and recover the old gap-only guard, raise BOTH this and "
              "--object_travel_release out of reach -- the controller demands "
              "release above trip and will abort if only one moves.");
DEFINE_double(object_travel_release, -1.0,
              "Override jam_guard.object_travel_release [m].  Negative keeps "
              "the yaml value.");
DEFINE_double(object_travel_window_seconds, -1.0,
              "Override jam_guard.object_travel_window_seconds.  Negative "
              "keeps the yaml value.  The term cannot arm until this has "
              "filled, so it is the floor on detection latency.");
DEFINE_double(trip_hold_seconds, -1.0,
              "Override jam_guard.trip_hold_seconds.  Negative keeps the yaml "
              "value.");
DEFINE_double(release_hold_seconds, -1.0,
              "Override jam_guard.release_hold_seconds.  Negative keeps the "
              "yaml value.");
DEFINE_int32(goal_step, -1,
             "Which step of goal_params.yaml's fixed_target_position_sequence "
             "the log is on at --start_time.  The controller derives its goal "
             "step by counting changes of the final target, so a replay that "
             "starts mid-run would otherwise sit on step 0 and apply the wrong "
             "step's keep-out geometry and cost switching threshold.  Given "
             "this, the harness walks the earlier steps' final targets past "
             "the controller first and then checks it landed on the right one. "
             " Negative skips the priming.");
DEFINE_string(demo_name, "cone", "Demo within sampling_c3/three_d_printer/.");

constexpr char kTickChannel[] = "SAMPLING_C3_DEBUG";
constexpr char kPrinterStateChannel[] = "PRINTER_STATE";
constexpr char kObjectStateChannel[] = "OBJECT_STATE";
constexpr char kRadioChannel[] = "SAMPLING_C3_RADIO";
constexpr char kTargetChannel[] = "C3_TARGET";
constexpr char kFinalTargetChannel[] = "C3_FINAL_TARGET";

// Channels that carry nothing about the experiment and would dominate the
// output file's size.
bool IsNoiseChannel(const std::string& channel) {
  return channel.rfind("PMD_", 0) == 0 || channel == "LCM_SELF_TEST";
}

// Both of these lcmtypes gained fields after these logs were recorded, so a
// verbatim copy of the old bytes would not decode against the tools at HEAD.
// Every added field was appended, which makes an old message the prefix of a
// new one:  swap in the new fingerprint and append the new fields as "no
// reading" and the result is a valid current message.  Anything else is copied
// through untouched.
std::vector<uint8_t> UpgradeIfNeeded(const std::string& channel,
                                     const void* data, int size) {
  const uint8_t* bytes = static_cast<const uint8_t*>(data);
  std::vector<uint8_t> out(bytes, bytes + size);

  auto overwrite_hash = [&out](int64_t hash) {
    for (int i = 0; i < 8; ++i) {
      out[i] = static_cast<uint8_t>((hash >> (8 * (7 - i))) & 0xFF);
    }
  };
  auto append_float = [&out](float value) {
    uint32_t raw;
    std::memcpy(&raw, &value, 4);
    for (int i = 0; i < 4; ++i) {
      out.push_back(static_cast<uint8_t>((raw >> (8 * (3 - i))) & 0xFF));
    }
  };
  const float kNoReading = std::numeric_limits<float>::quiet_NaN();

  auto append_int32 = [&out](int32_t value) {
    const uint32_t raw = static_cast<uint32_t>(value);
    for (int i = 0; i < 4; ++i) {
      out.push_back(static_cast<uint8_t>((raw >> (8 * (3 - i))) & 0xFF));
    }
  };

  if (channel == kTickChannel) {
    // Every field this type has gained was appended, so each archived
    // generation is a prefix of the next.  Identify the generation by its
    // fingerprint -- NOT by how short it is: two generations have been exactly
    // the same number of bytes short of current before, and a length rule then
    // upgrades one as the other and lands a field's bytes in its neighbour.
    // The appended tails below are cumulative, oldest generation first.
    if (size < 8) return out;
    int64_t hash = 0;
    for (int i = 0; i < 8; ++i) hash = (hash << 8) | bytes[i];
    // repos_target_decision's "no reading": 0 is a real decision ("kept the
    // incumbent"), so an upgraded log must not claim it.
    constexpr int32_t kNoDecision = -1;
    const auto append_travel = [&]() {
      append_float(kNoReading);  // jam_object_travel
    };
    const auto append_repos_target_decision = [&]() {
      append_int32(kNoDecision);
    };
    const auto append_deep_tier = [&]() {
      append_float(kNoReading);  // jam_ee_object_gap_measured
      out.push_back(0);          // jam_deep_armed
    };
    if (hash == archive::dairlib::lcmt_sampling_c3_debug::getHash()) {
      // v1: the original three jam fields, nothing after.
      append_travel();
      append_repos_target_decision();
      append_deep_tier();
    } else if (hash == archive::dairlib::lcmt_sampling_c3_debug_v2::getHash()) {
      append_repos_target_decision();
      append_deep_tier();
    } else if (hash == archive::dairlib::lcmt_sampling_c3_debug_v3::getHash()) {
      append_deep_tier();
    } else if (size + (4 + 4 + 1) + 4 + 4 + (4 + 1) ==
               dairlib::lcmt_sampling_c3_debug().getEncodedSize()) {
      // Predates the watchdog entirely, and has no archived layout to match a
      // fingerprint against; the only generation left that is this short.
      append_float(kNoReading);  // jam_ee_object_force
      append_float(kNoReading);  // jam_ee_object_gap
      out.push_back(0);          // jam_tripped
      append_travel();
      append_repos_target_decision();
      append_deep_tier();
    } else {
      return out;
    }
    overwrite_hash(dairlib::lcmt_sampling_c3_debug::getHash());
    return out;
  }
  if (channel == "SAMPLE_BUFFER" || channel == "UNSUCCESSFUL_SAMPLE_BUFFER") {
    if (size < 20) return out;
    int32_t buffer_length = 0;
    for (int i = 0; i < 4; ++i) {
      buffer_length = (buffer_length << 8) | out[16 + i];
    }
    if (buffer_length < 0 || buffer_length > 100000) return out;
    // jam_labels, jam_travel, plan_is_real, one float each per slot.
    overwrite_hash(dairlib::lcmt_sample_buffer::getHash());
    for (int i = 0; i < 3 * buffer_length; ++i) append_float(kNoReading);
    dairlib::lcmt_sample_buffer check;
    if (check.decode(out.data(), 0, out.size()) < 0) {
      return std::vector<uint8_t>(bytes, bytes + size);
    }
    return out;
  }
  return out;
}

struct RawEvent {
  int64_t timestamp = 0;
  std::string channel;
  std::vector<uint8_t> data;
};

// The end effector's position at the next control loop, under the model that
// the published trajectory is achieved.  Returns false while no plan has been
// published yet.
class EndEffectorFollower {
 public:
  void Accept(const dairlib::lcmt_timestamped_saved_traj& msg) {
    LcmTrajectory lcm_traj(msg.saved_traj);
    const LcmTrajectory::Trajectory& block =
        lcm_traj.GetTrajectory("end_effector_position_target");
    plan_ = PiecewisePolynomial<double>::FirstOrderHold(block.time_vector,
                                                        block.datapoints);
    has_plan_ = true;
  }

  bool Evaluate(double time, Vector3d* position) const {
    if (!has_plan_) return false;
    const double clamped =
        std::clamp(time, plan_.start_time(), plan_.end_time());
    *position = plan_.value(clamped);
    return true;
  }

 private:
  PiecewisePolynomial<double> plan_;
  bool has_plan_ = false;
};

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  if (FLAGS_log.empty()) {
    std::cerr << "--log is required." << std::endl;
    return 1;
  }
  if (FLAGS_demo_name != "cone") {
    throw std::runtime_error("Unknown --demo_name value: " + FLAGS_demo_name);
  }

  // --------------------------------------------------------------------------
  // Parameters.  These are the ones the demo ships today, not the ones the log
  // flew:  the question being asked is what the controller at HEAD would do,
  // and the archived per-log yamls point at repo paths anyway.
  // --------------------------------------------------------------------------
  const std::string controller_params_path =
      "examples/sampling_c3/three_d_printer/" + FLAGS_demo_name +
      "/parameters/sampling_c3_controller_params.yaml";
  SamplingC3ControllerParams controller_params =
      drake::yaml::LoadYamlFile<SamplingC3ControllerParams>(
          controller_params_path);
  const SamplingC3LcmChannels lcm_channel_params =
      drake::yaml::LoadYamlFile<SamplingC3LcmChannels>(
          controller_params.lcm_channels_hardware_file);

  if (FLAGS_observe_only && !FLAGS_jam_guard) {
    throw std::runtime_error(
        "--observe_only needs the guard enabled:  --jam_guard=false drops the "
        "jam_guard block entirely, so UpdateJamWatchdog returns early and both "
        "signals stay at their defaults (force 0, gap NaN).  Use "
        "--observe_only on its own.");
  }
  if (!FLAGS_jam_guard) {
    controller_params.progress_params.jam_guard.reset();
    std::cout << "Jam guard DISABLED -- this is the A/B baseline run."
              << std::endl;
  } else if (!controller_params.progress_params.jam_guard.has_value()) {
    throw std::runtime_error(
        "This demo's progress params configure no jam_guard, so there is "
        "nothing to replay.  Add the block or pass --jam_guard=false.");
  } else {
    JamGuardParams& guard = *controller_params.progress_params.jam_guard;
    if (FLAGS_force_trip >= 0.0) guard.force_trip = FLAGS_force_trip;
    if (FLAGS_force_release >= 0.0) guard.force_release = FLAGS_force_release;
    if (FLAGS_gap_trip < 0.0) guard.gap_trip = FLAGS_gap_trip;
    if (FLAGS_gap_release < 90.0) guard.gap_release = FLAGS_gap_release;
    if (FLAGS_force_gate_gap < 90.0) {
      guard.force_gate_gap = FLAGS_force_gate_gap;
    }
    if (FLAGS_object_travel_trip >= 0.0) {
      guard.object_travel_trip = FLAGS_object_travel_trip;
    }
    if (FLAGS_object_travel_release >= 0.0) {
      guard.object_travel_release = FLAGS_object_travel_release;
    }
    if (FLAGS_object_travel_window_seconds >= 0.0) {
      guard.object_travel_window_seconds = FLAGS_object_travel_window_seconds;
    }
    if (FLAGS_trip_hold_seconds >= 0.0) {
      guard.trip_hold_seconds = FLAGS_trip_hold_seconds;
    }
    if (FLAGS_release_hold_seconds >= 0.0) {
      guard.release_hold_seconds = FLAGS_release_hold_seconds;
    }
    std::cout << "Jam guard: trip below " << guard.gap_trip
              << " m gap while the object moves under "
              << guard.object_travel_trip << " m per "
              << guard.object_travel_window_seconds << " s, or "
              << "above " << guard.force_trip << " N within "
              << guard.force_gate_gap << " m of contact, held "
              << guard.trip_hold_seconds << " s; release under "
              << guard.force_release << " N and above " << guard.gap_release
              << " m for " << guard.release_hold_seconds << " s; retreat "
              << guard.retreat_knots << " knots at the repositioning speed"
              << " limits."
              << std::endl;
  }

  // --------------------------------------------------------------------------
  // Plants, exactly as three_d_printer_sampling_c3_controller.cc builds them.
  // --------------------------------------------------------------------------
  MultibodyPlant<double> plant_three_d_printer(0.0);
  Add3DPrinterToPlant(&plant_three_d_printer, nullptr, true);
  plant_three_d_printer.Finalize();
  auto three_d_printer_context = plant_three_d_printer.CreateDefaultContext();

  MultibodyPlant<double> plant_object(0.0);
  std::vector<ModelInstanceIndex> object_indices = AddObjectsToPlant(
      &plant_object, nullptr, controller_params.object_models);
  plant_object.Finalize();
  auto object_context = plant_object.CreateDefaultContext();

  DiagramBuilder<double> plant_lcs_builder;
  auto [plant_lcs, scene_graph] =
      AddMultibodyPlantSceneGraph(&plant_lcs_builder, 0.0);
  std::vector<ModelInstanceIndex> object_indices_lcs =
      AddLCSModelsTo3DPrinterPlant(&plant_lcs, &scene_graph,
                                   controller_params.object_models);
  plant_lcs.Finalize();
  std::unique_ptr<MultibodyPlant<drake::AutoDiffXd>> plant_lcs_autodiff =
      drake::systems::System<double>::ToAutoDiffXd(plant_lcs);
  auto plant_lcs_diagram = plant_lcs_builder.Build();
  std::unique_ptr<drake::systems::Context<double>> plant_lcs_diagram_context =
      plant_lcs_diagram->CreateDefaultContext();
  auto& plant_lcs_context = plant_lcs_diagram->GetMutableSubsystemContext(
      plant_lcs, plant_lcs_diagram_context.get());
  auto plant_lcs_context_ad = plant_lcs_autodiff->CreateDefaultContext();

  std::vector<std::vector<SortedPair<GeometryId>>> contact_pairs =
      BuildConeContactPairs(plant_lcs, controller_params.base_names);

  // The offset between printer joint space and end effector tip position, the
  // same one ThreeDPrinterInverseKinematics computes, used to write a
  // synthesized PRINTER_STATE in phase B.
  multibody::SetPositionsIfNew<double>(
      plant_three_d_printer,
      VectorXd::Zero(plant_three_d_printer.num_positions()),
      three_d_printer_context.get());
  const Vector3d end_effector_offset =
      plant_three_d_printer
          .EvalBodyPoseInWorld(
              *three_d_printer_context,
              plant_three_d_printer.GetBodyByName(k3dEndEffectorTipName))
          .translation();

  // --------------------------------------------------------------------------
  // Diagram.  The same controller and the same publishers as the live
  // controller binary, minus the goal generator:  the targets it would compute
  // are in the log already, on C3_TARGET and C3_FINAL_TARGET, and replaying
  // them keeps the controller chasing the goal the real run was chasing rather
  // than re-deriving a goal index from a mid-run object pose.
  // --------------------------------------------------------------------------
  drake::lcm::DrakeLcmLog out_log(FLAGS_out, /*is_write=*/true,
                                  /*overwrite_publish_time=*/false);
  DiagramBuilder<double> builder;

  auto robot_state_receiver =
      builder.AddSystem<systems::RobotOutputReceiver>(plant_three_d_printer);
  std::vector<systems::ObjectStateReceiver*> object_state_receivers;
  for (int i = 0; i < static_cast<int>(object_indices_lcs.size()); ++i) {
    object_state_receivers.push_back(
        builder.AddSystem<systems::ObjectStateReceiver>(
            plant_lcs, object_indices_lcs.at(i)));
  }
  auto kinematics = builder.AddSystem<systems::ThreeDPrinterKinematics>(
      plant_three_d_printer, three_d_printer_context.get(), plant_object,
      object_context.get(), k3dEndEffectorTipName,
      controller_params.base_names);
  auto controller = builder.AddSystem<systems::SamplingC3Controller>(
      plant_lcs, &plant_lcs_context, *plant_lcs_autodiff,
      plant_lcs_context_ad.get(), contact_pairs, controller_params);

  auto publisher = [&](const std::string& channel) {
    return builder.AddSystem(
        LcmPublisherSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
            channel, &out_log, TriggerTypeSet({TriggerType::kForced})));
  };
  auto actor_curr_plan_pub =
      publisher(lcm_channel_params.c3_actor_curr_plan_channel);
  auto object_curr_plan_pub =
      publisher(lcm_channel_params.c3_object_curr_plan_channel);
  auto actor_best_plan_pub =
      publisher(lcm_channel_params.c3_actor_best_plan_channel);
  auto object_best_plan_pub =
      publisher(lcm_channel_params.c3_object_best_plan_channel);
  auto c3_exec_pub =
      publisher(lcm_channel_params.c3_trajectory_exec_actor_channel);
  auto repos_exec_pub =
      publisher(lcm_channel_params.repos_trajectory_exec_actor_channel);
  auto tracking_pub =
      publisher(lcm_channel_params.tracking_trajectory_actor_channel);
  auto sample_locations_pub =
      publisher(lcm_channel_params.sample_locations_channel);
  auto sample_costs_pub = publisher(lcm_channel_params.sample_costs_channel);
  auto is_c3_mode_pub = publisher(lcm_channel_params.is_c3_mode_channel);
  auto feasible_curr_actor_pub = publisher(
      lcm_channel_params.dynamically_feasible_curr_actor_plan_channel);
  auto feasible_curr_object_pub =
      publisher(lcm_channel_params.dynamically_feasible_curr_plan_channel);
  auto feasible_best_actor_pub = publisher(
      lcm_channel_params.dynamically_feasible_best_actor_plan_channel);
  auto feasible_best_object_pub =
      publisher(lcm_channel_params.dynamically_feasible_best_plan_channel);

  auto debug_pub = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_sampling_c3_debug>(
          lcm_channel_params.sampling_c3_debug_channel, &out_log,
          TriggerTypeSet({TriggerType::kForced})));

  auto sample_buffer_sender = builder.AddSystem<systems::SampleBufferSender>(
      controller_params.sampling_params.N_sample_buffer,
      plant_lcs.num_positions(), "sample_buffer_sender");
  auto sample_buffer_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_sample_buffer>(
          lcm_channel_params.sample_buffer_channel, &out_log,
          TriggerTypeSet({TriggerType::kForced})));
  auto unsuccessful_sample_buffer_sender =
      builder.AddSystem<systems::SampleBufferSender>(
          controller_params.sampling_params.N_unsuccessful_sample_buffer,
          plant_lcs.num_positions(), "unsuccessful_sample_buffer_sender");
  auto unsuccessful_sample_buffer_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_sample_buffer>(
          lcm_channel_params.unsuccessful_sample_buffer_channel, &out_log,
          TriggerTypeSet({TriggerType::kForced})));

  std::vector<std::string> state_names = {"end_effector_x", "end_effector_y",
                                          "end_effector_z"};
  const std::vector<std::string> object_pose_names = {
      "object_qw", "object_qx", "object_qy", "object_qz",
      "object_x",  "object_y",  "object_z"};
  const std::vector<std::string> object_velo_names = {"object_wx", "object_wy",
                                                      "object_wz", "object_vx",
                                                      "object_vy", "object_vz"};
  for (int i = 0; i < controller_params.num_objects; i++) {
    for (const std::string& name : object_pose_names) {
      state_names.push_back(name + "_" + std::to_string(i));
    }
  }
  state_names.push_back("end_effector_vx");
  state_names.push_back("end_effector_vy");
  state_names.push_back("end_effector_vz");
  for (int i = 0; i < controller_params.num_objects; i++) {
    for (const std::string& name : object_velo_names) {
      state_names.push_back(name + "_" + std::to_string(i));
    }
  }
  const int n_x = plant_lcs.num_positions() + plant_lcs.num_velocities();
  auto c3_state_sender =
      builder.AddSystem<systems::C3StateSender>(n_x, state_names);
  auto c3_target_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_c3_state>(
          lcm_channel_params.c3_target_state_channel, &out_log,
          TriggerTypeSet({TriggerType::kForced})));
  auto c3_actual_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_c3_state>(
          lcm_channel_params.c3_actual_state_channel, &out_log,
          TriggerTypeSet({TriggerType::kForced})));
  auto c3_final_target_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_c3_state>(
          lcm_channel_params.c3_final_target_state_channel, &out_log,
          TriggerTypeSet({TriggerType::kForced})));

  builder.Connect(robot_state_receiver->get_output_port(),
                  kinematics->get_input_port_printer_state());
  const std::vector<const drake::systems::InputPort<double>*>
      kinematics_object_ports = kinematics->get_input_ports_object_state();
  for (int i = 0; i < controller_params.num_objects; i++) {
    builder.Connect(object_state_receivers.at(i)->get_output_port(),
                    *(kinematics_object_ports.at(i)));
  }
  builder.Connect(kinematics->get_output_port_lcs_state(),
                  controller->get_input_port_lcs_state());
  builder.Connect(kinematics->get_output_port_lcs_state(),
                  c3_state_sender->get_input_port_actual_state());

  builder.Connect(controller->get_output_port_c3_solution_curr_plan_actor(),
                  actor_curr_plan_pub->get_input_port());
  builder.Connect(controller->get_output_port_c3_solution_curr_plan_object(),
                  object_curr_plan_pub->get_input_port());
  builder.Connect(controller->get_output_port_c3_solution_best_plan_actor(),
                  actor_best_plan_pub->get_input_port());
  builder.Connect(controller->get_output_port_c3_solution_best_plan_object(),
                  object_best_plan_pub->get_input_port());
  builder.Connect(controller->get_output_port_c3_traj_execute_actor(),
                  c3_exec_pub->get_input_port());
  builder.Connect(controller->get_output_port_repos_traj_execute_actor(),
                  repos_exec_pub->get_input_port());
  builder.Connect(controller->get_output_port_traj_execute_actor(),
                  tracking_pub->get_input_port());
  builder.Connect(controller->get_output_port_all_sample_locations(),
                  sample_locations_pub->get_input_port());
  builder.Connect(controller->get_output_port_all_sample_costs(),
                  sample_costs_pub->get_input_port());
  builder.Connect(controller->get_output_port_is_c3_mode(),
                  is_c3_mode_pub->get_input_port());
  builder.Connect(controller->get_output_port_debug(),
                  debug_pub->get_input_port());
  builder.Connect(
      controller->get_output_port_dynamically_feasible_curr_plan_actor(),
      feasible_curr_actor_pub->get_input_port());
  builder.Connect(
      controller->get_output_port_dynamically_feasible_curr_plan_object(),
      feasible_curr_object_pub->get_input_port());
  builder.Connect(
      controller->get_output_port_dynamically_feasible_best_plan_actor(),
      feasible_best_actor_pub->get_input_port());
  builder.Connect(
      controller->get_output_port_dynamically_feasible_best_plan_object(),
      feasible_best_object_pub->get_input_port());
  builder.Connect(sample_buffer_sender->get_output_port_sample_buffer(),
                  sample_buffer_pub->get_input_port());
  builder.Connect(controller->get_output_port_sample_buffer_configurations(),
                  sample_buffer_sender->get_input_port_samples());
  builder.Connect(controller->get_output_port_sample_buffer_costs(),
                  sample_buffer_sender->get_input_port_sample_costs());
  builder.Connect(
      unsuccessful_sample_buffer_sender->get_output_port_sample_buffer(),
      unsuccessful_sample_buffer_pub->get_input_port());
  builder.Connect(
      controller->get_output_port_unsuccessful_sample_buffer_configurations(),
      unsuccessful_sample_buffer_sender->get_input_port_samples());
  builder.Connect(
      controller->get_output_port_unsuccessful_sample_buffer_costs(),
      unsuccessful_sample_buffer_sender->get_input_port_sample_costs());
  if (controller->publishes_jam_data()) {
    builder.Connect(controller->get_output_port_sample_buffer_jam_data(),
                    sample_buffer_sender->get_input_port_jam_data());
    builder.Connect(
        controller->get_output_port_unsuccessful_sample_buffer_jam_data(),
        unsuccessful_sample_buffer_sender->get_input_port_jam_data());
  }
  builder.Connect(c3_state_sender->get_output_port_target_c3_state(),
                  c3_target_pub->get_input_port());
  builder.Connect(c3_state_sender->get_output_port_final_target_c3_state(),
                  c3_final_target_pub->get_input_port());
  builder.Connect(c3_state_sender->get_output_port_actual_c3_state(),
                  c3_actual_pub->get_input_port());

  C3OutputGenerator::AddLcmPublisherToBuilder(
      builder, controller->get_output_port_c3_solution_curr_plan(),
      controller->get_output_port_c3_intermediates_curr_plan(),
      lcm_channel_params.c3_debug_output_curr_channel, &out_log,
      TriggerTypeSet({TriggerType::kForced}));
  ContactForceGenerator::AddLcmPublisherToBuilder(
      builder, controller->get_output_port_c3_solution_curr_plan(),
      controller->get_output_port_lcs_contact_jacobian_curr_plan(),
      lcm_channel_params.c3_force_curr_channel, &out_log,
      TriggerTypeSet({TriggerType::kForced}));
  C3OutputGenerator::AddLcmPublisherToBuilder(
      builder, controller->get_output_port_c3_solution_best_plan(),
      controller->get_output_port_c3_intermediates_best_plan(),
      lcm_channel_params.c3_debug_output_best_channel, &out_log,
      TriggerTypeSet({TriggerType::kForced}));
  ContactForceGenerator::AddLcmPublisherToBuilder(
      builder, controller->get_output_port_c3_solution_best_plan(),
      controller->get_output_port_lcs_contact_jacobian_best_plan(),
      lcm_channel_params.c3_force_best_channel, &out_log,
      TriggerTypeSet({TriggerType::kForced}));

  auto diagram = builder.Build();
  diagram->set_name("hybrid_jam_replay");
  auto root_context = diagram->CreateDefaultContext();

  auto& robot_receiver_context = diagram->GetMutableSubsystemContext(
      *robot_state_receiver, root_context.get());
  auto& controller_context =
      diagram->GetMutableSubsystemContext(*controller, root_context.get());
  auto& c3_state_sender_context =
      diagram->GetMutableSubsystemContext(*c3_state_sender, root_context.get());

  // A final-target vector as the goal generator would mux it:  end effector
  // target, then each object's pose, then the velocity targets.  Only the
  // object block is compared when the controller decides a goal changed, so the
  // rest can be anything.
  auto final_target_for_goal_step = [&](int step) {
    VectorXd target = VectorXd::Zero(n_x);
    for (int i = 0; i < controller_params.num_objects; ++i) {
      const Eigen::Vector4d& quaternion =
          controller_params.goal_params.fixed_target_orientation_sequence
              .at(step)
              .at(i);
      const Vector3d& position =
          controller_params.goal_params.fixed_target_position_sequence.at(step)
              .at(i);
      target.segment(3 + 7 * i, 4) = quaternion;
      target.segment(3 + 7 * i + 4, 3) = position;
    }
    return target;
  };

  // --------------------------------------------------------------------------
  // Replay.
  // --------------------------------------------------------------------------
  lcm::LogFile source(FLAGS_log, "r");
  if (!source.good()) {
    std::cerr << "Could not open log: " << FLAGS_log << std::endl;
    return 1;
  }

  enum class Phase { kSeeking, kOpenLoop, kClosedLoop, kDone };
  Phase phase = Phase::kSeeking;

  int64_t log_start_utime = -1;
  double trip_time = std::numeric_limits<double>::quiet_NaN();
  double closed_loop_start = std::numeric_limits<double>::quiet_NaN();
  double back_in_c3_since = std::numeric_limits<double>::quiet_NaN();

  dairlib::lcmt_robot_output latest_printer_state{};
  std::vector<dairlib::lcmt_object_state> latest_object_states(
      controller_params.num_objects);
  std::vector<RawEvent> latest_object_events(controller_params.num_objects);
  dairlib::lcmt_radio_out latest_radio{};
  VectorXd latest_target = VectorXd::Zero(n_x);
  VectorXd latest_final_target = VectorXd::Zero(n_x);
  bool have_printer_state = false;
  bool have_target = false;
  std::vector<bool> have_object_state(controller_params.num_objects, false);

  std::deque<RawEvent> context_ring;
  EndEffectorFollower follower;
  Vector3d closed_loop_ee = Vector3d::Zero();

  std::ofstream csv(FLAGS_out + ".csv");
  csv << "t_log,phase,is_c3_mode,mode_switch_reason,jam_force_N,jam_gap_m,"
         "jam_object_travel_m,jam_tripped,ee_x,ee_y,ee_z,object_x,object_y,"
         "object_z\n";
  csv << std::setprecision(9);

  auto decode_c3_state = [n_x](const void* data, int size, VectorXd* out) {
    dairlib::lcmt_c3_state msg;
    if (msg.decode(data, 0, size) < 0) return false;
    if (msg.num_states != n_x) return false;
    for (int i = 0; i < n_x; ++i) (*out)(i) = msg.state[i];
    return true;
  };

  int steps = 0;
  bool checked_goal_step = false;
  auto run_one_loop = [&](double t_context, const VectorXd& target,
                          const VectorXd& final_target) {
    root_context->SetTime(t_context);
    robot_state_receiver->get_input_port().FixValue(&robot_receiver_context,
                                                    latest_printer_state);
    for (int i = 0; i < controller_params.num_objects; ++i) {
      auto& object_receiver_context = diagram->GetMutableSubsystemContext(
          *object_state_receivers.at(i), root_context.get());
      object_state_receivers.at(i)->get_input_port().FixValue(
          &object_receiver_context, latest_object_states.at(i));
    }
    controller->get_input_port_radio().FixValue(&controller_context,
                                                latest_radio);
    controller->get_input_port_target().FixValue(&controller_context, target);
    controller->get_input_port_final_target().FixValue(&controller_context,
                                                       final_target);
    c3_state_sender->get_input_port_target_state().FixValue(
        &c3_state_sender_context, target);
    c3_state_sender->get_input_port_final_target_state().FixValue(
        &c3_state_sender_context, final_target);

    diagram->CalcForcedUnrestrictedUpdate(*root_context,
                                          &root_context->get_mutable_state());
    diagram->CalcForcedDiscreteVariableUpdate(
        *root_context, &root_context->get_mutable_discrete_state());
    ++steps;
  };

  const lcm::LogEvent* event = nullptr;
  while ((event = source.readNextEvent()) != nullptr && phase != Phase::kDone) {
    if (log_start_utime < 0) log_start_utime = event->timestamp;
    const double t_log = (event->timestamp - log_start_utime) * 1e-6;
    const double t_context = event->timestamp * 1e-6;
    const std::string channel = event->channel;

    // Keep the latest of every input the controller needs.
    if (channel == kPrinterStateChannel) {
      if (latest_printer_state.decode(event->data, 0, event->datalen) >= 0) {
        have_printer_state = true;
      }
    } else if (channel.rfind(kObjectStateChannel, 0) == 0 &&
               channel == kObjectStateChannel) {
      if (latest_object_states.at(0).decode(event->data, 0, event->datalen) >=
          0) {
        have_object_state.at(0) = true;
        latest_object_events.at(0) = RawEvent{
            event->timestamp, channel,
            std::vector<uint8_t>(
                static_cast<const uint8_t*>(event->data),
                static_cast<const uint8_t*>(event->data) + event->datalen)};
      }
    } else if (channel == kRadioChannel && phase != Phase::kClosedLoop) {
      latest_radio.decode(event->data, 0, event->datalen);
    } else if (channel == kTargetChannel) {
      decode_c3_state(event->data, event->datalen, &latest_target);
      have_target = true;
    } else if (channel == kFinalTargetChannel) {
      decode_c3_state(event->data, event->datalen, &latest_final_target);
    }

    // Phase A keeps a rolling window of the source log, ready to become the
    // "what actually happened" half of the output the moment the guard trips.
    if (phase != Phase::kClosedLoop && !IsNoiseChannel(channel)) {
      context_ring.push_back(
          RawEvent{event->timestamp, channel,
                   UpgradeIfNeeded(channel, event->data, event->datalen)});
      while (!context_ring.empty() &&
             (event->timestamp - context_ring.front().timestamp) * 1e-6 >
                 FLAGS_context_seconds) {
        context_ring.pop_front();
      }
    }

    // In closed loop the object estimate is still the log's, so its events keep
    // flowing into the output.
    if (phase == Phase::kClosedLoop && channel == kObjectStateChannel &&
        !FLAGS_freeze_object_after_trip) {
      out_log.Publish(channel, event->data, event->datalen, t_context);
    }

    if (channel != kTickChannel) continue;
    if (t_log < FLAGS_start_time) continue;
    if (!have_printer_state || !have_object_state.at(0) || !have_target) {
      continue;
    }
    if (FLAGS_give_up_time > 0 && t_log > FLAGS_give_up_time &&
        phase != Phase::kClosedLoop) {
      std::cout << "Reached --give_up_time=" << FLAGS_give_up_time
                << " with no trip." << std::endl;
      break;
    }
    if (phase == Phase::kSeeking) {
      phase = Phase::kOpenLoop;
      root_context->SetTime(t_context);
      std::cout << "Stepping the controller on logged state from t=" << t_log
                << " s." << std::endl;
      // Walk the controller through goal changes until its own counter sits
      // one below the step the log was on, so that the first logged target --
      // which is a different goal again -- lands it exactly there.  The
      // counter is the controller's private business (it advances on a change
      // of the final target, and the first one after construction does not
      // count), so drive it by watching it rather than by assuming a mapping.
      for (int attempt = 0; attempt < 10 && FLAGS_goal_step > 0; ++attempt) {
        const VectorXd primer = final_target_for_goal_step(attempt % 2);
        run_one_loop(t_context, primer, primer);
        if (controller->get_output_port_debug()
                .Eval<dairlib::lcmt_sampling_c3_debug>(controller_context)
                .detected_goal_changes >= FLAGS_goal_step - 1) {
          break;
        }
      }
    }

    // ---- one control loop -------------------------------------------------
    if (phase == Phase::kClosedLoop) {
      // The end effector is wherever last loop's published plan says it should
      // be by now.
      Vector3d next_ee;
      if (follower.Evaluate(t_context, &next_ee)) {
        closed_loop_ee = next_ee;
      }
      const VectorXd joint_positions = closed_loop_ee - end_effector_offset;
      for (int i = 0; i < latest_printer_state.num_positions; ++i) {
        latest_printer_state.position[i] = joint_positions(i);
        // The hardware's PRINTER_STATE carries zero velocities, so the LCS
        // state the real controller saw had a zero end effector velocity.
        // Matching that keeps phase B comparable to phase A.
        latest_printer_state.velocity[i] = 0.0;
      }
      latest_printer_state.utime = event->timestamp;
      // The synthesized robot state is an output of phase B, not an input, so
      // it has to be written explicitly -- nothing in the diagram publishes it.
      // Without it a playback would show the end effector frozen wherever the
      // source log left it.
      std::vector<uint8_t> encoded(latest_printer_state.getEncodedSize());
      latest_printer_state.encode(encoded.data(), 0, encoded.size());
      out_log.Publish(lcm_channel_params.robot_state_channel, encoded.data(),
                      static_cast<int>(encoded.size()), t_context);
    }

    run_one_loop(t_context, latest_target, latest_final_target);

    const auto& debug =
        controller->get_output_port_debug()
            .Eval<dairlib::lcmt_sampling_c3_debug>(controller_context);

    if (!checked_goal_step) {
      checked_goal_step = true;
      if (FLAGS_goal_step >= 0 &&
          debug.detected_goal_changes != FLAGS_goal_step) {
        std::cout << "!!! WARNING !!! Priming left the controller on goal step "
                  << debug.detected_goal_changes
                  << ", not the --goal_step=" << FLAGS_goal_step
                  << " asked for.  The per-goal keep-out geometry and cost "
                     "switching threshold in play are not the ones the log ran "
                     "with."
                  << std::endl;
      } else if (FLAGS_goal_step >= 0) {
        std::cout << "Controller entered the window on goal step "
                  << debug.detected_goal_changes << "." << std::endl;
      }
    }

    if (phase == Phase::kClosedLoop) {
      diagram->ForcedPublish(*root_context);
      follower.Accept(
          controller->get_output_port_traj_execute_actor()
              .Eval<dairlib::lcmt_timestamped_saved_traj>(controller_context));
    }

    const VectorXd x_lcs = kinematics->get_output_port_lcs_state().Eval(
        diagram->GetMutableSubsystemContext(*kinematics, root_context.get()));
    csv << t_log << ','
        << (phase == Phase::kClosedLoop ? "closed_loop" : "open_loop") << ','
        << static_cast<int>(debug.is_c3_mode) << ',' << debug.mode_switch_reason
        << ',' << debug.jam_ee_object_force << ',' << debug.jam_ee_object_gap
        << ',' << debug.jam_object_travel << ','
        << static_cast<int>(debug.jam_tripped) << ',' << x_lcs(0) << ','
        << x_lcs(1) << ',' << x_lcs(2) << ',' << x_lcs(7) << ',' << x_lcs(8)
        << ',' << x_lcs(9) << '\n';

    // ---- phase transitions ------------------------------------------------
    const bool should_close_loop =
        !FLAGS_observe_only &&
        ((FLAGS_jam_guard && debug.jam_tripped) ||
         (!FLAGS_jam_guard && FLAGS_baseline_switch_time > 0 &&
          t_log >= FLAGS_baseline_switch_time));
    if (phase == Phase::kOpenLoop && should_close_loop) {
      trip_time = t_log;
      closed_loop_start = t_log;
      std::cout << "\n=== "
                << (FLAGS_jam_guard ? "JAM DETECTED"
                                    : "BASELINE "
                                      "SWITCH")
                << " at t=" << t_log << " s ===" << std::endl;
      std::cout << "    EE<->object force " << debug.jam_ee_object_force
                << " N, gap " << debug.jam_ee_object_gap << " m, mode "
                << (debug.is_c3_mode ? "C3" : "repositioning") << std::endl;
      std::cout << "    Copying " << context_ring.size()
                << " source events (the " << FLAGS_context_seconds
                << " s leading up to it) into the output." << std::endl;
      for (const RawEvent& past : context_ring) {
        out_log.Publish(past.channel, past.data.data(),
                        static_cast<int>(past.data.size()),
                        past.timestamp * 1e-6);
      }
      context_ring.clear();
      phase = Phase::kClosedLoop;
      // Seed the end effector where the log left it, and publish this loop's
      // outputs so the closed-loop section starts at the trip rather than one
      // loop after it.
      closed_loop_ee = x_lcs.head(3);
      diagram->ForcedPublish(*root_context);
      follower.Accept(
          controller->get_output_port_traj_execute_actor()
              .Eval<dairlib::lcmt_timestamped_saved_traj>(controller_context));
    } else if (phase == Phase::kClosedLoop) {
      if (debug.is_c3_mode && !debug.jam_tripped) {
        if (std::isnan(back_in_c3_since)) {
          back_in_c3_since = t_log;
          std::cout << "Back in C3 mode at t=" << t_log << " s ("
                    << (t_log - trip_time) << " s after the trip)."
                    << std::endl;
        }
        if (t_log - back_in_c3_since >= FLAGS_hold_after_c3_seconds) {
          phase = Phase::kDone;
        }
      } else {
        back_in_c3_since = std::numeric_limits<double>::quiet_NaN();
      }
      if (t_log - closed_loop_start >= FLAGS_max_closed_loop_seconds) {
        std::cout << "Stopping at --max_closed_loop_seconds." << std::endl;
        phase = Phase::kDone;
      }
    }
  }

  csv.close();
  std::cout << "\nStepped " << steps << " control loops." << std::endl;
  if (FLAGS_observe_only) {
    std::cout << "--observe_only:  the whole window ran open loop on the "
                 "logged states.  The watchdog trace is in "
              << FLAGS_out << ".csv; no hybrid log was written." << std::endl;
  } else if (std::isnan(trip_time)) {
    std::cout << "No trip in the window searched -- the output log holds only "
                 "the watchdog trace in "
              << FLAGS_out << ".csv." << std::endl;
  } else {
    std::cout << "Wrote hybrid log to " << FLAGS_out << "\n"
              << "  verbatim source events: up to t=" << trip_time << " s\n"
              << "  new controller:         from t=" << trip_time << " s\n"
              << "Play it back with:\n"
              << "  lcm-logplayer " << FLAGS_out << "\n"
              << "against examples/sampling_c3/three_d_printer/"
                 "three_d_printer_visualizer (hardware channels)."
              << std::endl;
  }
  std::cout << "Watchdog trace: " << FLAGS_out << ".csv" << std::endl;
  return 0;
}

}  // namespace
}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::DoMain(argc, argv); }
