#include <gflags/gflags.h>

#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_target_standing_height.hpp"
#include "examples/Cassie/cassie_utils.h"
#include "examples/Cassie/osc/osc_standing_gains.h"
#include "examples/Cassie/osc/standing_com_traj.h"
#include "examples/Cassie/osc/standing_pelvis_orientation_traj.h"
#include "examples/Cassie/systems/cassie_out_to_radio.h"
#include "multibody/kinematic/kinematic_evaluator_set.h"
#include "multibody/multibody_utils.h"
#include "systems/controllers/osc/com_tracking_data.h"
#include "systems/controllers/osc/joint_space_tracking_data.h"
#include "systems/controllers/osc/operational_space_control.h"
#include "systems/controllers/osc/options_tracking_data.h"
#include "systems/controllers/osc/osc_gains.h"
#include "systems/controllers/osc/rot_space_tracking_data.h"
#include "systems/controllers/osc/trans_space_tracking_data.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/robot_lcm_systems.h"

#include "drake/common/yaml/yaml_io.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/primitives/zero_order_hold.h"

namespace dairlib {

using std::cout;
using std::endl;

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using drake::geometry::SceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::DiagramBuilder;
using drake::systems::TriggerType;
using drake::systems::TriggerTypeSet;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;

using systems::controllers::ComTrackingData;
using systems::controllers::JointSpaceTrackingData;
using systems::controllers::RotTaskSpaceTrackingData;
using systems::controllers::TransTaskSpaceTrackingData;

DEFINE_string(channel_x, "CASSIE_STATE_SIMULATION",
              "LCM channel for receiving state. "
              "Use CASSIE_STATE_SIMULATION to get state from simulator, and "
              "use CASSIE_STATE_DISPATCHER to get state from state estimator");
DEFINE_string(channel_u, "CASSIE_INPUT",
              "The name of the channel which publishes command");
DEFINE_double(max_qp_hz, 800.0, "Maximum control frequency");
DEFINE_string(
    cassie_out_channel, "CASSIE_OUTPUT_ECHO",
    "The name of the channel to receive the cassie out structure from.");
DEFINE_string(gains_filename, "examples/Cassie/osc/osc_standing_gains.yaml",
              "Filepath containing gains");
DEFINE_double(height, .8, "The initial COM height (m)");
DEFINE_bool(use_radio, false, "use the radio to set height or not");
DEFINE_bool(publish_osc, false,
            "only publish debug if needed to avoid excess LCM traffic "
            "when running many controllers");

DEFINE_double(qp_time_limit, 0.002, "maximum qp solve time");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Build Cassie MBP
  drake::multibody::MultibodyPlant<double> plant_w_springs(0.0);
  AddCassieMultibody(&plant_w_springs, nullptr, true /*floating base*/,
                     "examples/Cassie/urdf/cassie_v2.urdf",
                     true /*spring model*/, false /*loop closure*/);
  plant_w_springs.Finalize();
  // Build fix-spring Cassie MBP
  drake::multibody::MultibodyPlant<double> plant_wo_springs(0.0);
  AddCassieMultibody(&plant_wo_springs, nullptr, true,
                     "examples/Cassie/urdf/cassie_fixed_springs.urdf", false,
                     false);
  plant_wo_springs.Finalize();

  auto context_w_spr = plant_w_springs.CreateDefaultContext();
  auto context_wo_spr = plant_wo_springs.CreateDefaultContext();

  // Get contact frames and position (doesn't matter whether we use
  // plant_w_springs or plant_wo_springs because the contact frames exit in both
  // plants)
  auto left_toe = LeftToeFront(plant_wo_springs);
  auto left_heel = LeftToeRear(plant_wo_springs);
  auto right_toe = RightToeFront(plant_wo_springs);
  auto right_heel = RightToeRear(plant_wo_springs);

  // Build the controller diagram
  DiagramBuilder<double> builder;

  drake::lcm::DrakeLcm lcm_local;
  drake::yaml::LoadYamlOptions yaml_options;
  yaml_options.allow_yaml_with_no_cpp = true;
  OSCGains gains = drake::yaml::LoadYamlFile<OSCGains>(
      FindResourceOrThrow(FLAGS_gains_filename), {}, {}, yaml_options);
  OSCStandingGains osc_gains = drake::yaml::LoadYamlFile<OSCStandingGains>(
      FindResourceOrThrow(FLAGS_gains_filename));

  // Create Lcm subsriber for lcmt_target_standing_height
  auto target_height_receiver = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_target_standing_height>(
          "TARGET_HEIGHT", &lcm_local));

  // Create state receiver.
  auto state_receiver =
      builder.AddSystem<systems::RobotOutputReceiver>(plant_w_springs);

  auto cassie_out_receiver =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_cassie_out>(
          FLAGS_cassie_out_channel, &lcm_local));
  auto cassie_out_to_radio = builder.AddSystem<systems::CassieOutToRadio>();
  builder.Connect(*cassie_out_receiver, *cassie_out_to_radio);

  // Create command sender.
  auto command_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          FLAGS_channel_u, &lcm_local,
          TriggerTypeSet({TriggerType::kPeriodic}), 1.0 / FLAGS_max_qp_hz));
  auto command_sender =
      builder.AddSystem<systems::RobotCommandSender>(plant_w_springs);

  builder.Connect(*command_sender, *command_pub);


  // Create desired center of mass traj
  std::vector<std::pair<const Vector3d, const drake::multibody::Frame<double>&>>
      feet_contact_points = {left_toe, left_heel, right_toe, right_heel};
  auto com_traj_generator = builder.AddSystem<cassie::osc::StandingComTraj>(
      plant_w_springs, context_w_spr.get(), feet_contact_points, FLAGS_height,
      FLAGS_use_radio);
  auto pelvis_rot_traj_generator =
      builder.AddSystem<cassie::osc::StandingPelvisOrientationTraj>(
          plant_w_springs, context_w_spr.get(), feet_contact_points,
          "pelvis_rot_traj");
  builder.Connect(state_receiver->get_output_port(0),
                  com_traj_generator->get_input_port_state());
  builder.Connect(state_receiver->get_output_port(0),
                  pelvis_rot_traj_generator->get_input_port_state());
  builder.Connect(cassie_out_to_radio->get_output_port(),
                  pelvis_rot_traj_generator->get_input_port_radio());
  builder.Connect(cassie_out_to_radio->get_output_port(),
                  com_traj_generator->get_input_port_radio());
  builder.Connect(target_height_receiver->get_output_port(),
                  com_traj_generator->get_input_port_target_height());

  // Create Operational space control
  auto osc = builder.AddSystem<systems::controllers::OperationalSpaceControl>(
      plant_w_springs, plant_wo_springs, context_w_spr.get(),
      context_wo_spr.get(), false);

  // Distance constraint
  multibody::KinematicEvaluatorSet<double> evaluators(plant_wo_springs);
  auto left_loop = LeftLoopClosureEvaluator(plant_wo_springs);
  auto right_loop = RightLoopClosureEvaluator(plant_wo_springs);
  evaluators.add_evaluator(&left_loop);
  evaluators.add_evaluator(&right_loop);
  osc->AddKinematicConstraint(&evaluators);
  // Soft constraint
  // We don't want w_contact_relax to be too big, cause we want tracking
  // error to be important
  double w_contact_relax = gains.w_soft_constraint;
  osc->SetContactSoftConstraintWeight(w_contact_relax);
  // Friction coefficient
  double mu = 0.8;
  osc->SetContactFriction(mu);
  // Add contact points
  auto left_toe_evaluator = multibody::WorldPointEvaluator(
      plant_wo_springs, left_toe.first, left_toe.second, Matrix3d::Identity(),
      Vector3d::Zero(), {1, 2});
  osc->AddContactPoint(&left_toe_evaluator);
  auto left_heel_evaluator = multibody::WorldPointEvaluator(
      plant_wo_springs, left_heel.first, left_heel.second, Matrix3d::Identity(),
      Vector3d::Zero(), {0, 1, 2});
  osc->AddContactPoint(&left_heel_evaluator);
  auto right_toe_evaluator = multibody::WorldPointEvaluator(
      plant_wo_springs, right_toe.first, right_toe.second, Matrix3d::Identity(),
      Vector3d::Zero(), {1, 2});
  osc->AddContactPoint(&right_toe_evaluator);
  auto right_heel_evaluator = multibody::WorldPointEvaluator(
      plant_wo_springs, right_heel.first, right_heel.second,
      Matrix3d::Identity(), Vector3d::Zero(), {0, 1, 2});
  osc->AddContactPoint(&right_heel_evaluator);
  // Cost
  int n_v = plant_wo_springs.num_velocities();
  MatrixXd Q_accel = gains.w_accel * MatrixXd::Identity(n_v, n_v);
  osc->SetAccelerationCostWeights(Q_accel);

  auto pelvis_tracking_data = std::make_unique<TransTaskSpaceTrackingData>(
      "com_traj", osc_gains.K_p_pelvis, osc_gains.K_d_pelvis,
      osc_gains.W_pelvis, plant_w_springs, plant_wo_springs);
  pelvis_tracking_data->AddPointToTrack("pelvis");
  double cutoff_freq = 5;  // in Hz
  double tau = 1 / (2 * M_PI * cutoff_freq);
  pelvis_tracking_data->SetLowPassFilter(tau, {1});
  osc->AddTrackingData(std::move(pelvis_tracking_data));
  auto pelvis_rot_tracking_data = std::make_unique<RotTaskSpaceTrackingData>(
      "pelvis_rot_traj", osc_gains.K_p_pelvis_rot, osc_gains.K_d_pelvis_rot,
      osc_gains.W_pelvis_rot, plant_w_springs, plant_wo_springs);
  pelvis_rot_tracking_data->AddFrameToTrack("pelvis");
  osc->AddTrackingData(std::move(pelvis_rot_tracking_data));

  // Hip yaw joint tracking
  // We use hip yaw joint tracking instead of pelvis yaw tracking because the
  // foot sometimes rotates about yaw, and we need hip yaw joint to go back to
  // 0.
  double w_hip_yaw = 0.5;
  double hip_yaw_kp = 40;
  double hip_yaw_kd = 0.5;
  auto left_hip_yaw_traj = std::make_unique<JointSpaceTrackingData>(
      "left_hip_yaw_traj", hip_yaw_kp * MatrixXd::Ones(1, 1),
      hip_yaw_kd * MatrixXd::Ones(1, 1), w_hip_yaw * MatrixXd::Ones(1, 1),
      plant_w_springs, plant_wo_springs);
  auto right_hip_yaw_traj = std::make_unique<JointSpaceTrackingData>(
      "right_hip_yaw_traj", hip_yaw_kp * MatrixXd::Ones(1, 1),
      hip_yaw_kd * MatrixXd::Ones(1, 1), w_hip_yaw * MatrixXd::Ones(1, 1),
      plant_w_springs, plant_wo_springs);
  left_hip_yaw_traj->AddJointToTrack("hip_yaw_left", "hip_yaw_leftdot");
  osc->AddConstTrackingData(std::move(left_hip_yaw_traj), VectorXd::Zero(1));
  right_hip_yaw_traj->AddJointToTrack("hip_yaw_right", "hip_yaw_rightdot");
  osc->AddConstTrackingData(std::move(right_hip_yaw_traj), VectorXd::Zero(1));

  // Build OSC problem
  osc->Build();
  // Connect ports
  builder.Connect(state_receiver->get_output_port(0),
                  osc->get_input_port_robot_output());
  builder.Connect(osc->get_output_port_osc_command(),
                  command_sender->get_input_port(0));
  builder.Connect(com_traj_generator->get_output_port(0),
                  osc->get_input_port_tracking_data("com_traj"));
  builder.Connect(pelvis_rot_traj_generator->get_output_port(0),
                  osc->get_input_port_tracking_data("pelvis_rot_traj"));

  // Create osc debug sender and add to diagram if desired
  auto osc_debug_pub_ptr = LcmPublisherSystem::Make<dairlib::lcmt_osc_output>(
      "OSC_DEBUG_STANDING", &lcm_local,
      TriggerTypeSet({TriggerType::kForced}));
  if (FLAGS_publish_osc) {
    auto osc_debug_pub =
        builder.AddSystem(std::move(osc_debug_pub_ptr));
    builder.Connect(osc->get_output_port_osc_debug(),
                    osc_debug_pub->get_input_port());
  }

  // Create the diagram
  auto owned_diagram = builder.Build();
  owned_diagram->set_name(("osc standing controller"));

  // Build lcm-driven simulation
  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm_local, std::move(owned_diagram), state_receiver, FLAGS_channel_x,
      true);

  loop.Simulate();

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::DoMain(argc, argv); }
