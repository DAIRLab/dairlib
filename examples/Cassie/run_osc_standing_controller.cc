#include <gflags/gflags.h>

#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_target_standing_height.hpp"
#include "examples/Cassie/cassie_utils.h"
#include "examples/Cassie/osc/osc_standing_gains.h"
#include "examples/Cassie/osc/standing_com_traj.h"
#include "examples/Cassie/osc/standing_pelvis_orientation_traj.h"
#include "examples/Cassie/systems/cassie_out_to_radio.h"
#include "multibody/kinematic/fixed_joint_evaluator.h"
#include "multibody/kinematic/kinematic_evaluator_set.h"
#include "multibody/multibody_utils.h"
#include "systems/controllers/osc/com_tracking_data.h"
#include "systems/controllers/osc/joint_space_tracking_data.h"
#include "systems/controllers/osc/operational_space_control.h"
#include "systems/controllers/osc/options_tracking_data.h"
#include "systems/controllers/osc/osc_gains.h"
#include "systems/controllers/osc/relative_translation_tracking_data.h"
#include "systems/controllers/osc/rot_space_tracking_data.h"
#include "systems/controllers/osc/trans_space_tracking_data.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"

#include "drake/common/yaml/yaml_io.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"

namespace dairlib {

using std::cout;
using std::endl;
using std::unique_ptr;

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
using multibody::FixedJointEvaluator;
using multibody::WorldYawViewFrame;

using systems::controllers::ComTrackingData;
using systems::controllers::JointSpaceTrackingData;
using systems::controllers::RelativeTranslationTrackingData;
using systems::controllers::RotTaskSpaceTrackingData;
using systems::controllers::TransTaskSpaceTrackingData;

DEFINE_string(channel_x, "CASSIE_STATE_DISPATCHER",
              "LCM channel for receiving state. "
              "Use CASSIE_STATE_SIMULATION to get state from simulator, and "
              "use CASSIE_STATE_DISPATCHER to get state from state estimator");
DEFINE_string(channel_u, "CASSIE_INPUT",
              "The name of the channel which publishes command");
DEFINE_string(
    cassie_out_channel, "CASSIE_OUTPUT_ECHO",
    "The name of the channel to receive the cassie out structure from.");
DEFINE_double(cost_weight_multiplier, 1.0,
              "A cosntant times with cost weight of OSC traj tracking");
DEFINE_double(height, .8, "The initial COM height (m)");
DEFINE_string(gains_filename, "examples/Cassie/osc/osc_standing_gains.yaml",
              "Filepath containing osc_gains");
DEFINE_string(osqp_settings, "solvers/default_osc_osqp_settings.yaml",
              "Filepath containing qp settings");
DEFINE_bool(use_radio, false, "use the radio to set height or not");

// Currently the controller runs at the rate between 500 Hz and 200 Hz, so the
// publish rate of the robot state needs to be less than 500 Hz. Otherwise, the
// performance seems to degrade due to this. (Recommended publish rate: 200 Hz)
// Maybe we need to update the lcm driven loop to clear the queue of lcm message
// if it's more than one message?

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Build Cassie MBP
  drake::multibody::MultibodyPlant<double> plant(0.0);
  AddCassieMultibody(&plant, nullptr, true /*floating base*/,
                     "examples/Cassie/urdf/cassie_v2_conservative.urdf",
                     true /*spring model*/, false /*loop closure*/);
  plant_w_springs.Finalize();

  auto context_w_spr = plant_w_springs.CreateDefaultContext();

  // Get contact frames and position (doesn't matter whether we use
  // plant_w_springs or plant_w_springs because the contact frames exit in both
  // plants)
  auto left_toe = LeftToeFront(plant_w_springs);
  auto left_heel = LeftToeRear(plant_w_springs);
  auto right_toe = RightToeFront(plant_w_springs);
  auto right_heel = RightToeRear(plant_w_springs);

  // Build the controller diagram
  DiagramBuilder<double> builder;

  drake::lcm::DrakeLcm lcm_local("udpm://239.255.76.67:7667?ttl=0");
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
  auto state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(plant);

  auto cassie_out_receiver =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_cassie_out>(
          FLAGS_cassie_out_channel, &lcm_local));
  auto cassie_out_to_radio = builder.AddSystem<systems::CassieOutToRadio>();
  builder.Connect(*cassie_out_receiver, *cassie_out_to_radio);

  // Create command sender.
  auto command_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          FLAGS_channel_u, &lcm_local, TriggerTypeSet({TriggerType::kForced})));
  auto command_sender = builder.AddSystem<systems::RobotCommandSender>(plant);

  builder.Connect(command_sender->get_output_port(0),
                  command_pub->get_input_port());

  // Create osc debug sender.
  auto osc_debug_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_osc_output>(
          "OSC_DEBUG_STANDING", &lcm_local,
          TriggerTypeSet({TriggerType::kForced})));

  // Create desired center of mass traj
  std::vector<std::pair<const Vector3d, const drake::multibody::Frame<double>&>>
      feet_contact_points = {left_toe, left_heel, right_toe, right_heel};
  auto com_traj_generator = builder.AddSystem<cassie::osc::StandingComTraj>(
      plant, context_w_spr.get(), feet_contact_points, FLAGS_height,
      FLAGS_use_radio);
  com_traj_generator->SetCommandFilter(
      osc_gains.center_of_mass_command_filter_alpha);
  auto pelvis_rot_traj_generator =
      builder.AddSystem<cassie::osc::StandingPelvisOrientationTraj>(
          plant, context_w_spr.get(), feet_contact_points, "pelvis_rot_traj");
  pelvis_rot_traj_generator->SetCommandFilter(
      osc_gains.orientation_command_filter_alpha);
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
      plant_w_springs, context_w_spr.get(), false);

  // Distance constraint
  multibody::KinematicEvaluatorSet<double> evaluators(plant_w_springs);
  auto left_loop = LeftLoopClosureEvaluator(plant_w_springs);
  auto right_loop = RightLoopClosureEvaluator(plant_w_springs);
  evaluators.add_evaluator(&left_loop);
  evaluators.add_evaluator(&right_loop);

  auto pos_idx_map = multibody::MakeNameToPositionsMap(plant_w_springs);
  auto vel_idx_map = multibody::MakeNameToVelocitiesMap(plant_w_springs);
  auto left_fixed_knee_spring = multibody::FixedJointEvaluator(
      plant_w_springs, pos_idx_map.at("knee_joint_left"),
      vel_idx_map.at("knee_joint_leftdot"), 0);
  auto right_fixed_knee_spring = multibody::FixedJointEvaluator(
      plant_w_springs, pos_idx_map.at("knee_joint_right"),
      vel_idx_map.at("knee_joint_rightdot"), 0);
  auto left_fixed_ankle_spring = multibody::FixedJointEvaluator(
      plant_w_springs, pos_idx_map.at("ankle_spring_joint_left"),
      vel_idx_map.at("ankle_spring_joint_leftdot"), 0);
  auto right_fixed_ankle_spring = multibody::FixedJointEvaluator(
      plant_w_springs, pos_idx_map.at("ankle_spring_joint_right"),
      vel_idx_map.at("ankle_spring_joint_rightdot"), 0);
  evaluators.add_evaluator(&left_fixed_knee_spring);
  evaluators.add_evaluator(&right_fixed_knee_spring);
  evaluators.add_evaluator(&left_fixed_ankle_spring);
  evaluators.add_evaluator(&right_fixed_ankle_spring);

  osc->AddKinematicConstraint(
      std::unique_ptr<multibody::KinematicEvaluatorSet<double>>(&evaluators));
  // Soft constraint
  // We don't want w_contact_relax to be too big, cause we want tracking
  // error to be important
  double w_contact_relax = gains.w_soft_constraint;
  osc->SetContactSoftConstraintWeight(w_contact_relax);
  // Friction coefficient
  osc->SetContactFriction(gains.mu);
  // Add contact points
  auto left_toe_evaluator = multibody::WorldPointEvaluator(
      plant_w_springs, left_toe.first, left_toe.second, Matrix3d::Identity(),
      Vector3d::Zero(), {1, 2});
  auto left_heel_evaluator = multibody::WorldPointEvaluator(
      plant_w_springs, left_heel.first, left_heel.second, Matrix3d::Identity(),
      Vector3d::Zero(), {0, 1, 2});
  auto right_toe_evaluator = multibody::WorldPointEvaluator(
      plant_w_springs, right_toe.first, right_toe.second, Matrix3d::Identity(),
      Vector3d::Zero(), {1, 2});
  auto right_heel_evaluator = multibody::WorldPointEvaluator(
      plant_w_springs, right_heel.first, right_heel.second,
      Matrix3d::Identity(), Vector3d::Zero(), {0, 1, 2});

  osc->AddContactPoint(
      "left_toe",
      unique_ptr<multibody::WorldPointEvaluator<double>>(&left_toe_evaluator));
  osc->AddContactPoint(
      "left_heel",
      unique_ptr<multibody::WorldPointEvaluator<double>>(&left_heel_evaluator));
  osc->AddContactPoint(
      "right_toe",
      unique_ptr<multibody::WorldPointEvaluator<double>>(&right_toe_evaluator));
  osc->AddContactPoint("right_heel",
                       unique_ptr<multibody::WorldPointEvaluator<double>>(
                           &right_heel_evaluator));
  // Cost
  int n_v = plant_w_springs.num_velocities();
  MatrixXd Q_accel = gains.w_accel * MatrixXd::Identity(n_v, n_v);
  osc->SetAccelerationCostWeights(Q_accel);
  // Center of mass tracking
  // Weighting x-y higher than z, as they are more important to balancing
  //  ComTrackingData center_of_mass_traj("com_traj", K_p_com, K_d_com,
  //                                      W_com * FLAGS_cost_weight_multiplier,
  //                                      plant_w_springs, plant_w_springs);
  auto center_of_mass_traj = std::make_unique<TransTaskSpaceTrackingData>(
      "com_traj", osc_gains.K_p_pelvis, osc_gains.K_d_pelvis,
      osc_gains.W_pelvis * FLAGS_cost_weight_multiplier, plant_w_springs,
      plant_w_springs);
  center_of_mass_traj->AddPointToTrack("pelvis");
  double cutoff_freq = 5;  // in Hz
  double tau = 1 / (2 * M_PI * cutoff_freq);
  center_of_mass_traj->SetLowPassFilter(tau, {1});
  osc->AddTrackingData(std::move(center_of_mass_traj));
  auto pelvis_rot_tracking_data = std::make_unique<RotTaskSpaceTrackingData>(
      "pelvis_rot_traj", osc_gains.K_p_pelvis_rot, osc_gains.K_d_pelvis_rot,
      osc_gains.W_pelvis_rot * FLAGS_cost_weight_multiplier, plant_w_springs,
      plant_w_springs);
  pelvis_rot_tracking_data->AddFrameToTrack("pelvis");
  //  if (osc_gains.rot_filter_tau > 0) {
  //    pelvis_rot_tracking_data->SetLowPassFilter(osc_gains.rot_filter_tau,
  //                                               {0, 1, 2});
  //  }
  osc->AddTrackingData(std::move(pelvis_trans_rel_tracking_data));
  osc->AddTrackingData(std::move(pelvis_rot_tracking_data));

  auto left_hip_yaw_traj = std::make_unique<JointSpaceTrackingData>(
      "left_hip_yaw_traj", hip_yaw_kp * MatrixXd::Ones(1, 1),
      hip_yaw_kd * MatrixXd::Ones(1, 1), w_hip_yaw * MatrixXd::Ones(1, 1),
      plant_w_springs, plant_w_springs);
  auto right_hip_yaw_traj = std::make_unique<JointSpaceTrackingData>(
      "right_hip_yaw_traj", hip_yaw_kp * MatrixXd::Ones(1, 1),
      hip_yaw_kd * MatrixXd::Ones(1, 1), w_hip_yaw * MatrixXd::Ones(1, 1),
      plant_w_springs, plant_w_springs);
  left_hip_yaw_traj->AddJointToTrack("hip_yaw_left", "hip_yaw_leftdot");
  osc->AddConstTrackingData(std::move(left_hip_yaw_traj), VectorXd::Zero(1));
  //  right_hip_yaw_traj->AddJointToTrack("hip_yaw_right", "hip_yaw_rightdot");
  //  osc->AddConstTrackingData(std::move(right_hip_yaw_traj),
  //  VectorXd::Zero(1));

  // Build OSC problem
  osc->Build();
  // Connect ports
  builder.Connect(state_receiver->get_output_port(0),
                  osc->get_input_port_robot_output());
  builder.Connect(osc->get_output_port_osc_command(),
                  command_sender->get_input_port(0));
  builder.Connect(osc->get_output_port_osc_debug(),
                  osc_debug_pub->get_input_port());
  builder.Connect(com_traj_generator->get_output_port(0),
                  osc->get_input_port_tracking_data("pelvis_trans_traj"));
  builder.Connect(pelvis_rot_traj_generator->get_output_port(0),
                  osc->get_input_port_tracking_data("pelvis_rot_traj"));

  // Create the diagram
  auto owned_diagram = builder.Build();
  owned_diagram->set_name(("osc_standing_controller"));

  // Build lcm-driven simulation
  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm_local, std::move(owned_diagram), state_receiver, FLAGS_channel_x,
      true);
  DrawAndSaveDiagramGraph(*loop.get_diagram());

  loop.Simulate();

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::DoMain(argc, argv); }
