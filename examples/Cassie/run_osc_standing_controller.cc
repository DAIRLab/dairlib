#include <gflags/gflags.h>

#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_target_standing_height.hpp"
#include "examples/Cassie/cassie_utils.h"
#include "examples/Cassie/osc/osc_standing_gains.h"
#include "examples/Cassie/osc/standing_com_traj.h"
#include "examples/Cassie/osc/standing_pelvis_orientation_traj.h"
#include "multibody/kinematic/kinematic_evaluator_set.h"
#include "multibody/kinematic/fixed_joint_evaluator.h"
#include "multibody/multibody_utils.h"
#include "systems/controllers/controller_failure_aggregator.h"
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
using multibody::FixedJointEvaluator;

DEFINE_string(channel_x, "CASSIE_STATE_SIMULATION",
              "LCM channel for receiving state. "
              "Use CASSIE_STATE_SIMULATION to get state from simulator, and "
              "use CASSIE_STATE_DISPATCHER to get state from state estimator");
DEFINE_string(channel_u, "CASSIE_INPUT",
              "The name of the channel which publishes command");
DEFINE_string(
    cassie_out_channel, "CASSIE_OUTPUT_ECHO",
    "The name of the channel to receive the cassie out structure from.");
DEFINE_bool(print_osc, false, "whether to print the osc debug message or not");
DEFINE_double(cost_weight_multiplier, 1.0,
              "A cosntant times with cost weight of OSC traj tracking");
DEFINE_double(height, .8, "The initial COM height (m)");
DEFINE_string(osc_gains_filename, "examples/Cassie/osc/osc_standing_gains.yaml",
              "Filepath containing osc_gains");
DEFINE_string(osqp_settings, "solvers/default_osc_osqp_settings.yaml",
              "Filepath containing qp settings");
DEFINE_bool(use_radio, false, "use the radio to set height or not");
DEFINE_double(qp_time_limit, 0.002, "maximum qp solve time");

// Currently the controller runs at the rate between 500 Hz and 200 Hz, so the
// publish rate of the robot state needs to be less than 500 Hz. Otherwise, the
// performance seems to degrade due to this. (Recommended publish rate: 200 Hz)
// Maybe we need to update the lcm driven loop to clear the queue of lcm message
// if it's more than one message?

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Build Cassie MBP
  drake::multibody::MultibodyPlant<double> plant(0.0);
  addCassieMultibody(&plant, nullptr, true /*floating base*/,
                     "examples/Cassie/urdf/cassie_v2_conservative.urdf",
                     true /*spring model*/, false /*loop closure*/);
  plant.Finalize();
  // Build fix-spring Cassie MBP

  auto context_w_spr = plant.CreateDefaultContext();
  auto context_wo_spr = plant.CreateDefaultContext();

  // Get contact frames and position (doesn't matter whether we use
  // plant_w_springs or plant_wo_springs because the contact frames exit in both
  // plants)
  auto left_toe = LeftToeFront(plant);
  auto left_heel = LeftToeRear(plant);
  auto right_toe = RightToeFront(plant);
  auto right_heel = RightToeRear(plant);

  // Build the controller diagram
  DiagramBuilder<double> builder;

  drake::lcm::DrakeLcm lcm_local("udpm://239.255.76.67:7667?ttl=0");
//  auto osc_gains =
//      drake::yaml::LoadYamlFile<OSCStandingGains>(FLAGS_osc_gains_filename);
  drake::yaml::YamlReadArchive::Options yaml_options;
  yaml_options.allow_yaml_with_no_cpp = true;
  OSCGains gains = drake::yaml::LoadYamlFile<OSCGains>(
      FindResourceOrThrow(FLAGS_osc_gains_filename), {}, {}, yaml_options);
  OSCStandingGains osc_gains = drake::yaml::LoadYamlFile<OSCStandingGains>(
      FindResourceOrThrow(FLAGS_osc_gains_filename));

  MatrixXd K_p_com = Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      osc_gains.CoMKp.data(), osc_gains.rows, osc_gains.cols);
  MatrixXd K_d_com = Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      osc_gains.CoMKd.data(), osc_gains.rows, osc_gains.cols);
  MatrixXd K_p_pelvis = Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      osc_gains.PelvisRotKp.data(), osc_gains.rows, osc_gains.cols);
  MatrixXd K_d_pelvis = Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      osc_gains.PelvisRotKd.data(), osc_gains.rows, osc_gains.cols);
  MatrixXd K_p_hip_yaw = osc_gains.HipYawKp * MatrixXd::Identity(1, 1);
  MatrixXd K_d_hip_yaw = osc_gains.HipYawKd * MatrixXd::Identity(1, 1);
  MatrixXd W_com = Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      osc_gains.CoMW.data(), osc_gains.rows, osc_gains.cols);
  MatrixXd W_pelvis = Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      osc_gains.PelvisW.data(), osc_gains.rows, osc_gains.cols);
  MatrixXd W_hip_yaw = osc_gains.HipYawW * MatrixXd::Identity(1, 1);

  // Create Lcm subsriber for lcmt_target_standing_height
  auto target_height_receiver = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_target_standing_height>(
          "TARGET_HEIGHT", &lcm_local));

  // Create state receiver.
  auto state_receiver =
      builder.AddSystem<systems::RobotOutputReceiver>(plant);

  auto cassie_out_receiver =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_cassie_out>(
          FLAGS_cassie_out_channel, &lcm_local));

  // Create command sender.
  auto command_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          FLAGS_channel_u, &lcm_local, TriggerTypeSet({TriggerType::kForced})));
  auto command_sender =
      builder.AddSystem<systems::RobotCommandSender>(plant);

  builder.Connect(command_sender->get_output_port(0),
                  command_pub->get_input_port());

  // Create osc debug sender.
  auto osc_debug_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_osc_output>(
          "OSC_DEBUG_STANDING", &lcm_local,
          TriggerTypeSet({TriggerType::kForced})));
  auto failure_aggregator =
      builder.AddSystem<systems::ControllerFailureAggregator>(FLAGS_channel_u,
                                                              1);
  auto controller_failure_pub = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_controller_failure>(
          "CONTROLLER_ERROR", &lcm_local, TriggerTypeSet({TriggerType::kForced})));


  // Create desired center of mass traj
  std::vector<std::pair<const Vector3d, const drake::multibody::Frame<double>&>>
      feet_contact_points = {left_toe, left_heel, right_toe, right_heel};
  auto com_traj_generator = builder.AddSystem<cassie::osc::StandingComTraj>(
      plant, context_w_spr.get(), feet_contact_points, FLAGS_height,
      FLAGS_use_radio);
  auto pelvis_rot_traj_generator =
      builder.AddSystem<cassie::osc::StandingPelvisOrientationTraj>(
          plant, context_w_spr.get(), feet_contact_points,
          "pelvis_rot_traj");
  builder.Connect(state_receiver->get_output_port(0),
                  com_traj_generator->get_input_port_state());
  builder.Connect(state_receiver->get_output_port(0),
                  pelvis_rot_traj_generator->get_input_port_state());
  builder.Connect(cassie_out_receiver->get_output_port(),
                  pelvis_rot_traj_generator->get_input_port_radio());
  builder.Connect(cassie_out_receiver->get_output_port(),
                  com_traj_generator->get_input_port_radio());
  builder.Connect(target_height_receiver->get_output_port(),
                  com_traj_generator->get_input_port_target_height());

  // Create Operational space control
  auto osc = builder.AddSystem<systems::controllers::OperationalSpaceControl>(
      plant, plant, context_w_spr.get(),
      context_wo_spr.get(), false, FLAGS_print_osc, FLAGS_qp_time_limit);

  // Distance constraint
  multibody::KinematicEvaluatorSet<double> evaluators(plant);
  auto left_loop = LeftLoopClosureEvaluator(plant);
  auto right_loop = RightLoopClosureEvaluator(plant);
  evaluators.add_evaluator(&left_loop);
  evaluators.add_evaluator(&right_loop);

  // Friction coefficient
  osc->SetContactFriction(gains.mu);
  // Add contact points
  auto left_toe_evaluator = multibody::WorldPointEvaluator(
      plant, left_toe.first, left_toe.second, Matrix3d::Identity(),
      Vector3d::Zero(), {1, 2});
  auto left_heel_evaluator = multibody::WorldPointEvaluator(
      plant, left_heel.first, left_heel.second, Matrix3d::Identity(),
      Vector3d::Zero(), {0, 1, 2});
  auto right_toe_evaluator = multibody::WorldPointEvaluator(
      plant, right_toe.first, right_toe.second, Matrix3d::Identity(),
      Vector3d::Zero(), {1, 2});
  auto right_heel_evaluator = multibody::WorldPointEvaluator(
      plant, right_heel.first, right_heel.second,
      Matrix3d::Identity(), Vector3d::Zero(), {0, 1, 2});
  osc->AddContactPoint(&left_toe_evaluator);
  osc->AddContactPoint(&left_heel_evaluator);
  osc->AddContactPoint(&right_toe_evaluator);
  osc->AddContactPoint(&right_heel_evaluator);

  auto pos_idx_map = multibody::makeNameToPositionsMap(plant);
  auto vel_idx_map = multibody::makeNameToVelocitiesMap(plant);
  auto left_fixed_knee_spring =
      FixedJointEvaluator(plant, pos_idx_map.at("knee_joint_left"),
                          vel_idx_map.at("knee_joint_leftdot"), 0);
  auto right_fixed_knee_spring =
      FixedJointEvaluator(plant, pos_idx_map.at("knee_joint_right"),
                          vel_idx_map.at("knee_joint_rightdot"), 0);
  auto left_fixed_ankle_spring =
      FixedJointEvaluator(plant, pos_idx_map.at("ankle_spring_joint_left"),
                          vel_idx_map.at("ankle_spring_joint_leftdot"), 0);
  auto right_fixed_ankle_spring =
      FixedJointEvaluator(plant, pos_idx_map.at("ankle_spring_joint_right"),
                          vel_idx_map.at("ankle_spring_joint_rightdot"), 0);
  evaluators.add_evaluator(&left_fixed_knee_spring);
  evaluators.add_evaluator(&right_fixed_knee_spring);
  evaluators.add_evaluator(&left_fixed_ankle_spring);
  evaluators.add_evaluator(&right_fixed_ankle_spring);
  osc->AddKinematicConstraint(&evaluators);

  // Cost
  int n_v = plant.num_velocities();

  osc->SetAccelerationCostWeights(gains.w_accel * gains.W_acceleration);
  osc->SetInputSmoothingWeights(1e-3 * gains.W_input_regularization);
  osc->SetInputCostWeights(gains.w_input * gains.W_input_regularization);
  osc->SetLambdaHolonomicRegularizationWeight(1e-5 *
                                              gains.W_lambda_h_regularization);

  // Center of mass tracking
  // Weighting x-y higher than z, as they are more important to balancing
  //  ComTrackingData center_of_mass_traj("com_traj", K_p_com, K_d_com,
  //                                      W_com * FLAGS_cost_weight_multiplier,
  //                                      plant_w_springs, plant_wo_springs);
  auto center_of_mass_traj = std::make_unique<TransTaskSpaceTrackingData>(
      "com_traj", K_p_com, K_d_com, W_com * FLAGS_cost_weight_multiplier, plant,
      plant);
  center_of_mass_traj->AddPointToTrack("pelvis");
  //  double cutoff_freq = 5; // in Hz
  //  double tau = 1 / (2 * M_PI * cutoff_freq);
  center_of_mass_traj->SetLowPassFilter(0.03, {1});
  osc->AddTrackingData(std::move(center_of_mass_traj));
  // Pelvis rotation tracking
  auto pelvis_rot_traj = std::make_unique<RotTaskSpaceTrackingData>(
      "pelvis_rot_traj", K_p_pelvis, K_d_pelvis,
      W_pelvis * FLAGS_cost_weight_multiplier, plant, plant);
  pelvis_rot_traj->AddFrameToTrack("pelvis");
  osc->AddTrackingData(std::move(pelvis_rot_traj));

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
      plant, plant);
  auto right_hip_yaw_traj = std::make_unique<JointSpaceTrackingData>(
      "right_hip_yaw_traj", hip_yaw_kp * MatrixXd::Ones(1, 1),
      hip_yaw_kd * MatrixXd::Ones(1, 1), w_hip_yaw * MatrixXd::Ones(1, 1),
      plant, plant);
  left_hip_yaw_traj->AddJointToTrack("hip_yaw_left", "hip_yaw_leftdot");
  osc->AddConstTrackingData(std::move(left_hip_yaw_traj), VectorXd::Zero(1));
  right_hip_yaw_traj->AddJointToTrack("hip_yaw_right", "hip_yaw_rightdot");
  osc->AddConstTrackingData(std::move(right_hip_yaw_traj), VectorXd::Zero(1));

  // Build OSC problem
  osc->Build();
  // Connect ports
  builder.Connect(state_receiver->get_output_port(0),
                  osc->get_robot_output_input_port());
  builder.Connect(osc->get_osc_output_port(),
                  command_sender->get_input_port(0));
  builder.Connect(osc->get_osc_debug_port(), osc_debug_pub->get_input_port());
  builder.Connect(com_traj_generator->get_output_port(0),
                  osc->get_tracking_data_input_port("com_traj"));
  builder.Connect(pelvis_rot_traj_generator->get_output_port(0),
                  osc->get_tracking_data_input_port("pelvis_rot_traj"));
  builder.Connect(osc->get_failure_output_port(),
                  failure_aggregator->get_input_port(0));
  builder.Connect(failure_aggregator->get_status_output_port(),
                  controller_failure_pub->get_input_port());

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
