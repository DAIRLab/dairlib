#include <drake/common/yaml/yaml_io.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/lcm/lcm_publisher_system.h>
#include <gflags/gflags.h>
#include <dairlib/lcmt_timestamped_saved_traj.hpp>

#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "examples/Cassie/cassie_utils.h"
#include "examples/Cassie/kinematic_centroidal_planner/contact_scheduler.h"
#include "examples/Cassie/kinematic_centroidal_planner/kinematic_trajectory_generator.h"
#include "examples/Cassie/osc_jump/osc_jumping_gains.h"
#include "lcm/lcm_trajectory.h"
#include "multibody/kinematic/fixed_joint_evaluator.h"
#include "systems/controllers/controller_failure_aggregator.h"
#include "systems/controllers/osc/joint_space_tracking_data.h"
#include "systems/controllers/osc/operational_space_control.h"
#include "systems/controllers/osc/relative_translation_tracking_data.h"
#include "systems/controllers/osc/rot_space_tracking_data.h"
#include "systems/controllers/osc/trans_space_tracking_data.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/primitives/trajectory_passthrough.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"
#include "systems/trajectory_optimization/lcm_trajectory_systems.h"

namespace dairlib {

using std::map;
using std::pair;
using std::string;
using std::vector;

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using drake::geometry::SceneGraph;
using drake::multibody::Frame;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::DiagramBuilder;
using drake::systems::TriggerType;
using drake::systems::TriggerTypeSet;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::trajectories::PiecewisePolynomial;
using multibody::FixedJointEvaluator;
using systems::LcmTrajectoryReceiver;
using systems::TrajectoryPassthrough;
using systems::controllers::JointSpaceTrackingData;
using systems::controllers::RelativeTranslationTrackingData;
using systems::controllers::RotTaskSpaceTrackingData;
using systems::controllers::TransTaskSpaceTrackingData;

namespace examples {

DEFINE_string(
    channel_x, "CASSIE_STATE_SIMULATION",
    "The name of the channel where state estimation is published. Set to "
    "CASSIE_STATE_DISPATCHER for use on hardware with the state estimator");
DEFINE_string(channel_u, "CASSIE_INPUT",
              "The name of the channel where control efforts are published");
DEFINE_string(channel_reference, "KCMPC_OUTPUT",
              "The name of the channel where the reference trajectories from "
              "MPC are published");
DEFINE_string(folder_path, "examples/Cassie/saved_trajectories/",
              "Folder path for where the trajectory names are stored");
DEFINE_string(traj_name, "kcmpc_solution",
              "File to load saved trajectories from");
DEFINE_string(
    gains_filename,
    "examples/Cassie/kinematic_centroidal_planner/osc_centroidal_gains.yaml",
    "Filepath containing gains");
DEFINE_string(
    osqp_settings,
    "examples/Cassie/kinematic_centroidal_planner/osc_tracking_qp_settings.yaml",
    "Filepath containing qp settings");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Build the controller diagram
  DiagramBuilder<double> builder;

  // Built the Cassie MBPs
  drake::multibody::MultibodyPlant<double> plant_w_spr(0.0);
  AddCassieMultibody(&plant_w_spr, nullptr, true,
                     "examples/Cassie/urdf/cassie_v2_conservative.urdf",
                     false /*spring model*/, false /*loop closure*/);
  plant_w_spr.Finalize();

  auto context_w_spr = plant_w_spr.CreateDefaultContext();

  drake::multibody::MultibodyPlant<double> plant_wo_spr(0.0);
  AddCassieMultibody(&plant_wo_spr, nullptr, true,
                     "examples/Cassie/urdf/cassie_fixed_springs.urdf", false,
                     false);
  plant_wo_spr.Finalize();
  auto context_wo_spr = plant_wo_spr.CreateDefaultContext();

  // Get contact frames and position
  auto left_toe = LeftToeFront(plant_w_spr);
  auto left_heel = LeftToeRear(plant_w_spr);
  auto right_toe = RightToeFront(plant_w_spr);
  auto right_heel = RightToeRear(plant_w_spr);

  int n_v = plant_w_spr.num_velocities();
  int n_u = plant_w_spr.num_actuators();

  map<string, int> pos_map_wo_spr =
      multibody::MakeNameToPositionsMap(plant_wo_spr);

  // Create maps for joints
  map<string, int> pos_map = multibody::MakeNameToPositionsMap(plant_w_spr);
  map<string, int> vel_map = multibody::MakeNameToVelocitiesMap(plant_w_spr);
  map<string, int> act_map = multibody::MakeNameToActuatorsMap(plant_w_spr);

  std::vector<std::pair<const Vector3d, const drake::multibody::Frame<double>&>>
      feet_contact_points = {left_toe, right_toe};

  /**** Convert the gains from the yaml struct to Eigen Matrices ****/
  auto gains = drake::yaml::LoadYamlFile<OSCJumpingGains>(FLAGS_gains_filename);

  string output_traj_path = FLAGS_folder_path + FLAGS_traj_name;

  /**** Initialize all the leaf systems ****/
  drake::lcm::DrakeLcm lcm("udpm://239.255.76.67:7667?ttl=0");

  std::unordered_map<int, int> contact_state_to_fsm_map;
  const int FLIGHT = 0;
  const int LEFT_STANCE = 1;
  const int RIGHT_STANCE = 2;
  const int DOUBLE_STANCE = 3;
  contact_state_to_fsm_map[0] = 0;
  contact_state_to_fsm_map[12] = 2;  // left stance
  contact_state_to_fsm_map[3] = 1;   // right stance
  contact_state_to_fsm_map[15] = 3;  // double stance

  auto trajectory_subscriber =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          FLAGS_channel_reference, &lcm));
  auto state_reference_receiver =
      builder.AddSystem<LcmTrajectoryReceiver>("state_traj");
  // (TODO):yangwill add functionality to OSC to track force trajectory
  auto contact_force_reference_receiver =
      builder.AddSystem<LcmTrajectoryReceiver>("contact_force_traj");

  auto state_receiver =
      builder.AddSystem<systems::RobotOutputReceiver>(plant_w_spr);
  // This actually outputs the target position of the pelvis not the true
  // center of mass

  // (TODO):yangwill consider or at least verify why we want to track the pelvis
  // over the CoM
  auto pelvis_trans_traj_generator =
      builder.AddSystem<KinematicTrajectoryGenerator>(
          plant_wo_spr, context_wo_spr.get(), "pelvis", VectorXd::Zero(3));
  KinematicTrajectoryGenerator* l_foot_traj_generator;
  KinematicTrajectoryGenerator* r_foot_traj_generator;
  if (gains.relative_feet) {
    l_foot_traj_generator = builder.AddSystem<KinematicTrajectoryGenerator>(
        plant_wo_spr, context_wo_spr.get(), "pelvis", VectorXd::Zero(3),
        "toe_left", VectorXd::Zero(3));
    r_foot_traj_generator = builder.AddSystem<KinematicTrajectoryGenerator>(
        plant_wo_spr, context_wo_spr.get(), "pelvis", VectorXd::Zero(3),
        "toe_right", VectorXd::Zero(3));
  } else {
    l_foot_traj_generator = builder.AddSystem<KinematicTrajectoryGenerator>(
        plant_wo_spr, context_wo_spr.get(), "toe_left", VectorXd::Zero(3));
    r_foot_traj_generator = builder.AddSystem<KinematicTrajectoryGenerator>(
        plant_wo_spr, context_wo_spr.get(), "toe_right", VectorXd::Zero(3));
  }
  auto pelvis_rot_traj_generator =
      builder.AddSystem<TrajectoryPassthrough>("pelvis_rot_traj", 0, 4);
  auto contact_scheduler = builder.AddSystem<systems::ContactScheduler>(
      plant_w_spr, contact_state_to_fsm_map);
  auto command_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          FLAGS_channel_u, &lcm, TriggerTypeSet({TriggerType::kForced})));
  auto command_sender =
      builder.AddSystem<systems::RobotCommandSender>(plant_w_spr);
  auto osc = builder.AddSystem<systems::controllers::OperationalSpaceControl>(
      plant_w_spr, plant_w_spr, context_w_spr.get(), context_w_spr.get(), true);
  auto osc_debug_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_osc_output>(
          "OSC_DEBUG_CENTROIDAL", &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto failure_aggregator =
      builder.AddSystem<systems::ControllerFailureAggregator>(FLAGS_channel_u,
                                                              1);
  auto controller_failure_pub = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_controller_failure>(
          "CONTROLLER_ERROR", &lcm, TriggerTypeSet({TriggerType::kForced})));

  /**** OSC setup ****/
  // Cost
  MatrixXd Q_accel = gains.w_accel * MatrixXd::Identity(n_v, n_v);
  osc->SetAccelerationCostWeights(Q_accel);
  // Soft constraint on contacts
  double w_contact_relax = gains.w_soft_constraint;
  osc->SetContactSoftConstraintWeight(w_contact_relax);
  // Soft constraint on contacts
  double w_input_reg = gains.w_input_reg;
  osc->SetInputSmoothingWeights(gains.w_input_reg *
                                MatrixXd::Identity(n_u, n_u));

  // Contact information for OSC
  osc->SetContactFriction(gains.mu);

  auto left_toe_evaluator = multibody::WorldPointEvaluator(
      plant_w_spr, left_toe.first, left_toe.second, Matrix3d::Identity(),
      Vector3d::Zero(), {1, 2});
  auto left_heel_evaluator = multibody::WorldPointEvaluator(
      plant_w_spr, left_heel.first, left_heel.second, Matrix3d::Identity(),
      Vector3d::Zero(), {0, 1, 2});
  auto right_toe_evaluator = multibody::WorldPointEvaluator(
      plant_w_spr, right_toe.first, right_toe.second, Matrix3d::Identity(),
      Vector3d::Zero(), {1, 2});
  auto right_heel_evaluator = multibody::WorldPointEvaluator(
      plant_w_spr, right_heel.first, right_heel.second, Matrix3d::Identity(),
      Vector3d::Zero(), {0, 1, 2});

  osc->AddStateAndContactPoint(LEFT_STANCE, &left_toe_evaluator);
  osc->AddStateAndContactPoint(LEFT_STANCE, &left_heel_evaluator);
  osc->AddStateAndContactPoint(RIGHT_STANCE, &right_toe_evaluator);
  osc->AddStateAndContactPoint(RIGHT_STANCE, &right_heel_evaluator);
  osc->AddStateAndContactPoint(DOUBLE_STANCE, &left_toe_evaluator);
  osc->AddStateAndContactPoint(DOUBLE_STANCE, &left_heel_evaluator);
  osc->AddStateAndContactPoint(DOUBLE_STANCE, &right_toe_evaluator);
  osc->AddStateAndContactPoint(DOUBLE_STANCE, &right_heel_evaluator);

  multibody::KinematicEvaluatorSet<double> evaluators(plant_w_spr);
  auto left_loop = LeftLoopClosureEvaluator(plant_w_spr);
  auto right_loop = RightLoopClosureEvaluator(plant_w_spr);
  evaluators.add_evaluator(&left_loop);
  evaluators.add_evaluator(&right_loop);

  // Fix the springs in the dynamics
  auto pos_idx_map = multibody::MakeNameToPositionsMap(plant_w_spr);
  auto vel_idx_map = multibody::MakeNameToVelocitiesMap(plant_w_spr);
  auto left_fixed_knee_spring =
      FixedJointEvaluator(plant_w_spr, pos_idx_map.at("knee_joint_left"),
                          vel_idx_map.at("knee_joint_leftdot"), 0);
  auto right_fixed_knee_spring =
      FixedJointEvaluator(plant_w_spr, pos_idx_map.at("knee_joint_right"),
                          vel_idx_map.at("knee_joint_rightdot"), 0);
  auto left_fixed_ankle_spring = FixedJointEvaluator(
      plant_w_spr, pos_idx_map.at("ankle_spring_joint_left"),
      vel_idx_map.at("ankle_spring_joint_leftdot"), 0);
  auto right_fixed_ankle_spring = FixedJointEvaluator(
      plant_w_spr, pos_idx_map.at("ankle_spring_joint_right"),
      vel_idx_map.at("ankle_spring_joint_rightdot"), 0);
  evaluators.add_evaluator(&left_fixed_knee_spring);
  evaluators.add_evaluator(&right_fixed_knee_spring);
  evaluators.add_evaluator(&left_fixed_ankle_spring);
  evaluators.add_evaluator(&right_fixed_ankle_spring);

  osc->AddKinematicConstraint(&evaluators);

  /**** Tracking Data for OSC *****/
  TransTaskSpaceTrackingData pelvis_tracking_data(
      "pelvis_trans_traj", gains.K_p_com, gains.K_d_com, gains.W_com,
      plant_w_spr, plant_w_spr);
  pelvis_tracking_data.AddStateAndPointToTrack(DOUBLE_STANCE, "pelvis");
  pelvis_tracking_data.AddStateAndPointToTrack(LEFT_STANCE, "pelvis");
  pelvis_tracking_data.AddStateAndPointToTrack(RIGHT_STANCE, "pelvis");

  RotTaskSpaceTrackingData pelvis_rot_tracking_data(
      "pelvis_rot_tracking_data", gains.K_p_pelvis, gains.K_d_pelvis,
      gains.W_pelvis, plant_w_spr, plant_w_spr);
  pelvis_rot_tracking_data.AddStateAndFrameToTrack(DOUBLE_STANCE, "pelvis");
  pelvis_rot_tracking_data.AddStateAndFrameToTrack(LEFT_STANCE, "pelvis");
  pelvis_rot_tracking_data.AddStateAndFrameToTrack(RIGHT_STANCE, "pelvis");

  TransTaskSpaceTrackingData left_foot_tracking_data(
      "left_ft_traj", gains.K_p_flight_foot, gains.K_d_flight_foot,
      gains.W_flight_foot, plant_w_spr, plant_w_spr);
  TransTaskSpaceTrackingData right_foot_tracking_data(
      "right_ft_traj", gains.K_p_flight_foot, gains.K_d_flight_foot,
      gains.W_flight_foot, plant_w_spr, plant_w_spr);
  left_foot_tracking_data.AddStateAndPointToTrack(FLIGHT, "toe_left");
  right_foot_tracking_data.AddStateAndPointToTrack(FLIGHT, "toe_right");
  left_foot_tracking_data.AddStateAndPointToTrack(RIGHT_STANCE, "toe_left");
  right_foot_tracking_data.AddStateAndPointToTrack(LEFT_STANCE, "toe_right");

  TransTaskSpaceTrackingData left_hip_tracking_data(
      "left_hip_traj", gains.K_p_flight_foot, gains.K_d_flight_foot,
      gains.W_flight_foot, plant_w_spr, plant_w_spr);
  TransTaskSpaceTrackingData right_hip_tracking_data(
      "right_hip_traj", gains.K_p_flight_foot, gains.K_d_flight_foot,
      gains.W_flight_foot, plant_w_spr, plant_w_spr);
  left_hip_tracking_data.AddStateAndPointToTrack(FLIGHT, "pelvis");
  right_hip_tracking_data.AddStateAndPointToTrack(FLIGHT, "pelvis");
  left_hip_tracking_data.AddStateAndPointToTrack(RIGHT_STANCE, "pelvis");
  right_hip_tracking_data.AddStateAndPointToTrack(LEFT_STANCE, "pelvis");
  //  auto stance_foot_tracking_data =
  //  std::make_unique<TransTaskSpaceTrackingData>(
  //      "stance_ft_traj", gains.K_p_flight_foot, gains.K_d_flight_foot,
  //      gains.W_flight_foot, plant_w_spr, plant_w_spr);
  //  stance_foot_tracking_data->AddStateAndPointToTrack(
  //      LEFT_STANCE, "toe_left");
  //  stance_foot_tracking_data->AddStateAndPointToTrack(
  //      RIGHT_STANCE, "toe_right");

  RelativeTranslationTrackingData left_foot_rel_tracking_data(
      "left_ft_traj", gains.K_p_flight_foot, gains.K_d_flight_foot,
      gains.W_flight_foot, plant_w_spr, plant_w_spr, &left_foot_tracking_data,
      &left_hip_tracking_data);
  RelativeTranslationTrackingData right_foot_rel_tracking_data(
      "right_ft_traj", gains.K_p_flight_foot, gains.K_d_flight_foot,
      gains.W_flight_foot, plant_w_spr, plant_w_spr, &right_foot_tracking_data,
      &right_hip_tracking_data);
  //  auto pelvis_trans_rel_tracking_data =
  //      std::make_unique<RelativeTranslationTrackingData>(
  //          "pelvis_trans_traj", gains.K_p_pelvis, gains.K_d_pelvis,
  //          gains.W_pelvis, plant_w_spr, plant_w_spr, &pelvis_tracking_data,
  //          stance_foot_tracking_data.get());

  // Flight phase hip yaw tracking
  JointSpaceTrackingData left_hip_yaw_tracking_data(
      "hip_yaw_left_traj", gains.K_p_hip_yaw, gains.K_d_hip_yaw,
      gains.W_hip_yaw, plant_w_spr, plant_w_spr);
  JointSpaceTrackingData right_hip_yaw_tracking_data(
      "hip_yaw_right_traj", gains.K_p_hip_yaw, gains.K_d_hip_yaw,
      gains.W_hip_yaw, plant_w_spr, plant_w_spr);
  left_hip_yaw_tracking_data.AddStateAndJointToTrack(FLIGHT, "hip_yaw_left",
                                                     "hip_yaw_leftdot");
  right_hip_yaw_tracking_data.AddStateAndJointToTrack(FLIGHT, "hip_yaw_right",
                                                      "hip_yaw_rightdot");
  left_hip_yaw_tracking_data.AddStateAndJointToTrack(
      RIGHT_STANCE, "hip_yaw_left", "hip_yaw_leftdot");
  right_hip_yaw_tracking_data.AddStateAndJointToTrack(
      LEFT_STANCE, "hip_yaw_right", "hip_yaw_rightdot");

  //  osc->AddConstTrackingData(&left_hip_yaw_tracking_data, VectorXd::Zero(1));
  //  osc->AddConstTrackingData(&right_hip_yaw_tracking_data,
  //  VectorXd::Zero(1));
  osc->AddTrackingData(&left_hip_yaw_tracking_data);
  osc->AddTrackingData(&right_hip_yaw_tracking_data);

  // Flight phase toe pitch tracking
  MatrixXd W_swing_toe = gains.w_swing_toe * MatrixXd::Identity(1, 1);
  MatrixXd K_p_swing_toe = gains.swing_toe_kp * MatrixXd::Identity(1, 1);
  MatrixXd K_d_swing_toe = gains.swing_toe_kd * MatrixXd::Identity(1, 1);
  JointSpaceTrackingData left_toe_angle_tracking_data(
      "left_toe_angle_traj", K_p_swing_toe, K_d_swing_toe, W_swing_toe,
      plant_w_spr, plant_w_spr);
  JointSpaceTrackingData right_toe_angle_tracking_data(
      "right_toe_angle_traj", K_p_swing_toe, K_d_swing_toe, W_swing_toe,
      plant_w_spr, plant_w_spr);

  vector<std::pair<const Vector3d, const Frame<double>&>> left_foot_points = {
      left_heel, left_toe};
  vector<std::pair<const Vector3d, const Frame<double>&>> right_foot_points = {
      right_heel, right_toe};

  auto left_toe_angle_traj_gen =
      builder.AddSystem<systems::TrajectoryPassthrough>(
          "left_toe_traj", pos_map_wo_spr["toe_left"], 1);
  auto right_toe_angle_traj_gen =
      builder.AddSystem<systems::TrajectoryPassthrough>(
          "right_toe_traj", pos_map_wo_spr["toe_right"], 1);
  auto hip_yaw_left_traj_gen =
      builder.AddSystem<systems::TrajectoryPassthrough>(
          "hip_yaw_left_traj", pos_map_wo_spr["hip_yaw_left"], 1);
  auto hip_yaw_right_traj_gen =
      builder.AddSystem<systems::TrajectoryPassthrough>(
          "hip_yaw_right_traj", pos_map_wo_spr["hip_yaw_right"], 1);

  left_toe_angle_tracking_data.AddStateAndJointToTrack(FLIGHT, "toe_left",
                                                       "toe_leftdot");
  right_toe_angle_tracking_data.AddStateAndJointToTrack(FLIGHT, "toe_right",
                                                        "toe_rightdot");
  left_toe_angle_tracking_data.AddStateAndJointToTrack(RIGHT_STANCE, "toe_left",
                                                       "toe_leftdot");
  right_toe_angle_tracking_data.AddStateAndJointToTrack(
      LEFT_STANCE, "toe_right", "toe_rightdot");

  osc->AddTrackingData(&pelvis_tracking_data);
  osc->AddTrackingData(&pelvis_rot_tracking_data);
  if (gains.relative_feet) {
    left_foot_rel_tracking_data.SetImpactInvariantProjection(true);
    right_foot_rel_tracking_data.SetImpactInvariantProjection(true);
    //    pelvis_trans_rel_tracking_data->SetImpactInvariantProjection(true);
    osc->AddTrackingData(&left_foot_rel_tracking_data);
    osc->AddTrackingData(&right_foot_rel_tracking_data);
    //    osc->AddTrackingData(pelvis_trans_rel_tracking_data.get());
  } else {
    left_foot_tracking_data.SetImpactInvariantProjection(true);
    right_foot_tracking_data.SetImpactInvariantProjection(true);
    osc->AddTrackingData(&left_foot_tracking_data);
    osc->AddTrackingData(&right_foot_tracking_data);
  }
  osc->AddTrackingData(&left_toe_angle_tracking_data);
  osc->AddTrackingData(&right_toe_angle_tracking_data);

  left_toe_angle_tracking_data.SetImpactInvariantProjection(true);
  right_toe_angle_tracking_data.SetImpactInvariantProjection(true);
  left_hip_yaw_tracking_data.SetImpactInvariantProjection(true);
  right_hip_yaw_tracking_data.SetImpactInvariantProjection(true);
  pelvis_rot_tracking_data.SetImpactInvariantProjection(true);
  pelvis_tracking_data.SetImpactInvariantProjection(true);

  osc->SetOsqpSolverOptionsFromYaml(FLAGS_osqp_settings);
  // Build OSC problem
  osc->Build();
  std::cout << "Built OSC" << std::endl;

  /*****Connect ports*****/

  // Subscriber connections
  builder.Connect(trajectory_subscriber->get_output_port(),
                  state_reference_receiver->get_input_port_trajectory());
  builder.Connect(
      trajectory_subscriber->get_output_port(),
      contact_force_reference_receiver->get_input_port_trajectory());

  // Receiver connections
  builder.Connect(
      contact_force_reference_receiver->get_output_port_trajectory(),
      contact_scheduler->get_input_port_force_trajectory());
  builder.Connect(state_receiver->get_output_port(),
                  contact_scheduler->get_input_port_state());
  builder.Connect(state_reference_receiver->get_output_port(),
                  right_toe_angle_traj_gen->get_trajectory_input_port());
  builder.Connect(state_reference_receiver->get_output_port(),
                  left_toe_angle_traj_gen->get_trajectory_input_port());
  builder.Connect(state_reference_receiver->get_output_port(),
                  hip_yaw_left_traj_gen->get_trajectory_input_port());
  builder.Connect(state_reference_receiver->get_output_port(),
                  hip_yaw_right_traj_gen->get_trajectory_input_port());
  builder.Connect(state_reference_receiver->get_output_port(),
                  pelvis_rot_traj_generator->get_trajectory_input_port());
  builder.Connect(
      state_reference_receiver->get_output_port(),
      pelvis_trans_traj_generator->get_state_trajectory_input_port());
  builder.Connect(state_reference_receiver->get_output_port(),
                  l_foot_traj_generator->get_state_trajectory_input_port());
  builder.Connect(state_reference_receiver->get_output_port(),
                  r_foot_traj_generator->get_state_trajectory_input_port());

  // OSC connections
  builder.Connect(contact_scheduler->get_output_port_fsm(),
                  osc->get_fsm_input_port());
  builder.Connect(state_receiver->get_output_port(0),
                  osc->get_robot_output_input_port());
  builder.Connect(pelvis_trans_traj_generator->get_output_port(0),
                  osc->get_tracking_data_input_port("pelvis_trans_traj"));
  builder.Connect(l_foot_traj_generator->get_output_port(0),
                  osc->get_tracking_data_input_port("left_ft_traj"));
  builder.Connect(r_foot_traj_generator->get_output_port(0),
                  osc->get_tracking_data_input_port("right_ft_traj"));
  builder.Connect(left_toe_angle_traj_gen->get_output_port(0),
                  osc->get_tracking_data_input_port("left_toe_angle_traj"));
  builder.Connect(right_toe_angle_traj_gen->get_output_port(0),
                  osc->get_tracking_data_input_port("right_toe_angle_traj"));
  builder.Connect(hip_yaw_left_traj_gen->get_output_port(0),
                  osc->get_tracking_data_input_port("hip_yaw_left_traj"));
  builder.Connect(hip_yaw_right_traj_gen->get_output_port(0),
                  osc->get_tracking_data_input_port("hip_yaw_right_traj"));
  builder.Connect(
      pelvis_rot_traj_generator->get_output_port(0),
      osc->get_tracking_data_input_port("pelvis_rot_tracking_data"));

  // Publisher connections
  builder.Connect(osc->get_osc_output_port(),
                  command_sender->get_input_port(0));
  builder.Connect(command_sender->get_output_port(0),
                  command_pub->get_input_port());
  builder.Connect(osc->get_osc_debug_port(), osc_debug_pub->get_input_port());
  builder.Connect(osc->get_failure_output_port(),
                  failure_aggregator->get_input_port(0));
  builder.Connect(failure_aggregator->get_status_output_port(),
                  controller_failure_pub->get_input_port());

  // Run lcm-driven simulation
  // Create the diagram
  auto owned_diagram = builder.Build();
  owned_diagram->set_name(("centroidal_mpc_tracking_controller"));

  // Run lcm-driven simulation
  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm, std::move(owned_diagram), state_receiver, FLAGS_channel_x, true);
  DrawAndSaveDiagramGraph(*loop.get_diagram());

  loop.Simulate();

  return 0;
}
}  // namespace examples
}  // namespace dairlib

int main(int argc, char* argv[]) {
  return dairlib::examples::DoMain(argc, argv);
}
