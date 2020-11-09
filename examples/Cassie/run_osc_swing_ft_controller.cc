#include <gflags/gflags.h>

#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "examples/Cassie/cassie_utils.h"
#include "examples/Cassie/osc/flight_foot_traj_generator.h"
#include "multibody/kinematic/fixed_joint_evaluator.h"
#include "multibody/kinematic/kinematic_evaluator_set.h"
#include "multibody/multibody_utils.h"
//#include "systems/controllers/fsm_event_time.h"
#include "examples/Cassie/osc_jump/flight_toe_angle_traj_generator.h"
#include "systems/controllers/osc/operational_space_control.h"
#include "systems/controllers/time_based_fsm.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/robot_lcm_systems.h"

#include "drake/common/yaml/yaml_read_archive.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"

namespace dairlib {

using std::cout;
using std::endl;
using std::vector;
using std::string;
using std::map;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using drake::multibody::Frame;
using drake::systems::DiagramBuilder;
using drake::systems::TriggerType;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::lcm::TriggerTypeSet;

using systems::controllers::ComTrackingData;
using systems::controllers::JointSpaceTrackingData;
using systems::controllers::RotTaskSpaceTrackingData;
using systems::controllers::TransTaskSpaceTrackingData;

using multibody::FixedJointEvaluator;

DEFINE_double(drift_rate, 0.0, "Drift rate for floating-base state");
DEFINE_string(channel_x, "CASSIE_STATE_SIMULATION",
              "LCM channel for receiving state. "
              "Use CASSIE_STATE_SIMULATION to get state from simulator, and "
              "use CASSIE_STATE_DISPATCHER to get state from state estimator");
DEFINE_string(channel_u, "CASSIE_INPUT",
              "The name of the channel which publishes command");
DEFINE_double(period, 1.0, "Duration of each swing leg trajectory");
DEFINE_string(
    cassie_out_channel, "CASSIE_OUTPUT_ECHO",
    "The name of the channel to receive the cassie out structure from.");
DEFINE_string(gains_filename, "examples/Cassie/osc/osc_walking_gains.yaml",
              "Filepath containing gains");
DEFINE_bool(publish_osc_data, true,
            "whether to publish lcm messages for OscTrackData");
DEFINE_bool(print_osc, false, "whether to print the osc debug message or not");

// Currently the controller runs at the rate between 500 Hz and 200 Hz, so the
// publish rate of the robot state needs to be less than 500 Hz. Otherwise, the
// performance seems to degrade due to this. (Recommended publish rate: 200 Hz)
// Maybe we need to update the lcm driven loop to clear the queue of lcm message
// if it's more than one message?

struct OSCWalkingGains {
  int rows;
  int cols;
  double w_accel;
  double w_soft_constraint;
  std::vector<double> CoMW;
  std::vector<double> CoMKp;
  std::vector<double> CoMKd;
  std::vector<double> PelvisHeadingW;
  std::vector<double> PelvisHeadingKp;
  std::vector<double> PelvisHeadingKd;
  std::vector<double> PelvisBalanceW;
  std::vector<double> PelvisBalanceKp;
  std::vector<double> PelvisBalanceKd;
  std::vector<double> SwingFootW;
  std::vector<double> SwingFootKp;
  std::vector<double> SwingFootKd;
  double w_swing_toe;
  double swing_toe_kp;
  double swing_toe_kd;
  double w_hip_yaw;
  double hip_yaw_kp;
  double hip_yaw_kd;
  double center_line_offset;
  double footstep_offset;
  double mid_foot_height;
  double final_foot_height;
  double final_foot_velocity_z;
  double lipm_height;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(rows));
    a->Visit(DRAKE_NVP(cols));
    a->Visit(DRAKE_NVP(w_accel));
    a->Visit(DRAKE_NVP(w_soft_constraint));
    a->Visit(DRAKE_NVP(CoMW));
    a->Visit(DRAKE_NVP(CoMKp));
    a->Visit(DRAKE_NVP(CoMKd));
    a->Visit(DRAKE_NVP(PelvisHeadingW));
    a->Visit(DRAKE_NVP(PelvisHeadingKp));
    a->Visit(DRAKE_NVP(PelvisHeadingKd));
    a->Visit(DRAKE_NVP(PelvisBalanceW));
    a->Visit(DRAKE_NVP(PelvisBalanceKp));
    a->Visit(DRAKE_NVP(PelvisBalanceKd));
    a->Visit(DRAKE_NVP(SwingFootW));
    a->Visit(DRAKE_NVP(SwingFootKp));
    a->Visit(DRAKE_NVP(SwingFootKd));
    a->Visit(DRAKE_NVP(w_swing_toe));
    a->Visit(DRAKE_NVP(swing_toe_kp));
    a->Visit(DRAKE_NVP(swing_toe_kd));
    a->Visit(DRAKE_NVP(w_hip_yaw));
    a->Visit(DRAKE_NVP(hip_yaw_kp));
    a->Visit(DRAKE_NVP(hip_yaw_kd));
    // swing foot heuristics
    a->Visit(DRAKE_NVP(mid_foot_height));
    a->Visit(DRAKE_NVP(center_line_offset));
    a->Visit(DRAKE_NVP(footstep_offset));
    a->Visit(DRAKE_NVP(final_foot_height));
    a->Visit(DRAKE_NVP(final_foot_velocity_z));
    // lipm heursitics
    a->Visit(DRAKE_NVP(lipm_height));
  }
};

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Build Cassie MBP
  drake::multibody::MultibodyPlant<double> plant_w_spr(0.0);
  addCassieMultibody(&plant_w_spr, nullptr, false /*floating base*/,
                     "examples/Cassie/urdf/cassie_v2_experimental.urdf",
                     false /*spring model*/, false /*loop closure*/);
  plant_w_spr.Finalize();

  auto context_w_spr = plant_w_spr.CreateDefaultContext();

  // Build the controller diagram
  DiagramBuilder<double> builder;

  drake::lcm::DrakeLcm lcm_local("udpm://239.255.76.67:7667?ttl=0");
  drake::lcm::DrakeLcm lcm_network("udpm://239.255.76.67:7667?ttl=1");

  OSCWalkingGains gains;
  const YAML::Node& root =
      YAML::LoadFile(FindResourceOrThrow(FLAGS_gains_filename));
  drake::yaml::YamlReadArchive(root).Accept(&gains);

  MatrixXd W_com = Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      gains.CoMW.data(), gains.rows, gains.cols);
  MatrixXd K_p_com = Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      gains.CoMKp.data(), gains.rows, gains.cols);
  MatrixXd K_d_com = Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      gains.CoMKd.data(), gains.rows, gains.cols);
  MatrixXd W_pelvis_heading = Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      gains.PelvisHeadingW.data(), gains.rows, gains.cols);
  MatrixXd K_p_pelvis_heading = Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      gains.PelvisHeadingKp.data(), gains.rows, gains.cols);
  MatrixXd K_d_pelvis_heading = Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      gains.PelvisHeadingKd.data(), gains.rows, gains.cols);
  MatrixXd W_pelvis_balance = Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      gains.PelvisBalanceW.data(), gains.rows, gains.cols);
  MatrixXd K_p_pelvis_balance = Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      gains.PelvisBalanceKp.data(), gains.rows, gains.cols);
  MatrixXd K_d_pelvis_balance = Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      gains.PelvisBalanceKd.data(), gains.rows, gains.cols);
  MatrixXd W_swing_foot = Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      gains.SwingFootW.data(), gains.rows, gains.cols);
  MatrixXd K_p_swing_foot = Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      gains.SwingFootKp.data(), gains.rows, gains.cols);
  MatrixXd K_d_swing_foot = Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      gains.SwingFootKd.data(), gains.rows, gains.cols);
  std::cout << "w accel: \n" << gains.w_accel << std::endl;
  std::cout << "w soft constraint: \n" << gains.w_soft_constraint << std::endl;
  std::cout << "COM W: \n" << W_com << std::endl;
  std::cout << "COM Kp: \n" << K_p_com << std::endl;
  std::cout << "COM Kd: \n" << K_d_com << std::endl;
  std::cout << "Pelvis Heading W: \n" << W_pelvis_heading << std::endl;
  std::cout << "Pelvis Heading Kp: \n" << K_p_pelvis_heading << std::endl;
  std::cout << "Pelvis Heading Kd: \n" << K_d_pelvis_heading << std::endl;
  std::cout << "Pelvis Balance W: \n" << W_pelvis_balance << std::endl;
  std::cout << "Pelvis Balance Kp: \n" << K_p_pelvis_balance << std::endl;
  std::cout << "Pelvis Balance Kd: \n" << K_d_pelvis_balance << std::endl;
  std::cout << "Swing Foot W: \n" << W_swing_foot << std::endl;
  std::cout << "Swing Foot Kp: \n" << K_p_swing_foot << std::endl;
  std::cout << "Swing Foot Kd: \n" << K_d_swing_foot << std::endl;

  // Create state receiver (must use cassie with spring because dispather_out
  // uses it)
  auto state_receiver =
      builder.AddSystem<systems::RobotOutputReceiver>(plant_w_spr);

  // Create command sender.
  auto command_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          FLAGS_channel_u, &lcm_local, TriggerTypeSet({TriggerType::kForced})));
  auto command_sender =
      builder.AddSystem<systems::RobotCommandSender>(plant_w_spr);

  auto cassie_out_receiver =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_cassie_out>(
          FLAGS_cassie_out_channel, &lcm_local));

  builder.Connect(command_sender->get_output_port(0),
                  command_pub->get_input_port());

  // Create swing leg trajectory generator (capture point)
  // Since the ground is soft in the simulation, we raise the desired final
  // foot height by 1 cm. The controller is sensitive to this number, should
  // tune this every time we change the simulation parameter or when we move
  // to the hardware testing.
  // Additionally, implementing a double support phase might mitigate the
  // instability around state transition.

  int left_stance_state = 0;
  int right_stance_state = 1;

  std::vector<int> fsm_states = {left_stance_state, right_stance_state};
  std::vector<double> state_durations = {FLAGS_period, FLAGS_period};
  auto fsm = builder.AddSystem<systems::TimeBasedFiniteStateMachine>(
      plant_w_spr, fsm_states, state_durations);

  auto left_ft_traj_generator =
      builder.AddSystem<examples::osc::FlightFootTrajGenerator>(
          plant_w_spr, context_w_spr.get(), FLAGS_period, 1);
  auto right_ft_traj_generator =
      builder.AddSystem<examples::osc::FlightFootTrajGenerator>(
          plant_w_spr, context_w_spr.get(), FLAGS_period, 0);

  // Create Operational space control
  auto osc = builder.AddSystem<systems::controllers::OperationalSpaceControl>(
      plant_w_spr, plant_w_spr, context_w_spr.get(), context_w_spr.get(), true,
      FLAGS_print_osc /*print_tracking_info*/);

  // Cost
  int n_v = plant_w_spr.num_velocities();
  MatrixXd Q_accel = gains.w_accel * MatrixXd::Identity(n_v, n_v);
  osc->SetAccelerationCostForAllJoints(Q_accel);

  // Constraints in OSC
  multibody::KinematicEvaluatorSet<double> evaluators(plant_w_spr);
  // 1. fourbar constraint
  auto left_loop = LeftLoopClosureEvaluator(plant_w_spr);
  auto right_loop = RightLoopClosureEvaluator(plant_w_spr);
  evaluators.add_evaluator(&left_loop);
  evaluators.add_evaluator(&right_loop);
  // 2. fixed spring constriant
  // Note that we set the position value to 0, but this is not used in OSC,
  // because OSC constraint only use JdotV and J.
  auto pos_idx_map = multibody::makeNameToPositionsMap(plant_w_spr);
  auto vel_idx_map = multibody::makeNameToVelocitiesMap(plant_w_spr);
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

  // Swing foot tracking
  TransTaskSpaceTrackingData left_foot_traj("left_ft_traj", K_p_swing_foot,
                                            K_d_swing_foot, W_swing_foot,
                                            plant_w_spr, plant_w_spr);
  TransTaskSpaceTrackingData right_foot_traj("right_ft_traj", K_p_swing_foot,
                                             K_d_swing_foot, W_swing_foot,
                                             plant_w_spr, plant_w_spr);
  //  swing_foot_traj.AddStateAndPointToTrack(left_stance_state, "toe_right");
  //  swing_foot_traj.AddStateAndPointToTrack(right_stance_state, "toe_left");
  left_foot_traj.AddPointToTrack("toe_left");
  right_foot_traj.AddPointToTrack("toe_right");
  osc->AddTrackingData(&left_foot_traj);
  osc->AddTrackingData(&right_foot_traj);


  map<string, int> pos_map = multibody::makeNameToPositionsMap(plant_w_spr);
  map<string, int> vel_map =
      multibody::makeNameToVelocitiesMap(plant_w_spr);
  map<string, int> act_map = multibody::makeNameToActuatorsMap(plant_w_spr);
  // Swing toe joint tracking (Currently use fix position)
  // The desired position, -1.5, was derived heuristically. It is roughly the
  // toe angle when Cassie stands on the ground.
  MatrixXd W_swing_toe = gains.w_swing_toe * MatrixXd::Identity(1, 1);
  MatrixXd K_p_swing_toe = gains.swing_toe_kp * MatrixXd::Identity(1, 1);
  MatrixXd K_d_swing_toe = gains.swing_toe_kd * MatrixXd::Identity(1, 1);
  JointSpaceTrackingData swing_toe_traj_left("left_toe_angle_traj", K_p_swing_toe,
                                        K_d_swing_toe, W_swing_toe, plant_w_spr,
                                        plant_w_spr);
  JointSpaceTrackingData swing_toe_traj_right("right_toe_angle_traj", K_p_swing_toe,
                                        K_d_swing_toe, W_swing_toe, plant_w_spr,
                                        plant_w_spr);

  // Get contact frames and position (doesn't matter whether we use
  // plant_w_springs or plant_wo_springs because the contact frames exit in both
  // plants)
  auto left_toe = LeftToeFront(plant_w_spr);
  auto left_heel = LeftToeRear(plant_w_spr);
  auto right_toe = RightToeFront(plant_w_spr);
  auto right_heel = RightToeRear(plant_w_spr);
  vector<std::pair<const Vector3d, const Frame<double>&>> left_foot_points = {
      left_heel, left_toe};
  vector<std::pair<const Vector3d, const Frame<double>&>> right_foot_points = {
      right_heel, right_toe};
  auto left_toe_angle_traj_gen =
      builder.AddSystem<cassie::osc_jump::FlightToeAngleTrajGenerator>(
          plant_w_spr, context_w_spr.get(), pos_map["toe_left"],
          left_foot_points, "left_toe_angle_traj");
  auto right_toe_angle_traj_gen =
      builder.AddSystem<cassie::osc_jump::FlightToeAngleTrajGenerator>(
          plant_w_spr, context_w_spr.get(), pos_map["toe_right"],
          right_foot_points, "right_toe_angle_traj");
  swing_toe_traj_right.AddJointToTrack("toe_right", "toe_rightdot");
  swing_toe_traj_left.AddJointToTrack("toe_left", "toe_leftdot");
  osc->AddTrackingData(&swing_toe_traj_left);
  osc->AddTrackingData(&swing_toe_traj_right);
//  osc->AddConstTrackingData(&swing_toe_traj_left, -1.6 * VectorXd::Ones(1));
//  osc->AddConstTrackingData(&swing_toe_traj_right, -1.6 * VectorXd::Ones(1));
  // Swing hip yaw joint tracking
  MatrixXd W_hip_yaw = gains.w_hip_yaw * MatrixXd::Identity(1, 1);
  MatrixXd K_p_hip_yaw = gains.hip_yaw_kp * MatrixXd::Identity(1, 1);
  MatrixXd K_d_hip_yaw = gains.hip_yaw_kd * MatrixXd::Identity(1, 1);
  JointSpaceTrackingData swing_hip_yaw_left_traj("swing_hip_yaw_traj", K_p_hip_yaw,
                                            K_d_hip_yaw, W_hip_yaw, plant_w_spr,
                                            plant_w_spr);
  JointSpaceTrackingData swing_hip_yaw_right_traj("swing_hip_yaw_traj", K_p_hip_yaw,
                                            K_d_hip_yaw, W_hip_yaw, plant_w_spr,
                                            plant_w_spr);
  swing_hip_yaw_left_traj.AddStateAndJointToTrack(right_stance_state, "hip_yaw_left",
                                             "hip_yaw_leftdot");
  swing_hip_yaw_right_traj.AddStateAndJointToTrack(left_stance_state, "hip_yaw_right",
                                             "hip_yaw_rightdot");
  osc->AddConstTrackingData(&swing_hip_yaw_left_traj, VectorXd::Zero(1));
  osc->AddConstTrackingData(&swing_hip_yaw_right_traj, VectorXd::Zero(1));
  // Build OSC problem
  osc->Build();
  // Connect ports
  builder.Connect(state_receiver->get_output_port(0),
                  osc->get_robot_output_input_port());
  builder.Connect(state_receiver->get_output_port(0),
                  fsm->get_input_port_state());
  builder.Connect(fsm->get_output_port(0), osc->get_fsm_input_port());
  builder.Connect(fsm->get_output_port(0),
                  left_ft_traj_generator->get_fsm_input_port());
  builder.Connect(fsm->get_output_port(0),
                  right_ft_traj_generator->get_fsm_input_port());
  builder.Connect(state_receiver->get_output_port(0),
                  left_ft_traj_generator->get_state_input_port());
  builder.Connect(state_receiver->get_output_port(0),
                  right_ft_traj_generator->get_state_input_port());
  builder.Connect(left_ft_traj_generator->get_output_port(0),
                  osc->get_tracking_data_input_port("left_ft_traj"));
  builder.Connect(right_ft_traj_generator->get_output_port(0),
                  osc->get_tracking_data_input_port("right_ft_traj"));
  builder.Connect(left_toe_angle_traj_gen->get_output_port(0),
                  osc->get_tracking_data_input_port("left_toe_angle_traj"));
  builder.Connect(right_toe_angle_traj_gen->get_output_port(0),
                  osc->get_tracking_data_input_port("right_toe_angle_traj"));
  builder.Connect(fsm->get_output_port(0),
                  left_toe_angle_traj_gen->get_fsm_input_port());
  builder.Connect(fsm->get_output_port(0),
                  right_toe_angle_traj_gen->get_fsm_input_port());
  builder.Connect(state_receiver->get_output_port(0),
                  left_toe_angle_traj_gen->get_state_input_port());
  builder.Connect(state_receiver->get_output_port(0),
                  right_toe_angle_traj_gen->get_state_input_port());
  builder.Connect(osc->get_output_port(0), command_sender->get_input_port(0));
  if (FLAGS_publish_osc_data) {
    // Create osc debug sender.
    auto osc_debug_pub =
        builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_osc_output>(
            "OSC_DEBUG_SWING_FOOT", &lcm_local,
            TriggerTypeSet({TriggerType::kForced})));
    builder.Connect(osc->get_osc_debug_port(), osc_debug_pub->get_input_port());
  }

//  // Create osc debug sender.
//  auto osc_debug_pub_network =
//      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_osc_output>(
//          "NETWORK_OSC_DEBUG_SWING_FOOT", &lcm_network,
//          TriggerTypeSet({TriggerType::kPeriodic}), 0.02));
//  builder.Connect(osc->get_osc_debug_port(), osc_debug_pub_network->get_input_port());

  // Create the diagram
  auto owned_diagram = builder.Build();
  owned_diagram->set_name("osc walking controller");

  // Run lcm-driven simulation
  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm_local, std::move(owned_diagram), state_receiver, FLAGS_channel_x,
      true);
  loop.Simulate();

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::DoMain(argc, argv); }
