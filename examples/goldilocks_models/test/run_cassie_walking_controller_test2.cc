// (This file is modified from examples/Cassie/run_osc_walking_controller.cc)
// We use this script to test the performance of tracking a planned traj
// We only track for the first step

// This script also test the tracking for the mirror configuration (that is,
// when the robot is in right support phase)

#include <string>
#include <gflags/gflags.h>

#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "examples/Cassie/cassie_utils.h"
#include "examples/Cassie/osc/deviation_from_cp.h"
#include "examples/Cassie/osc/heading_traj_generator.h"
#include "examples/Cassie/osc/high_level_command.h"
#include "examples/Cassie/simulator_drift.h"
#include "examples/goldilocks_models/goldilocks_utils.h"
#include "examples/goldilocks_models/reduced_order_models.h"
#include "multibody/kinematic/kinematic_evaluator_set.h"
#include "multibody/multibody_utils.h"
#include "systems/controllers/cp_traj_gen.h"
#include "systems/controllers/lipm_traj_gen.h"
#include "systems/controllers/osc/operational_space_control.h"
#include "systems/controllers/time_based_fsm.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/framework/output_vector.h"
#include "systems/robot_lcm_systems.h"
#include "common/eigen_utils.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"

namespace dairlib::goldilocks_models {

using std::cout;
using std::endl;
using std::to_string;
using std::vector;

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using drake::MatrixX;
using drake::VectorX;
using drake::multibody::Frame;
using drake::multibody::JacobianWrtVariable;
using drake::systems::DiagramBuilder;
using drake::systems::TriggerType;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::lcm::TriggerTypeSet;
using drake::trajectories::PiecewisePolynomial;

using systems::OutputVector;
using systems::controllers::ComTrackingData;
using systems::controllers::JointSpaceTrackingData;
using systems::controllers::OptimalRomTrackingData;
using systems::controllers::RotTaskSpaceTrackingData;
using systems::controllers::TransTaskSpaceTrackingData;

using multibody::JwrtqdotToJwrtv;

DEFINE_int32(iter, 29, "The iteration # of the theta that you use");
DEFINE_bool(start_with_right_stance, false, "");

DEFINE_string(channel_x, "CASSIE_STATE_SIMULATION",
              "LCM channel for receiving state. "
              "Use CASSIE_STATE_SIMULATION to get state from simulator, and "
              "use CASSIE_STATE_DISPATCHER to get state from state estimator");
DEFINE_string(channel_u, "CASSIE_INPUT",
              "The name of the channel which publishes command");

DEFINE_bool(publish_osc_data, true,
            "whether to publish lcm messages for OscTrackData");
DEFINE_bool(print_osc, false, "whether to print the osc debug message or not");

DEFINE_bool(is_two_phase, false,
            "true: only right/left single support"
            "false: both double and single support");

DEFINE_double(drift_rate, 0.0, "Drift rate for floating-base state");

class OptimalRoMTrajGen : public drake::systems::LeafSystem<double> {
 public:
  OptimalRoMTrajGen(PiecewisePolynomial<double> desired_traj)
      : desired_traj_(desired_traj) {
    // Provide an instance to allocate the memory first (for the output)
    PiecewisePolynomial<double> pp(VectorXd(0));
    drake::trajectories::Trajectory<double>& traj_inst = pp;
    this->DeclareAbstractOutputPort("rom_lipm_traj", traj_inst,
                                    &OptimalRoMTrajGen::CalcDesiredTraj);
  };

 private:
  void CalcDesiredTraj(const drake::systems::Context<double>& context,
                       drake::trajectories::Trajectory<double>* traj) const {
    // Copy traj
    PiecewisePolynomial<double>* pp_traj =
        (PiecewisePolynomial<double>*)dynamic_cast<
            PiecewisePolynomial<double>*>(traj);
    *pp_traj = desired_traj_;
  };

  PiecewisePolynomial<double> desired_traj_;
};

// Currently the controller runs at the rate between 500 Hz and 200 Hz, so the
// publish rate of the robot state needs to be less than 500 Hz. Otherwise, the
// performance seems to degrade due to this. (Recommended publish rate: 200 Hz)
// Maybe we need to update the lcm driven loop to clear the queue of lcm message
// if it's more than one message?
int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Build Cassie MBP
  drake::multibody::MultibodyPlant<double> plant_w_springs(0.0);
  addCassieMultibody(&plant_w_springs, nullptr, true /*floating base*/,
                     "examples/Cassie/urdf/cassie_v2.urdf",
                     true /*spring model*/, false /*loop closure*/);
  plant_w_springs.Finalize();
  // Build fix-spring Cassie MBP
  drake::multibody::MultibodyPlant<double> plant_wo_springs(0.0);
  addCassieMultibody(&plant_wo_springs, nullptr, true,
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

  // Reduced order model
  const std::string dir_model =
      "../dairlib_data/goldilocks_models/planning/robot_1/models/";
  std::unique_ptr<ReducedOrderModel> rom =
      CreateRom(4 /*rom_option*/, 1 /*robot_option*/, plant_wo_springs, true);
  ReadModelParameters(rom.get(), dir_model, FLAGS_iter);

  // Get desired traj from ROM planner result
  const std::string dir_data =
      "../dairlib_data/goldilocks_models/planning/robot_1/data/";
  VectorXd time_at_knots =
      readCSV(dir_data + std::string("time_at_knots.csv")).col(0);
  MatrixXd state_at_knots =
      readCSV(dir_data + std::string("state_at_knots.csv"));
  std::vector<double> T_waypoint = CopyVectorXdToStdVector(time_at_knots);
  std::vector<MatrixXd> y(T_waypoint.size(), MatrixXd::Zero(rom->n_y(), 1));
  std::vector<MatrixXd> y_dot(T_waypoint.size(), MatrixXd::Zero(rom->n_y(), 1));
  for (int i = 0; i < T_waypoint.size(); i++) {
    y.at(i) = state_at_knots.col(i).head(rom->n_y());
    y_dot.at(i) = state_at_knots.col(i).tail(rom->n_y());
  }
  PiecewisePolynomial<double> desired_rom_traj =
      PiecewisePolynomial<double>::CubicHermite(T_waypoint, y, y_dot);

  // Read in the end time of the trajectory
  int knots_per_foot_step = readCSV(dir_data + "nodes_per_step.csv")(0, 0);
  double end_time_of_first_step =
      readCSV(dir_data + "time_at_knots.csv")(knots_per_foot_step, 0);

  // Build the controller diagram
  DiagramBuilder<double> builder;

  drake::lcm::DrakeLcm lcm_local("udpm://239.255.76.67:7667?ttl=0");

  // Get body frames and points
  Vector3d mid_contact_point = (left_toe.first + left_heel.first) / 2;
  auto left_toe_mid = std::pair<const Vector3d, const Frame<double>&>(
      mid_contact_point, plant_w_springs.GetFrameByName("toe_left"));
  auto right_toe_mid = std::pair<const Vector3d, const Frame<double>&>(
      mid_contact_point, plant_w_springs.GetFrameByName("toe_right"));
  auto left_toe_origin = std::pair<const Vector3d, const Frame<double>&>(
      Vector3d::Zero(), plant_w_springs.GetFrameByName("toe_left"));
  auto right_toe_origin = std::pair<const Vector3d, const Frame<double>&>(
      Vector3d::Zero(), plant_w_springs.GetFrameByName("toe_right"));

  // Create state receiver.
  auto state_receiver =
      builder.AddSystem<systems::RobotOutputReceiver>(plant_w_springs);

  // Create command sender.
  auto command_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          FLAGS_channel_u, &lcm_local, TriggerTypeSet({TriggerType::kForced})));
  auto command_sender =
      builder.AddSystem<systems::RobotCommandSender>(plant_w_springs);

  builder.Connect(command_sender->get_output_port(0),
                  command_pub->get_input_port());

  // Add emulator for floating base drift
  Eigen::VectorXd drift_mean =
      Eigen::VectorXd::Zero(plant_w_springs.num_positions());
  Eigen::MatrixXd drift_cov = Eigen::MatrixXd::Zero(
      plant_w_springs.num_positions(), plant_w_springs.num_positions());
  drift_cov(4, 4) = FLAGS_drift_rate;  // x
  drift_cov(5, 5) = FLAGS_drift_rate;  // y
  drift_cov(6, 6) = FLAGS_drift_rate;  // z
  // Note that we didn't add drift to yaw angle here because it requires
  // changing SimulatorDrift.

  auto simulator_drift =
      builder.AddSystem<SimulatorDrift>(plant_w_springs, drift_mean, drift_cov);
  builder.Connect(state_receiver->get_output_port(0),
                  simulator_drift->get_input_port_state());

  // Create human high-level control
  Eigen::Vector2d global_target_position(1, 0);
  Eigen::Vector2d params_of_no_turning(5, 1);
  // Logistic function 1/(1+5*exp(x-1))
  // The function ouputs 0.0007 when x = 0
  //                     0.5    when x = 1
  //                     0.9993 when x = 2
  auto high_level_command = builder.AddSystem<cassie::osc::HighLevelCommand>(
      plant_w_springs, context_w_spr.get(), global_target_position,
      params_of_no_turning);
  builder.Connect(state_receiver->get_output_port(0),
                  high_level_command->get_state_input_port());

  // Create heading traj generator
  auto head_traj_gen = builder.AddSystem<cassie::osc::HeadingTrajGenerator>(
      plant_w_springs, context_w_spr.get());
  builder.Connect(simulator_drift->get_output_port(0),
                  head_traj_gen->get_state_input_port());
  builder.Connect(high_level_command->get_yaw_output_port(),
                  head_traj_gen->get_yaw_input_port());

  // Create finite state machine
  int left_stance_state = 0;
  int right_stance_state = 1;
  int double_support_state = 2;
  double left_support_duration = end_time_of_first_step;
  double right_support_duration = end_time_of_first_step;
  double double_support_duration = 0.02;
  vector<int> fsm_states;
  vector<double> state_durations;
  if (FLAGS_is_two_phase) {
    fsm_states = {left_stance_state, right_stance_state};
    state_durations = {left_support_duration, right_support_duration};
  } else {
    fsm_states = {left_stance_state, double_support_state, right_stance_state,
                  double_support_state};
    state_durations = {left_support_duration, double_support_duration,
                       right_support_duration, double_support_duration};
  }
  auto fsm = builder.AddSystem<systems::TimeBasedFiniteStateMachine>(
      plant_w_springs, fsm_states, state_durations);
  builder.Connect(simulator_drift->get_output_port(0),
                  fsm->get_input_port_state());

  // Create CoM trajectory generator
  double desired_com_height = 0.89;
  vector<int> unordered_fsm_states;
  vector<double> unordered_state_durations;
  vector<vector<std::pair<const Vector3d, const Frame<double>&>>>
      contact_points_in_each_state;
  if (FLAGS_is_two_phase) {
    unordered_fsm_states = {left_stance_state, right_stance_state};
    unordered_state_durations = {left_support_duration, right_support_duration};
    contact_points_in_each_state.push_back({left_toe_mid});
    contact_points_in_each_state.push_back({right_toe_mid});
  } else {
    unordered_fsm_states = {left_stance_state, right_stance_state,
                            double_support_state};
    unordered_state_durations = {left_support_duration, right_support_duration,
                                 double_support_duration};
    contact_points_in_each_state.push_back({left_toe_mid});
    contact_points_in_each_state.push_back({right_toe_mid});
    contact_points_in_each_state.push_back({left_toe_mid, right_toe_mid});
  }
  auto lipm_traj_generator = builder.AddSystem<systems::LIPMTrajGenerator>(
      plant_w_springs, context_w_spr.get(), desired_com_height,
      unordered_fsm_states, unordered_state_durations,
      contact_points_in_each_state);
  builder.Connect(fsm->get_output_port(0),
                  lipm_traj_generator->get_input_port_fsm());
  builder.Connect(simulator_drift->get_output_port(0),
                  lipm_traj_generator->get_input_port_state());

  // Create velocity control by foot placement
  auto deviation_from_cp =
      builder.AddSystem<cassie::osc::DeviationFromCapturePoint>(
          plant_w_springs, context_w_spr.get());
  builder.Connect(high_level_command->get_xy_output_port(),
                  deviation_from_cp->get_input_port_des_hor_vel());
  builder.Connect(simulator_drift->get_output_port(0),
                  deviation_from_cp->get_input_port_state());

  // Create swing leg trajectory generator (capture point)
  double mid_foot_height = 0.1;
  // Since the ground is soft in the simulation, we raise the desired final
  // foot height by 1 cm. The controller is sensitive to this number, should
  // tune this every time we change the simulation parameter or when we move
  // to the hardware testing.
  // Additionally, implementing a double support phase might mitigate the
  // instability around state transition.
  double desired_final_foot_height = 0.01;
  double desired_final_vertical_foot_velocity = 0;  //-1;
  double max_CoM_to_CP_dist = 0.4;
  double cp_offset = 0.06;
  double center_line_offset = 0.06;
  vector<int> left_right_support_fsm_states = {left_stance_state,
                                               right_stance_state};
  vector<double> left_right_support_state_durations = {left_support_duration,
                                                       right_support_duration};
  vector<std::pair<const Vector3d, const Frame<double>&>> left_right_foot = {
      left_toe_origin, right_toe_origin};
  auto cp_traj_generator = builder.AddSystem<systems::CPTrajGenerator>(
      plant_w_springs, context_w_spr.get(), left_right_support_fsm_states,
      left_right_support_state_durations, left_right_foot, "pelvis",
      mid_foot_height, desired_final_foot_height,
      desired_final_vertical_foot_velocity, max_CoM_to_CP_dist, true, true,
      true, cp_offset, center_line_offset);
  builder.Connect(fsm->get_output_port(0),
                  cp_traj_generator->get_input_port_fsm());
  builder.Connect(simulator_drift->get_output_port(0),
                  cp_traj_generator->get_input_port_state());
  builder.Connect(lipm_traj_generator->get_output_port(0),
                  cp_traj_generator->get_input_port_com());
  builder.Connect(deviation_from_cp->get_output_port(0),
                  cp_traj_generator->get_input_port_fp());

  // Create optimal rom trajectory generator
  auto optimal_rom_traj_gen =
      builder.AddSystem<OptimalRoMTrajGen>(desired_rom_traj);

  // Create Operational space control
  auto osc = builder.AddSystem<systems::controllers::OperationalSpaceControl>(
      plant_w_springs, plant_wo_springs, context_w_spr.get(),
      context_wo_spr.get(), true, FLAGS_print_osc /*print_tracking_info*/);

  // Cost
  int n_v = plant_wo_springs.num_velocities();
  MatrixXd Q_accel = 2 * MatrixXd::Identity(n_v, n_v);
  osc->SetAccelerationCostForAllJoints(Q_accel);

  // Distance constraint
  multibody::KinematicEvaluatorSet<double> evaluators(plant_wo_springs);
  auto left_loop = LeftLoopClosureEvaluator(plant_wo_springs);
  auto right_loop = RightLoopClosureEvaluator(plant_wo_springs);
  evaluators.add_evaluator(&left_loop);
  evaluators.add_evaluator(&right_loop);
  osc->AddKinematicConstraint(&evaluators);

  // Soft constraint
  // w_contact_relax shouldn't be too big, cause we want tracking error to be
  // important
  double w_contact_relax = 2000;
  osc->SetWeightOfSoftContactConstraint(w_contact_relax);
  // Friction coefficient
  double mu = 0.4;
  osc->SetContactFriction(mu);
  // Add contact points (The position doesn't matter. It's not used in OSC)
  auto left_toe_evaluator = multibody::WorldPointEvaluator(
      plant_wo_springs, left_toe.first, left_toe.second, Matrix3d::Identity(),
      Vector3d::Zero(), {1, 2});
  auto left_heel_evaluator = multibody::WorldPointEvaluator(
      plant_wo_springs, left_heel.first, left_heel.second, Matrix3d::Identity(),
      Vector3d::Zero(), {0, 1, 2});
  auto right_toe_evaluator = multibody::WorldPointEvaluator(
      plant_wo_springs, right_toe.first, right_toe.second, Matrix3d::Identity(),
      Vector3d::Zero(), {1, 2});
  auto right_heel_evaluator = multibody::WorldPointEvaluator(
      plant_wo_springs, right_heel.first, right_heel.second,
      Matrix3d::Identity(), Vector3d::Zero(), {0, 1, 2});
  osc->AddStateAndContactPoint(left_stance_state, &left_toe_evaluator);
  osc->AddStateAndContactPoint(left_stance_state, &left_heel_evaluator);
  osc->AddStateAndContactPoint(right_stance_state, &right_toe_evaluator);
  osc->AddStateAndContactPoint(right_stance_state, &right_heel_evaluator);
  if (!FLAGS_is_two_phase) {
    osc->AddStateAndContactPoint(double_support_state, &left_toe_evaluator);
    osc->AddStateAndContactPoint(double_support_state, &left_heel_evaluator);
    osc->AddStateAndContactPoint(double_support_state, &right_toe_evaluator);
    osc->AddStateAndContactPoint(double_support_state, &right_heel_evaluator);
  }

  // Swing foot tracking
  MatrixXd W_swing_foot = 400 * MatrixXd::Identity(3, 3);
  MatrixXd K_p_sw_ft = 100 * MatrixXd::Identity(3, 3);
  MatrixXd K_d_sw_ft = 10 * MatrixXd::Identity(3, 3);
  TransTaskSpaceTrackingData swing_foot_traj("cp_traj", K_p_sw_ft, K_d_sw_ft,
                                             W_swing_foot, plant_w_springs,
                                             plant_wo_springs);
  swing_foot_traj.AddStateAndPointToTrack(left_stance_state, "toe_right");
  swing_foot_traj.AddStateAndPointToTrack(right_stance_state, "toe_left");
  osc->AddTrackingData(&swing_foot_traj);
  // Center of mass tracking (Using RomTrackingData with initial ROM being COM)
  MatrixXd W_com = MatrixXd::Identity(3, 3);
  W_com(0, 0) = 2;
  W_com(1, 1) = 2;
  W_com(2, 2) = 2000;
  MatrixXd K_p_com = 50 * MatrixXd::Identity(3, 3);
  MatrixXd K_d_com = 10 * MatrixXd::Identity(3, 3);
  OptimalRomTrackingData center_of_mass_traj("rom_lipm_traj", K_p_com, K_d_com,
                                             W_com, plant_w_springs,
                                             plant_wo_springs, *rom);
  osc->AddTrackingData(&center_of_mass_traj);
  // Pelvis rotation tracking (pitch and roll)
  double w_pelvis_balance = 200;
  double k_p_pelvis_balance = 200;
  double k_d_pelvis_balance = 80;
  Matrix3d W_pelvis_balance = MatrixXd::Zero(3, 3);
  W_pelvis_balance(0, 0) = w_pelvis_balance;
  W_pelvis_balance(1, 1) = w_pelvis_balance;
  Matrix3d K_p_pelvis_balance = MatrixXd::Zero(3, 3);
  K_p_pelvis_balance(0, 0) = k_p_pelvis_balance;
  K_p_pelvis_balance(1, 1) = k_p_pelvis_balance;
  Matrix3d K_d_pelvis_balance = MatrixXd::Zero(3, 3);
  K_d_pelvis_balance(0, 0) = k_d_pelvis_balance;
  K_d_pelvis_balance(1, 1) = k_d_pelvis_balance;
  RotTaskSpaceTrackingData pelvis_balance_traj(
      "pelvis_balance_traj", K_p_pelvis_balance, K_d_pelvis_balance,
      W_pelvis_balance, plant_w_springs, plant_wo_springs);
  pelvis_balance_traj.AddFrameToTrack("pelvis");
  osc->AddTrackingData(&pelvis_balance_traj);
  // Pelvis rotation tracking (yaw)
  double w_heading = 200;
  double k_p_heading = 50;
  double k_d_heading = 40;
  Matrix3d W_pelvis_heading = MatrixXd::Zero(3, 3);
  W_pelvis_heading(2, 2) = w_heading;
  Matrix3d K_p_pelvis_heading = MatrixXd::Zero(3, 3);
  K_p_pelvis_heading(2, 2) = k_p_heading;
  Matrix3d K_d_pelvis_heading = MatrixXd::Zero(3, 3);
  K_d_pelvis_heading(2, 2) = k_d_heading;
  RotTaskSpaceTrackingData pelvis_heading_traj(
      "pelvis_heading_traj", K_p_pelvis_heading, K_d_pelvis_heading,
      W_pelvis_heading, plant_w_springs, plant_wo_springs);
  pelvis_heading_traj.AddFrameToTrack("pelvis");
  osc->AddTrackingData(&pelvis_heading_traj, 0.1);  // 0.05
  // Swing toe joint tracking (Currently use fix position)
  // The desired position, -1.5, was derived heuristically. It is roughly the
  // toe angle when Cassie stands on the ground.
  MatrixXd W_swing_toe = 200 * MatrixXd::Identity(1, 1);
  MatrixXd K_p_swing_toe = 200 * MatrixXd::Identity(1, 1);
  MatrixXd K_d_swing_toe = 20 * MatrixXd::Identity(1, 1);
  JointSpaceTrackingData swing_toe_traj("swing_toe_traj", K_p_swing_toe,
                                        K_d_swing_toe, W_swing_toe,
                                        plant_w_springs, plant_wo_springs);
  swing_toe_traj.AddStateAndJointToTrack(left_stance_state, "toe_right",
                                         "toe_rightdot");
  swing_toe_traj.AddStateAndJointToTrack(right_stance_state, "toe_left",
                                         "toe_leftdot");
  osc->AddConstTrackingData(&swing_toe_traj, -1.5 * VectorXd::Ones(1), 0, 0.3);
  // Swing hip yaw joint tracking
  MatrixXd W_hip_yaw = 20 * MatrixXd::Identity(1, 1);
  MatrixXd K_p_hip_yaw = 200 * MatrixXd::Identity(1, 1);
  MatrixXd K_d_hip_yaw = 160 * MatrixXd::Identity(1, 1);
  JointSpaceTrackingData swing_hip_yaw_traj("swing_hip_yaw_traj", K_p_hip_yaw,
                                            K_d_hip_yaw, W_hip_yaw,
                                            plant_w_springs, plant_wo_springs);
  swing_hip_yaw_traj.AddStateAndJointToTrack(left_stance_state, "hip_yaw_right",
                                             "hip_yaw_rightdot");
  swing_hip_yaw_traj.AddStateAndJointToTrack(right_stance_state, "hip_yaw_left",
                                             "hip_yaw_leftdot");
  osc->AddConstTrackingData(&swing_hip_yaw_traj, VectorXd::Zero(1));
  // Build OSC problem
  osc->Build();
  // Connect ports
  builder.Connect(simulator_drift->get_output_port(0),
                  osc->get_robot_output_input_port());
  builder.Connect(fsm->get_output_port(0), osc->get_fsm_input_port());
  builder.Connect(optimal_rom_traj_gen->get_output_port(0),
                  osc->get_tracking_data_input_port("rom_lipm_traj"));
  builder.Connect(cp_traj_generator->get_output_port(0),
                  osc->get_tracking_data_input_port("cp_traj"));
  builder.Connect(head_traj_gen->get_output_port(0),
                  osc->get_tracking_data_input_port("pelvis_balance_traj"));
  builder.Connect(head_traj_gen->get_output_port(0),
                  osc->get_tracking_data_input_port("pelvis_heading_traj"));
  builder.Connect(osc->get_output_port(0), command_sender->get_input_port(0));
  if (FLAGS_publish_osc_data) {
    // Create osc debug sender.
    auto osc_debug_pub =
        builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_osc_output>(
            "OSC_DEBUG", &lcm_local, TriggerTypeSet({TriggerType::kForced})));
    builder.Connect(osc->get_osc_debug_port(), osc_debug_pub->get_input_port());
  }

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

}  // namespace dairlib::goldilocks_models

int main(int argc, char* argv[]) {
  return dairlib::goldilocks_models::DoMain(argc, argv);
}
