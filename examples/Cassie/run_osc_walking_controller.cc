#include <math.h>

#include <gflags/gflags.h>

#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/analysis/simulator.h"

#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"

#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_pd_config.hpp"
#include "systems/robot_lcm_systems.h"
#include "examples/Cassie/cassie_utils.h"
#include "attic/multibody/rigidbody_utils.h"

#include "examples/Cassie/osc_walking_control/foot_placement_control.h"
#include "systems/controllers/cp_traj_gen.h"
#include "systems/controllers/lipm_traj_gen.h"
#include "systems/controllers/time_based_fsm.h"
#include "systems/controllers/operational_space_control/osc_utils.h"


namespace dairlib {

using std::cout;
using std::endl;

using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::Matrix3d;
using Eigen::MatrixXd;

using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::DiagramBuilder;

using multibody::GetBodyIndexFromName;
using systems::controllers::TransTaskSpaceTrackingData;
using systems::controllers::RotTaskSpaceTrackingData;
using systems::controllers::JointSpaceTrackingData;

DEFINE_double(end_time, 0.01, "End time of simulation");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  DiagramBuilder<double> builder;

  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>(
               "udpm://239.255.76.67:7667?ttl=0");

  RigidBodyTree<double> tree_with_springs;
  RigidBodyTree<double> tree_without_springs;
  buildCassieTree(tree_with_springs,
                  "examples/Cassie/urdf/cassie_v2.urdf",
                  drake::multibody::joints::kQuaternion);
  buildCassieTree(tree_without_springs,
                  "examples/Cassie/urdf/cassie_fixed_springs.urdf",
                  drake::multibody::joints::kQuaternion, false/*no spring*/);
  const double terrain_size = 100;
  const double terrain_depth = 0.20;
  drake::multibody::AddFlatTerrainToWorld(&tree_with_springs,
                                          terrain_size, terrain_depth);
  drake::multibody::AddFlatTerrainToWorld(&tree_without_springs,
                                          terrain_size, terrain_depth);

  const std::string channel_x = "CASSIE_STATE";
  const std::string channel_u = "CASSIE_INPUT";

  // Create state receiver.
  auto state_sub = builder.AddSystem(
                     LcmSubscriberSystem::Make<dairlib::lcmt_robot_output>(
                       channel_x, lcm));
  auto state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(
                          tree_with_springs);
  builder.Connect(state_sub->get_output_port(),
                  state_receiver->get_input_port(0));

  // Create command sender.
  auto command_pub = builder.AddSystem(
                       LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
                         channel_u, lcm, 1.0 / 1000.0));
  auto command_sender = builder.AddSystem<systems::RobotCommandSender>(
                          tree_with_springs);

  builder.Connect(command_sender->get_output_port(0),
                  command_pub->get_input_port());

  // Get body indices for cassie with springs
  int pelvis_idx = GetBodyIndexFromName(tree_with_springs, "pelvis");
  int left_toe_idx = GetBodyIndexFromName(tree_with_springs, "toe_left");
  int right_toe_idx = GetBodyIndexFromName(tree_with_springs, "toe_right");
  DRAKE_DEMAND(pelvis_idx != -1 && left_toe_idx != -1 && right_toe_idx != -1);

  // Create finite state machine
  int left_stance_state = 2;
  int right_stance_state = 3;
  int initial_state_number = 2;
  double duration_per_state = 0.35;
  double time_shift = 0;
  auto fsm = builder.AddSystem<systems::TimeBasedFiniteStateMachine>(
               &tree_with_springs,
               left_stance_state, right_stance_state, initial_state_number,
               duration_per_state, time_shift);
  builder.Connect(state_receiver->get_output_port(0),
                  fsm->get_input_port_state());

  // Create CoM trajectory generator
  double desired_com_height = 0.89;
  auto lipm_traj_generator =
    builder.AddSystem<systems::LIPMTrajGenerator>(&tree_with_springs,
        desired_com_height,
        duration_per_state,
        left_stance_state,
        right_stance_state,
        left_toe_idx,
        Eigen::VectorXd::Zero(3),
        right_toe_idx,
        Eigen::VectorXd::Zero(3));
  builder.Connect(fsm->get_output_port(0),
                  lipm_traj_generator->get_input_port_fsm());
  builder.Connect(state_receiver->get_output_port(0),
                  lipm_traj_generator->get_input_port_state());

  // Create foot placement control block
  Eigen::Vector2d global_target_position(5, 0);
  double circle_radius_of_no_turning = 1;
  auto foot_placement_control =
    builder.AddSystem<cassie::cp_control::FootPlacementControl>(
      &tree_with_springs, pelvis_idx,
      global_target_position, circle_radius_of_no_turning);
  builder.Connect(state_receiver->get_output_port(0),
                  foot_placement_control->get_input_port_state());

  // Create swing leg trajectory generator (capture point)
  double mid_foot_height = 0.1 + 0.05;
  double desired_final_foot_height = -0.05; //0.05
  double desired_final_vertical_foot_velocity = -1;
  double max_CoM_to_CP_dist = 0.4;
  double cp_offset = 0.06;
  double center_line_offset = 0.06;
  auto cp_traj_generator =
    builder.AddSystem<systems::CPTrajGenerator>(&tree_with_springs,
        mid_foot_height,
        desired_final_foot_height,
        desired_final_vertical_foot_velocity,
        max_CoM_to_CP_dist,
        duration_per_state,
        left_stance_state,
        right_stance_state,
        left_toe_idx,
        Eigen::VectorXd::Zero(3),
        right_toe_idx,
        Eigen::VectorXd::Zero(3),
        pelvis_idx,
        true, true, true,
        cp_offset,
        center_line_offset);
  builder.Connect(fsm->get_output_port(0),
                  cp_traj_generator->get_input_port_fsm());
  builder.Connect(state_receiver->get_output_port(0),
                  cp_traj_generator->get_input_port_state());
  builder.Connect(lipm_traj_generator->get_output_port(0),
                  cp_traj_generator->get_input_port_com());
  builder.Connect(foot_placement_control->get_output_port(0),
                  cp_traj_generator->get_input_port_fp());

  // Create Operational space control
  auto osc = builder.AddSystem<systems::controllers::OperationalSpaceControl>(
               &tree_with_springs, &tree_without_springs, true, true);
  // Get body index
  int pelvis_idx_w_spr = pelvis_idx;
  int left_toe_idx_w_spr = left_toe_idx;
  int right_toe_idx_w_spr = right_toe_idx;
  int pelvis_idx_wo_spr = GetBodyIndexFromName(
                            tree_without_springs, "pelvis");
  int left_toe_idx_wo_spr = GetBodyIndexFromName(
                              tree_without_springs, "toe_left");
  int right_toe_idx_wo_spr = GetBodyIndexFromName(
                               tree_without_springs, "toe_right");
  // Get position/velocity index
  std::map<std::string, int> pos_idx_map_w_spr =
    multibody::makeNameToPositionsMap(tree_with_springs);
  std::map<std::string, int> vel_idx_map_w_spr =
    multibody::makeNameToVelocitiesMap(tree_with_springs);
  int left_toe_pos_idx_w_spr = pos_idx_map_w_spr.at("toe_left");
  int right_toe_pos_idx_w_spr = pos_idx_map_w_spr.at("toe_right");
  int left_hip_yaw_pos_idx_w_spr = pos_idx_map_w_spr.at("hip_yaw_left");
  int right_hip_yaw_pos_idx_w_spr = pos_idx_map_w_spr.at("hip_yaw_right");
  int left_toe_vel_idx_w_spr = vel_idx_map_w_spr.at("toe_leftdot");
  int right_toe_vel_idx_w_spr = vel_idx_map_w_spr.at("toe_rightdot");
  int left_hip_yaw_vel_idx_w_spr = vel_idx_map_w_spr.at("hip_yaw_leftdot");
  int right_hip_yaw_vel_idx_w_spr = vel_idx_map_w_spr.at("hip_yaw_rightdot");
  std::map<std::string, int> pos_idx_map_wo_spr =
    multibody::makeNameToPositionsMap(tree_without_springs);
  std::map<std::string, int> vel_idx_map_wo_spr =
    multibody::makeNameToVelocitiesMap(tree_without_springs);
  int left_toe_pos_idx_wo_spr = pos_idx_map_wo_spr.at("toe_left");
  int right_toe_pos_idx_wo_spr = pos_idx_map_wo_spr.at("toe_right");
  int left_hip_yaw_pos_idx_wo_spr = pos_idx_map_wo_spr.at("hip_yaw_left");
  int right_hip_yaw_pos_idx_wo_spr = pos_idx_map_wo_spr.at("hip_yaw_right");
  int left_toe_vel_idx_wo_spr = vel_idx_map_wo_spr.at("toe_leftdot");
  int right_toe_vel_idx_wo_spr = vel_idx_map_wo_spr.at("toe_rightdot");
  int left_hip_yaw_vel_idx_wo_spr = vel_idx_map_wo_spr.at("hip_yaw_leftdot");
  int right_hip_yaw_vel_idx_wo_spr = vel_idx_map_wo_spr.at("hip_yaw_rightdot");

  int n_v = tree_without_springs.get_num_velocities();
  // Cost
  // cout << "Adding cost\n";
  MatrixXd Q_accel = 0.00002 * MatrixXd::Identity(n_v, n_v);
  osc->SetAccelerationCostForAllJoints(Q_accel);
  double w_toe = 0.1;  // 1
  osc->AddAccelerationCost(left_toe_vel_idx_wo_spr, w_toe);
  osc->AddAccelerationCost(right_toe_vel_idx_wo_spr, w_toe);
  // Soft constraint
  // cout << "Adding constraint\n";
  // We don't want this to be too big, cause we want tracking error to be important
  double w_contact_relax = 200;
  osc->SetWeightOfSoftContactConstraint(w_contact_relax);
  // Firction coefficient
  double mu = 0.8;
  osc->SetContactFriction(mu);
  Vector3d front_contact_disp(-0.0457, 0.112, 0);
  Vector3d rear_contact_disp(0.088, 0, 0);
  osc->AddStateAndContactPoint(left_stance_state,
                               left_toe_idx_wo_spr, front_contact_disp);
  osc->AddStateAndContactPoint(left_stance_state,
                               left_toe_idx_wo_spr, rear_contact_disp);
  osc->AddStateAndContactPoint(right_stance_state,
                               right_toe_idx_wo_spr, front_contact_disp);
  osc->AddStateAndContactPoint(right_stance_state,
                               right_toe_idx_wo_spr, rear_contact_disp);
  // Swing foot tracking
  // cout << "Adding swing foot tracking\n";
  MatrixXd W_swing_foot = 200 * MatrixXd::Identity(3, 3);
  MatrixXd K_p_sw_ft = 100 * MatrixXd::Identity(3, 3);
  MatrixXd K_d_sw_ft = 10 * MatrixXd::Identity(3, 3);
  TransTaskSpaceTrackingData swing_foot_traj("cp_traj", 3,
      K_p_sw_ft, K_d_sw_ft, W_swing_foot);
  swing_foot_traj.AddStateAndPointToTrack(left_stance_state,
                                          right_toe_idx_w_spr,
                                          right_toe_idx_wo_spr);
  swing_foot_traj.AddStateAndPointToTrack(right_stance_state,
                                          left_toe_idx_w_spr,
                                          left_toe_idx_wo_spr);
  osc->AddTrackingData(&swing_foot_traj);
  // Center of mass tracking
  // cout << "Adding center of mass tracking\n";
  MatrixXd W_com = MatrixXd::Identity(3, 3);
  W_com(0, 0) = 2;
  W_com(1, 1) = 2;
  W_com(2, 2) = 2000;
  MatrixXd K_p_com = 50 * MatrixXd::Identity(3, 3);
  MatrixXd K_d_com = 10 * MatrixXd::Identity(3, 3);
  TransTaskSpaceTrackingData center_of_mass_traj("lipm_traj", 3,
      K_p_com, K_d_com, W_com, false, true, true);
  osc->AddTrackingData(&center_of_mass_traj);
  // Pelvis rotation tracking
  // cout << "Adding pelvis rotation tracking\n";
  double w_pelvis_balance = 200;
  double w_heading = 200;
  double k_p_pelvis_balance = 100;
  double k_d_pelvis_balance = 80;
  double k_p_heading = 50;
  double k_d_heading = 40;
  Matrix3d W_pelvis = MatrixXd::Identity(3, 3);
  W_pelvis(0, 0) = w_pelvis_balance;
  W_pelvis(1, 1) = w_pelvis_balance;
  W_pelvis(2, 2) = w_heading;
  Matrix3d K_p_pelvis = MatrixXd::Identity(3, 3);
  K_p_pelvis(0, 0) = k_p_pelvis_balance * 2;
  K_p_pelvis(1, 1) = k_p_pelvis_balance * 2;
  K_p_pelvis(2, 2) = k_p_heading;
  Matrix3d K_d_pelvis = MatrixXd::Identity(3, 3);
  K_d_pelvis(0, 0) = k_d_pelvis_balance;
  K_d_pelvis(1, 1) = k_d_pelvis_balance;
  K_d_pelvis(2, 2) = k_d_heading;
  RotTaskSpaceTrackingData pelvis_rot_traj("pelvis_rot_traj", 3,
      K_p_pelvis, K_d_pelvis, W_pelvis, true, false);
  pelvis_rot_traj.AddFrameToTrack(pelvis_idx_w_spr, pelvis_idx_wo_spr);
  pelvis_rot_traj.SetNoControlPeriod(
    0.05); // TODO separate yaw from the rest /////////////////////////////////////////
  VectorXd pelvis_desired_quat(4);
  pelvis_desired_quat << 1, 0, 0, 0;
  pelvis_rot_traj.SetConstantTraj(pelvis_desired_quat);
  osc->AddTrackingData(&pelvis_rot_traj);
  // Swing toe joint tracking (Currently use fix position) TODO: change this
  // cout << "Adding swing toe joint tracking\n";
  MatrixXd W_swing_toe = 2 * MatrixXd::Identity(1, 1);
  MatrixXd K_p_swing_toe = 1000 * MatrixXd::Identity(1, 1);
  MatrixXd K_d_swing_toe = 100 * MatrixXd::Identity(1, 1);
  JointSpaceTrackingData swing_toe_traj("swing_toe_traj", 1,
                                        K_p_swing_toe, K_d_swing_toe,
                                        W_swing_toe, true);
  swing_toe_traj.AddStateAndJointToTrack(left_stance_state,
                                         right_toe_pos_idx_w_spr,
                                         right_toe_vel_idx_w_spr,
                                         right_toe_pos_idx_wo_spr,
                                         right_toe_vel_idx_wo_spr);
  swing_toe_traj.AddStateAndJointToTrack(right_stance_state,
                                         left_toe_pos_idx_w_spr,
                                         left_toe_vel_idx_w_spr,
                                         left_toe_pos_idx_wo_spr,
                                         left_toe_vel_idx_wo_spr);
  swing_toe_traj.SetConstantTraj(-M_PI * VectorXd::Ones(1));
  osc->AddTrackingData(&swing_toe_traj);
  // Stance toe joint tracking (Currently use fix position) TODO: change this
  // cout << "Adding stance toe joint tracking\n";
  MatrixXd W_stance_toe = 2 * MatrixXd::Identity(1, 1);
  MatrixXd K_p_stance_toe = 100 * MatrixXd::Identity(1, 1);
  MatrixXd K_d_stance_toe = 20 * MatrixXd::Identity(1, 1);
  JointSpaceTrackingData stance_toe_traj("stance_toe_traj", 1,
                                         K_p_stance_toe, K_d_stance_toe,
                                         W_stance_toe, true);
  stance_toe_traj.AddStateAndJointToTrack(left_stance_state,
                                          left_toe_pos_idx_w_spr,
                                          left_toe_vel_idx_w_spr,
                                          left_toe_pos_idx_wo_spr,
                                          left_toe_vel_idx_wo_spr);
  stance_toe_traj.AddStateAndJointToTrack(right_stance_state,
                                          right_toe_pos_idx_w_spr,
                                          right_toe_vel_idx_w_spr,
                                          right_toe_pos_idx_wo_spr,
                                          right_toe_vel_idx_wo_spr);
  stance_toe_traj.SetConstantTraj(-M_PI * VectorXd::Ones(1));
  osc->AddTrackingData(&stance_toe_traj);
  // Swing hip yaw joint tracking
  // cout << "Adding swing hip yaw joint tracking\n";
  MatrixXd W_hip_yaw = 20 * MatrixXd::Identity(1, 1);
  MatrixXd K_p_hip_yaw = 200 * MatrixXd::Identity(1, 1);
  MatrixXd K_d_hip_yaw = 160 * MatrixXd::Identity(1, 1);
  JointSpaceTrackingData swing_hip_yaw_traj("swing_hip_yaw_traj", 1,
      K_p_hip_yaw, K_d_hip_yaw, W_hip_yaw, true);
  swing_hip_yaw_traj.AddStateAndJointToTrack(left_stance_state,
      right_hip_yaw_pos_idx_w_spr,
      right_hip_yaw_vel_idx_w_spr,
      right_hip_yaw_pos_idx_wo_spr,
      right_hip_yaw_vel_idx_wo_spr);
  swing_hip_yaw_traj.AddStateAndJointToTrack(right_stance_state,
      left_hip_yaw_pos_idx_w_spr,
      left_hip_yaw_vel_idx_w_spr,
      left_hip_yaw_pos_idx_wo_spr,
      left_hip_yaw_vel_idx_wo_spr);
  swing_hip_yaw_traj.SetConstantTraj(VectorXd::Zero(1));
  osc->AddTrackingData(&swing_hip_yaw_traj);
  // Build OSC problem
  osc->ConstructOSC();
  // Connect ports
  builder.Connect(state_receiver->get_output_port(0),
                  osc->get_robot_output_input_port());
  builder.Connect(fsm->get_output_port(0),
                  osc->get_fsm_input_port());
  builder.Connect(lipm_traj_generator->get_output_port(0),
                  osc->get_tracking_data_input_port("lipm_traj"));
  builder.Connect(cp_traj_generator->get_output_port(0),
                  osc->get_tracking_data_input_port("cp_traj"));
  builder.Connect(osc->get_output_port(0),
                  command_sender->get_input_port(0));


  // TODO: add a block for toe angle control


  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  // Assign fixed value to osc constant traj port
  systems::controllers::AssignConstTrajToInputPorts(osc,
      diagram.get(), context.get());

  /// Use the simulator to drive at a fixed rate
  /// If set_publish_every_time_step is true, this publishes twice
  /// Set realtime rate. Otherwise, runs as fast as possible
  auto stepper = std::make_unique<drake::systems::Simulator<double>>(*diagram,
                 std::move(context));
  stepper->set_publish_every_time_step(false);
  stepper->set_publish_at_initialization(false);
  stepper->set_target_realtime_rate(1.0);
  stepper->Initialize();


  drake::log()->info("controller started");

  stepper->StepTo(std::numeric_limits<double>::infinity());
  stepper->StepTo(FLAGS_end_time);

  return 0;
}


}

int main(int argc, char* argv[]) { return dairlib::DoMain(argc, argv); }