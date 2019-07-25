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

#include "examples/Cassie/osc_walk/deviation_from_cp.h"
#include "examples/Cassie/osc_walk/heading_control.h"
#include "systems/controllers/cp_traj_gen.h"
#include "systems/controllers/lipm_traj_gen.h"
#include "systems/controllers/time_based_fsm.h"
#include "systems/controllers/osc/operational_space_control.h"


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
using systems::controllers::ComTrackingData;
using systems::controllers::TransTaskSpaceTrackingData;
using systems::controllers::RotTaskSpaceTrackingData;
using systems::controllers::JointSpaceTrackingData;

// Currently the controller runs at the rate between 500 Hz and 200 Hz, so the
// publish rate of the robot state needs to be less than 500 Hz. Otherwise, the
// performance seems to degrade due to this. (Recommended publish rate: 200 Hz)
// Maybe we need to update the lcm driven loop to clear the queue of lcm message
// if it's more than one message?

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  DiagramBuilder<double> builder;

  drake::lcm::DrakeLcm lcm_local("udpm://239.255.76.67:7667?ttl=0");

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
  auto state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(
                          tree_with_springs);

  // Create command sender.
  auto command_pub = builder.AddSystem(
                       LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
                         channel_u, &lcm_local, 1.0 / 1000.0));
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
  int left_stance_state = 0;
  int right_stance_state = 1;
  double duration_per_state = 0.35;
  double time_shift = 0;
  auto fsm = builder.AddSystem<systems::TimeBasedFiniteStateMachine>(
      tree_with_springs, duration_per_state, time_shift);
  builder.Connect(state_receiver->get_output_port(0),
                  fsm->get_input_port_state());

  // Create CoM trajectory generator
  double desired_com_height = 0.89;
  auto lipm_traj_generator =
      builder.AddSystem<systems::LIPMTrajGenerator>(tree_with_springs,
          desired_com_height,
          duration_per_state,
          left_toe_idx,
          Eigen::VectorXd::Zero(3),
          right_toe_idx,
          Eigen::VectorXd::Zero(3));
  builder.Connect(fsm->get_output_port(0),
                  lipm_traj_generator->get_input_port_fsm());
  builder.Connect(state_receiver->get_output_port(0),
                  lipm_traj_generator->get_input_port_state());

  // Create foot placement control block
  Eigen::Vector2d global_target_position(5, 5);
  Eigen::Vector2d params_of_no_turning(5, 1);
  // Logistic function 1/(1+5*exp(x-1))
  // The function ouputs 0.0007 when x = 0
  //                     0.5    when x = 1
  //                     0.9993 when x = 2
  auto deviation_from_cp =
      builder.AddSystem<cassie::osc_walk::DeviationFromCapturePoint>(
        tree_with_springs, pelvis_idx,
        global_target_position, params_of_no_turning);
  builder.Connect(state_receiver->get_output_port(0),
                  deviation_from_cp->get_input_port_state());

  // Create swing leg trajectory generator (capture point)
  double mid_foot_height = 0.1 + 0.05;
  double desired_final_foot_height = 0.05;//-0.05;
  double desired_final_vertical_foot_velocity = 0;//-1;
  double max_CoM_to_CP_dist = 0.4;
  double cp_offset = 0.06;
  double center_line_offset = 0.06;
  auto cp_traj_generator =
      builder.AddSystem<systems::CPTrajGenerator>(tree_with_springs,
          mid_foot_height,
          desired_final_foot_height,
          desired_final_vertical_foot_velocity,
          max_CoM_to_CP_dist,
          duration_per_state,
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
  builder.Connect(deviation_from_cp->get_output_port(0),
                  cp_traj_generator->get_input_port_fp());

  // Desired Heading Angle
  auto heading_control =
      builder.AddSystem<cassie::osc_walk::HeadingControl>(
        tree_with_springs, pelvis_idx,
        global_target_position, params_of_no_turning);
  builder.Connect(state_receiver->get_output_port(0),
                  heading_control->get_input_port_state());

  // Create Operational space control
  auto osc = builder.AddSystem<systems::controllers::OperationalSpaceControl>(
               tree_with_springs, tree_without_springs, true, false);

  // Cost
  int n_v = tree_without_springs.get_num_velocities();
  MatrixXd Q_accel = 0.00002 * MatrixXd::Identity(n_v, n_v);
  osc->SetAccelerationCostForAllJoints(Q_accel);
  double w_toe = 0.1;  // 1
  osc->AddAccelerationCost("toe_leftdot", w_toe);
  osc->AddAccelerationCost("toe_rightdot", w_toe);
  // Soft constraint
  // w_contact_relax shouldn't be too big, cause we want tracking error to be
  // important
  double w_contact_relax = 200;
  osc->SetWeightOfSoftContactConstraint(w_contact_relax);
  // Firction coefficient
  double mu = 0.8;
  osc->SetContactFriction(mu);
  Vector3d front_contact_disp(-0.0457, 0.112, 0);
  Vector3d rear_contact_disp(0.088, 0, 0);
  osc->AddStateAndContactPoint(left_stance_state,
                               "toe_left", front_contact_disp);
  osc->AddStateAndContactPoint(left_stance_state,
                               "toe_left", rear_contact_disp);
  osc->AddStateAndContactPoint(right_stance_state,
                               "toe_right", front_contact_disp);
  osc->AddStateAndContactPoint(right_stance_state,
                               "toe_right", rear_contact_disp);
  // Swing foot tracking
  MatrixXd W_swing_foot = 200 * MatrixXd::Identity(3, 3);
  MatrixXd K_p_sw_ft = 100 * MatrixXd::Identity(3, 3);
  MatrixXd K_d_sw_ft = 10 * MatrixXd::Identity(3, 3);
  TransTaskSpaceTrackingData swing_foot_traj("cp_traj", 3,
      K_p_sw_ft, K_d_sw_ft, W_swing_foot,
      &tree_with_springs, &tree_without_springs);
  swing_foot_traj.AddStateAndPointToTrack(left_stance_state, "toe_right");
  swing_foot_traj.AddStateAndPointToTrack(right_stance_state, "toe_left");
  osc->AddTrackingData(&swing_foot_traj);
  // Center of mass tracking
  MatrixXd W_com = MatrixXd::Identity(3, 3);
  W_com(0, 0) = 2;
  W_com(1, 1) = 2;
  W_com(2, 2) = 2000;
  MatrixXd K_p_com = 50 * MatrixXd::Identity(3, 3);
  MatrixXd K_d_com = 10 * MatrixXd::Identity(3, 3);
  ComTrackingData center_of_mass_traj("lipm_traj", 3,
      K_p_com, K_d_com, W_com,
      &tree_with_springs, &tree_without_springs);
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
  RotTaskSpaceTrackingData pelvis_balance_traj("pelvis_balance_traj", 3,
      K_p_pelvis_balance, K_d_pelvis_balance, W_pelvis_balance,
      &tree_with_springs, &tree_without_springs);
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
  RotTaskSpaceTrackingData pelvis_heading_traj("pelvis_heading_traj", 3,
      K_p_pelvis_heading, K_d_pelvis_heading, W_pelvis_heading,
      &tree_with_springs, &tree_without_springs);
  pelvis_heading_traj.AddFrameToTrack("pelvis");
  osc->AddTrackingData(&pelvis_heading_traj, 0.05);
  // Swing toe joint tracking (Currently use fix position)
  MatrixXd W_swing_toe = 2 * MatrixXd::Identity(1, 1);
  MatrixXd K_p_swing_toe = 1000 * MatrixXd::Identity(1, 1);
  MatrixXd K_d_swing_toe = 100 * MatrixXd::Identity(1, 1);
  JointSpaceTrackingData swing_toe_traj("swing_toe_traj",
      K_p_swing_toe, K_d_swing_toe, W_swing_toe,
      &tree_with_springs, &tree_without_springs);
  swing_toe_traj.AddStateAndJointToTrack(left_stance_state,
                                         "toe_right", "toe_rightdot");
  swing_toe_traj.AddStateAndJointToTrack(right_stance_state,
                                         "toe_left", "toe_leftdot");
  osc->AddConstTrackingData(&swing_toe_traj, -1.5 * VectorXd::Ones(1));
  // Swing hip yaw joint tracking
  MatrixXd W_hip_yaw = 20 * MatrixXd::Identity(1, 1);
  MatrixXd K_p_hip_yaw = 200 * MatrixXd::Identity(1, 1);
  MatrixXd K_d_hip_yaw = 160 * MatrixXd::Identity(1, 1);
  JointSpaceTrackingData swing_hip_yaw_traj("swing_hip_yaw_traj",
      K_p_hip_yaw, K_d_hip_yaw, W_hip_yaw,
      &tree_with_springs, &tree_without_springs);
  swing_hip_yaw_traj.AddStateAndJointToTrack(left_stance_state,
      "hip_yaw_right", "hip_yaw_rightdot");
  swing_hip_yaw_traj.AddStateAndJointToTrack(right_stance_state,
      "hip_yaw_left", "hip_yaw_leftdot");
  osc->AddConstTrackingData(&swing_hip_yaw_traj, VectorXd::Zero(1));
  // Build OSC problem
  osc->BuildOSC();
  // Connect ports
  builder.Connect(state_receiver->get_output_port(0),
                  osc->get_robot_output_input_port());
  builder.Connect(fsm->get_output_port(0),
                  osc->get_fsm_input_port());
  builder.Connect(lipm_traj_generator->get_output_port(0),
                  osc->get_tracking_data_input_port("lipm_traj"));
  builder.Connect(cp_traj_generator->get_output_port(0),
                  osc->get_tracking_data_input_port("cp_traj"));
  builder.Connect(heading_control->get_output_port(0),
                  osc->get_tracking_data_input_port("pelvis_balance_traj"));
  builder.Connect(heading_control->get_output_port(0),
                  osc->get_tracking_data_input_port("pelvis_heading_traj"));
  builder.Connect(osc->get_output_port(0),
                  command_sender->get_input_port(0));

  // Create the diagram and context
  auto owned_diagram = builder.Build();
  auto context = owned_diagram->CreateDefaultContext();

  // Create the simulator
  const auto& diagram = *owned_diagram;
  drake::systems::Simulator<double> simulator(std::move(owned_diagram),
      std::move(context));
  auto& diagram_context = simulator.get_mutable_context();

  auto& state_receiver_context =
      diagram.GetMutableSubsystemContext(*state_receiver, &diagram_context);

  // Wait for the first message.
  drake::log()->info("Waiting for first lcmt_robot_output");
  drake::lcm::Subscriber<dairlib::lcmt_robot_output> input_sub(&lcm_local,
      "CASSIE_STATE");
  LcmHandleSubscriptionsUntil(&lcm_local, [&]() {
    return input_sub.count() > 0;
  });

  // Initialize the context based on the first message.
  const double t0 = input_sub.message().utime * 1e-6;
  diagram_context.SetTime(t0);
  auto& input_value = state_receiver->get_input_port(0).FixValue(
                        &state_receiver_context, input_sub.message());

  drake::log()->info("controller started");
  while (true) {
    // Wait for an lcmt_robot_output message.
    input_sub.clear();
    LcmHandleSubscriptionsUntil(&lcm_local, [&]() {
      return input_sub.count() > 0;
    });
    // Write the lcmt_robot_output message into the context and advance.
    input_value.GetMutableData()->set_value(input_sub.message());
    const double time = input_sub.message().utime * 1e-6;

    // Check if we are very far ahead or behind
    // (likely due to a restart of the driving clock)
    if (time > simulator.get_context().get_time() + 1.0 ||
        time < simulator.get_context().get_time() - 1.0) {
      std::cout << "Controller time is " << simulator.get_context().get_time()
          << ", but stepping to " << time << std::endl;
      std::cout << "Difference is too large, resetting controller time." <<
          std::endl;
      simulator.get_mutable_context().SetTime(time);
    }

    simulator.AdvanceTo(time);
    // Force-publish via the diagram
    diagram.Publish(diagram_context);
  }

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::DoMain(argc, argv); }
