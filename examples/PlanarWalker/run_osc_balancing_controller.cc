#include <math.h>

#include <gflags/gflags.h>

#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/primitives/discrete_time_delay.h"

#include "attic/multibody/rigidbody_utils.h"
#include "common/find_resource.h"
#include "dairlib/lcmt_pd_config.hpp"
#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "systems/robot_lcm_systems.h"

#include "examples/PlanarWalker/safe_traj_gen.h"
#include "examples/PlanarWalker/state_based_fsm.h"
#include "systems/controllers/osc/operational_space_control.h"

namespace dairlib {

using std::cout;
using std::endl;

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using drake::systems::DiagramBuilder;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;

using multibody::GetBodyIndexFromName;
using systems::controllers::ComTrackingData;
using systems::controllers::JointSpaceTrackingData;
using systems::controllers::RotTaskSpaceTrackingData;
using systems::controllers::TransTaskSpaceTrackingData;

// TODO: See if this will be an issue for safe_traj_gen
// Currently the controller runs at the rate between 500 Hz and 200 Hz, so the
// publish rate of the robot state needs to be less than 500 Hz. Otherwise, the
// performance seems to degrade due to this. (Recommended publish rate: 200 Hz)
// Maybe we need to update the lcm driven loop to clear the queue of lcm message
// if it's more than one message?

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  DiagramBuilder<double> builder;

  drake::lcm::DrakeLcm lcm_local("udpm://239.255.76.67:7667?ttl=0");

  RigidBodyTree<double> tree;
  drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      "examples/PlanarWalker/PlanarWalkerWithTorsoAndFeet.urdf",
      drake::multibody::joints::kFixed, &tree);

  const double terrain_size = 100;
  const double terrain_depth = 0.20;
  drake::multibody::AddFlatTerrainToWorld(&tree, terrain_size, terrain_depth);

  // Create state receiver.
  auto state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(tree);

  // Create command sender.
  auto command_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          "PLANAR_INPUT", &lcm_local, 1.0 / 1000.0));
  auto command_sender = builder.AddSystem<systems::RobotCommandSender>(tree);

  builder.Connect(command_sender->get_output_port(0),
                  command_pub->get_input_port());

  int torso_idx = GetBodyIndexFromName(tree, "torso_mass");
  int left_foot_idx = GetBodyIndexFromName(tree, "left_foot");
  int right_foot_idx = GetBodyIndexFromName(tree, "right_foot");
  DRAKE_DEMAND(torso_idx != -1 && left_foot_idx != -1 && right_foot_idx != -1);

  Vector3d pt_on_left_foot = Vector3d::Zero();
  Vector3d pt_on_right_foot = Vector3d::Zero();
  int left_stance_state = 0;
  int right_stance_state = 1;

  // FiniteStateMachine
  auto fsm = builder.AddSystem<StateBasedFiniteStateMachine>(
      tree, left_foot_idx, pt_on_left_foot, right_foot_idx, pt_on_right_foot);
  builder.Connect(state_receiver->get_output_port(0), fsm->get_input_port(0));

  // create CoM trajectory generator
  LIPMSwingLeg<double> lipm_model(9.81, 0.75, 5.5, 0.05);
  LoadLyapunovPolynomial polynomial_loader("examples/PlanarWalker/csv/V_M.csv",
                                           "examples/PlanarWalker/csv/V_p.csv");

  double mid_foot_height = 0.05;
  double desired_final_foot_height = 0.005; // 0.05
  double desired_final_vertical_foot_velocity = 0;
  auto safe_traj_generator = builder.AddSystem<SafeTrajGenerator>(
      tree, lipm_model, polynomial_loader, left_foot_idx, pt_on_left_foot,
      right_foot_idx, pt_on_right_foot, mid_foot_height,
      desired_final_foot_height, desired_final_vertical_foot_velocity, false);

  // auto discrete_time_delay =
  //   builder.AddSystem<drake::systems::DiscreteTimeDelay>(0.01, 1, 1);
  // builder.Connect(fsm->get_output_port(0),
  //                 discrete_time_delay->get_input_port());
  // builder.Connect(discrete_time_delay->get_output_port(),
  //                 safe_traj_generator->get_input_port_fsm());
  builder.Connect(fsm->get_output_port(0),
                  safe_traj_generator->get_input_port_fsm());

  // builder.Connect(fsm->get_output_port(0),
  //                 safe_traj_generator->get_input_port_fsm());
  builder.Connect(state_receiver->get_output_port(0),
                  safe_traj_generator->get_input_port_state());

  // Create Operational space control
  auto osc = builder.AddSystem<systems::controllers::OperationalSpaceControl>(
      tree, tree, true, false);
  // Cost
  // int n_v = tree.get_num_velocities();
  // MatrixXd Q_accel = 0.00002 * MatrixXd::Identity(n_v, n_v);
  // osc->SetAccelerationCostForAllJoints(Q_accel);

  // double w_toe = 0.1;
  // osc->AddAccelerationCost("left_ankledot", w_toe);
  // osc->AddAccelerationCost("right_ankledot", w_toe);

  // Soft constraint
  // w_contact_relax shouldn't be too big, cause we want tracking error to be
  // important

  // Set cost to a positive number these for soft constraint cost on the stance
  // foot
  double w_contact_relax = 200;
  osc->SetWeightOfSoftContactConstraint(w_contact_relax);

  // Firction coefficient
  double mu = 0.7;
  osc->SetContactFriction(mu);
  Vector3d front_contact_disp(0.075, 0, 0);
  Vector3d rear_contact_disp(-0.075, 0, 0);
  osc->AddStateAndContactPoint(left_stance_state, "left_foot",
                               front_contact_disp);
  osc->AddStateAndContactPoint(left_stance_state, "left_foot",
                               rear_contact_disp);
  osc->AddStateAndContactPoint(right_stance_state, "right_foot",
                               front_contact_disp);
  osc->AddStateAndContactPoint(right_stance_state, "right_foot",
                               rear_contact_disp);

  // Swing foot tracking
  MatrixXd W_swing_foot = 200 * MatrixXd::Identity(3, 3);
  MatrixXd K_p_sw_ft = 250.0 * MatrixXd::Identity(3, 3);
  K_p_sw_ft(0, 0) = 500.0;
  // K_p_sw_ft(2, 2) = 150;
  MatrixXd K_d_sw_ft = 10.0 * MatrixXd::Identity(3, 3);
  K_d_sw_ft(0, 0) = 50;
  // K_d_sw_ft(2, 2) = 0.05;
  TransTaskSpaceTrackingData swing_foot_traj(
      "swing_traj", 3, K_p_sw_ft, K_d_sw_ft, W_swing_foot, &tree, &tree);
  swing_foot_traj.AddStateAndPointToTrack(left_stance_state, "right_foot");
  swing_foot_traj.AddStateAndPointToTrack(right_stance_state, "left_foot");
  osc->AddTrackingData(&swing_foot_traj);

  // MatrixXd W_swing_foot_toe = 20 * MatrixXd::Identity(3, 3);
  // MatrixXd K_p_sw_ft_toe = 25.0 * MatrixXd::Identity(3, 3);
  // K_p_sw_ft(0, 0) = 50.0;
  // // K_p_sw_ft(2, 2) = 150;
  // MatrixXd K_d_sw_ft_toe = 1.0 * MatrixXd::Identity(3, 3);
  // K_d_sw_ft(0, 0) = 5;
  // // K_d_sw_ft(2, 2) = 0.05;
  // TransTaskSpaceTrackingData swing_foot_toe_traj(
  //     "swing_toe_traj", 3, K_p_sw_ft_toe, K_d_sw_ft_toe, W_swing_foot_toe,
  //     &tree, &tree);
  // Vector3d toe_displacement = Vector3d::Zero();
  // toe_displacement(0)  = 0.075;
  // swing_foot_toe_traj.AddStateAndPointToTrack(left_stance_state, "right_foot",
  //                                             toe_displacement);
  // swing_foot_toe_traj.AddStateAndPointToTrack(right_stance_state, "left_foot",
  //                                             toe_displacement);
  // osc->AddTrackingData(&swing_foot_toe_traj);

  // Center of mass tracking
  MatrixXd W_com = MatrixXd::Identity(3, 3);
  W_com(0, 0) = 200.0;
  W_com(1, 1) = 0.0;
  W_com(2, 2) = 200.0; // 200
  MatrixXd K_p_com = 100.0 * Matrix3d::Zero();
  K_p_com(2, 2) = 500.0;
  MatrixXd K_d_com = 10.0 * Matrix3d::Zero();
  K_d_com(2, 2) = 50.0;
  ComTrackingData center_of_mass_traj("com_traj", 3, K_p_com, K_d_com, W_com,
                                      &tree, &tree);
  osc->AddTrackingData(&center_of_mass_traj);

  // Swing toe joint tracking (Currently use fixed position)
  MatrixXd W_swing_toe = 20.0 * MatrixXd::Identity(1, 1);
  MatrixXd K_p_swing_toe = 100.0 * MatrixXd::Identity(1, 1);
  MatrixXd K_d_swing_toe = 10.0 * MatrixXd::Identity(1, 1);
  JointSpaceTrackingData swing_toe_traj("swing_toe_traj", K_p_swing_toe,
                                        K_d_swing_toe, W_swing_toe, &tree,
                                        &tree);
  swing_toe_traj.AddStateAndJointToTrack(left_stance_state, "right_ankle",
                                         "right_ankledot");
  swing_toe_traj.AddStateAndJointToTrack(right_stance_state, "left_ankle",
                                         "left_ankledot");
  osc->AddConstTrackingData(&swing_toe_traj, -0.58234* VectorXd::Ones(1));

  // Torso tracking
  // MatrixXd W_torso = 2 * MatrixXd::Identity(1, 1);
  // MatrixXd K_p_torso = 100 * MatrixXd::Identity(1, 1);
  // MatrixXd K_d_torso = 10 * MatrixXd::Identity(1, 1);
  // JointSpaceTrackingData torso_traj("torso_traj", K_p_torso, K_d_torso, W_torso,
  //                                   &tree, &tree);
  // torso_traj.AddJointToTrack("left_hip_pin", "left_hip_pindot");
  // osc->AddConstTrackingData(&torso_traj, 0.58234* VectorXd::Ones(1));
  // torso_traj.AddJointToTrack("right_hip_pin", "right_hip_pindot");
  // osc->AddConstTrackingData(&torso_traj, 0.58234* VectorXd::Ones(1));

  // Build OSC problem
  osc->Build();
  // Connect ports
  builder.Connect(state_receiver->get_output_port(0),
                  osc->get_robot_output_input_port());


  builder.Connect(fsm->get_output_port(0), osc->get_fsm_input_port());

  builder.Connect(safe_traj_generator->get_output_port(0),
                  osc->get_tracking_data_input_port("com_traj"));
  builder.Connect(safe_traj_generator->get_output_port(1),
                  osc->get_tracking_data_input_port("swing_traj"));
  builder.Connect(osc->get_output_port(0), command_sender->get_input_port(0));

  builder.Connect(safe_traj_generator->get_output_port(1),
                  fsm->get_input_port(1));
  // Create the diagram and context
  auto owned_diagram = builder.Build();
  auto context = owned_diagram->CreateDefaultContext();

  // std::cout << "HasAnydirectFeedThrough ?" << std::endl;
  // std::cout << fsm->HasAnyDirectFeedthrough() << std::endl;

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
                                                               "PLANAR_STATE");
  LcmHandleSubscriptionsUntil(&lcm_local,
                              [&]() { return input_sub.count() > 0; });

  // Initialize the context based on the first message.
  const double t0 = input_sub.message().utime * 1e-6;
  diagram_context.SetTime(t0);
  auto& input_value = state_receiver->get_input_port(0).FixValue(
      &state_receiver_context, input_sub.message());

  drake::log()->info("controller started");
  while (true) {
    // Wait for an lcmt_robot_output message.
    input_sub.clear();
    LcmHandleSubscriptionsUntil(&lcm_local,
                                [&]() { return input_sub.count() > 0; });
    // Write the lcmt_robot_output message into the context and advance.
    input_value.GetMutableData()->set_value(input_sub.message());
    const double time = input_sub.message().utime * 1e-6;

    // Check if we are very far ahead or behind
    // (likely due to a restart of the driving clock)
    if (time > simulator.get_context().get_time() + 1.0 ||
        time < simulator.get_context().get_time() - 1.0) {
      std::cout << "Controller time is " << simulator.get_context().get_time()
                << ", but stepping to " << time << std::endl;
      std::cout << "Difference is too large, resetting controller time."
                << std::endl;
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
