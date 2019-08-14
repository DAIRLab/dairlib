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
#include "attic/multibody/rigidbody_utils.h"
#include "examples/Cassie/cassie_utils.h"
#include "systems/robot_lcm_systems.h"
#include "systems/controllers/osc/operational_space_control.h"
#include "examples/Cassie/osc/standing_com_traj.h"

DEFINE_double(height, .89, "The desired height (m)");

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
using drake::systems::TriggerType;

// Currently the controller runs at the rate between 500 Hz and 200 Hz, so the
// publish rate of the robot state needs to be less than 200 Hz. Otherwise, the
// performance seems to degrade due to this. (Recommended publish rate: 200 Hz)
// Maybe we need to update the lcm driven loop to clear the queue of lcm message
// if it's more than one message?

DEFINE_double(publish_rate, 200, "Publishing frequency (Hz)");

DEFINE_string(channel, "CASSIE_STATE_SIMULATION",
    "LCM channel for receiving state. "
    "Use CASSIE_STATE_SIMULATION to get state from simulator, and "
    "use CASSIE_STATE_DISPATCHER to get state from state estimator");

DEFINE_double(cost_weight_multiplier, 0.001,
    "A cosntant times with cost weight of OSC traj tracking");

DEFINE_double(end_time, std::numeric_limits<double>::infinity(),
    "Stop time of the controller");

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

  const std::string channel_x = FLAGS_channel;
  const std::string channel_u = "CASSIE_INPUT";

  // Create state receiver.
  auto state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(
                          tree_with_springs);

  // Create command sender.
  auto command_pub = builder.AddSystem(
                       LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
                         channel_u, lcm, {TriggerType::kForced}));
  auto command_sender = builder.AddSystem<systems::RobotCommandSender>(
                          tree_with_springs);

  builder.Connect(command_sender->get_output_port(0),
                  command_pub->get_input_port());

  // Get body indices for cassie with springs
  int pelvis_idx = GetBodyIndexFromName(tree_with_springs, "pelvis");
  int left_toe_idx = GetBodyIndexFromName(tree_with_springs, "toe_left");
  int right_toe_idx = GetBodyIndexFromName(tree_with_springs, "toe_right");
  DRAKE_DEMAND(pelvis_idx != -1 && left_toe_idx != -1 && right_toe_idx != -1);

  // Create desired center of mass traj
  auto com_traj_generator =
      builder.AddSystem<cassie::osc::StandingComTraj>(
          tree_with_springs, pelvis_idx, left_toe_idx, right_toe_idx,
          FLAGS_height);
  builder.Connect(state_receiver->get_output_port(0),
                  com_traj_generator->get_input_port_state());

  // Create Operational space control
  auto osc = builder.AddSystem<systems::controllers::OperationalSpaceControl>(
               tree_with_springs, tree_without_springs, false, false);

  // Cost
  // cout << "Adding cost\n";
  int n_v = tree_without_springs.get_num_velocities();
  MatrixXd Q_accel = 10 * MatrixXd::Identity(n_v, n_v);
  osc->SetAccelerationCostForAllJoints(Q_accel);
  // Soft constraint
  // cout << "Adding constraint\n";
  // We don't want this to be too big, cause we want tracking error to be important
  double w_contact_relax = 20000;
  osc->SetWeightOfSoftContactConstraint(w_contact_relax);
  // Firction coefficient
  double mu = 0.8;
  osc->SetContactFriction(mu);
  Vector3d front_contact_disp(-0.0457, 0.112, 0);
  Vector3d rear_contact_disp(0.088, 0, 0);
  osc->AddContactPoint("toe_left", front_contact_disp);
  osc->AddContactPoint("toe_left", rear_contact_disp);
  osc->AddContactPoint("toe_right", front_contact_disp);
  osc->AddContactPoint("toe_right", rear_contact_disp);

  // Center of mass tracking
  // Weighting x-y higher than z, as they are more important to balancing
  MatrixXd W_com = MatrixXd::Identity(3, 3);
  W_com(0, 0) = 2000;
  W_com(1, 1) = 2000;
  W_com(2, 2) = 200;

  // Set xy PD gains so they do not effect  passive LIPM dynamics at capture
  // point, when x = sqrt(l/g) * xdot
  // Passive dynamics: xddot = g/l * x
  //
  // -Kp * x - Kd * xdot =
  // -Kp * x + Kd * sqrt(g/l) * x = g/l * x
  // Kp = sqrt(g/l) * Kd - g/l
  double xy_scale = 10;
  double g_over_l = 9.81/FLAGS_height;
  MatrixXd K_p_com = (xy_scale*sqrt(g_over_l)  - g_over_l) *
      MatrixXd::Identity(3, 3);
  MatrixXd K_d_com = xy_scale * MatrixXd::Identity(3, 3);

  K_p_com(2, 2) = 10;
  K_d_com(2, 2) = 10;

  ComTrackingData center_of_mass_traj("com_traj", 3,
      K_p_com, K_d_com, W_com * FLAGS_cost_weight_multiplier,
      &tree_with_springs, &tree_without_springs);
  osc->AddTrackingData(&center_of_mass_traj);
  // Pelvis rotation tracking
  // cout << "Adding pelvis rotation tracking\n";
  double w_pelvis_balance = 200;
  double w_heading = 200;
  double k_p_pelvis_balance = 10;
  double k_d_pelvis_balance = 10;
  double k_p_heading = 10;
  double k_d_heading = 10;
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
      K_p_pelvis, K_d_pelvis, W_pelvis * FLAGS_cost_weight_multiplier,
      &tree_with_springs, &tree_without_springs);
  pelvis_rot_traj.AddFrameToTrack("pelvis");
  VectorXd pelvis_desired_quat(4);
  pelvis_desired_quat << 1, 0, 0, 0;
  osc->AddConstTrackingData(&pelvis_rot_traj, pelvis_desired_quat);
  // // Left hip yaw joint tracking
  // MatrixXd W_hip_yaw = 20 * MatrixXd::Identity(1, 1);
  // MatrixXd K_p_hip_yaw = 200 * MatrixXd::Identity(1, 1);
  // MatrixXd K_d_hip_yaw = 160 * MatrixXd::Identity(1, 1);
  // JointSpaceTrackingData left_hip_yaw_traj("left_hip_yaw_traj",
  //     K_p_hip_yaw, K_d_hip_yaw, W_hip_yaw * FLAGS_cost_weight_multiplier,
  //     &tree_with_springs, &tree_without_springs);
  // left_hip_yaw_traj.AddJointToTrack("hip_yaw_left", "hip_yaw_leftdot");
  // osc->AddConstTrackingData(&left_hip_yaw_traj, VectorXd::Zero(1));
  // // right hip yaw joint tracking
  // JointSpaceTrackingData right_hip_yaw_traj("right_hip_yaw_traj",
  //     K_p_hip_yaw, K_d_hip_yaw, W_hip_yaw * FLAGS_cost_weight_multiplier,
  //     &tree_with_springs, &tree_without_springs);
  // right_hip_yaw_traj.AddJointToTrack("hip_yaw_right", "hip_yaw_rightdot");
  // osc->AddConstTrackingData(&right_hip_yaw_traj, VectorXd::Zero(1));
  // Build OSC problem
  osc->Build();
  // Connect ports
  builder.Connect(state_receiver->get_output_port(0),
                  osc->get_robot_output_input_port());
  builder.Connect(osc->get_output_port(0),
                  command_sender->get_input_port(0));
  builder.Connect(com_traj_generator->get_output_port(0),
                  osc->get_tracking_data_input_port("com_traj"));

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
  drake::log()->info("Waiting for first state message on " + channel_x);
  drake::lcm::Subscriber<dairlib::lcmt_robot_output> state_sub(lcm,
      channel_x);
  LcmHandleSubscriptionsUntil(lcm, [&]() {
    return state_sub.count() > 0;
  });

  // Initialize the context based on the first message.
  const double t0 = state_sub.message().utime * 1e-6;
  diagram_context.SetTime(t0);
  auto& input_value = state_receiver->get_input_port(0).FixValue(
                        &state_receiver_context, state_sub.message());

  drake::log()->info("controller started");
  while (true) {
    // Wait for an lcmt_robot_output message.
    state_sub.clear();
    LcmHandleSubscriptionsUntil(lcm, [&]() {
      return state_sub.count() > 0;
    });
    // Write the lcmt_robot_output message into the context and advance.
    input_value.GetMutableData()->set_value(state_sub.message());
    const double time = state_sub.message().utime * 1e-6;

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
