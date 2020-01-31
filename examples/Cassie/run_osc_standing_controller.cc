#include <gflags/gflags.h>
#include "attic/multibody/rigidbody_utils.h"
#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "examples/Cassie/cassie_utils.h"
#include "systems/controllers/osc/operational_space_control.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/robot_lcm_systems.h"

#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"

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

DEFINE_string(channel_x, "CASSIE_STATE",
              "The name of the channel which receives state");
DEFINE_string(channel_u, "CASSIE_INPUT",
              "The name of the channel which publishes command");

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

  // Create state receiver.
  auto state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(
                          tree_with_springs);

  // Create command sender.
  auto command_pub = builder.AddSystem(
                       LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
                           FLAGS_channel_u, &lcm_local, 1.0 / 1000.0));
  auto command_sender = builder.AddSystem<systems::RobotCommandSender>(
                          tree_with_springs);

  builder.Connect(command_sender->get_output_port(0),
                  command_pub->get_input_port());

  // Get body indices for cassie with springs
  int pelvis_idx = GetBodyIndexFromName(tree_with_springs, "pelvis");
  DRAKE_DEMAND(pelvis_idx != -1);

  // Create Operational space control
  auto osc = builder.AddSystem<systems::controllers::OperationalSpaceControl>(
               tree_with_springs, tree_without_springs, false, false);

  // Cost
  // cout << "Adding cost\n";
  int n_v = tree_without_springs.get_num_velocities();
  MatrixXd Q_accel = 0.00002 * MatrixXd::Identity(n_v, n_v);
  osc->SetAccelerationCostForAllJoints(Q_accel);
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
  osc->AddContactPoint("toe_left", front_contact_disp);
  osc->AddContactPoint("toe_left", rear_contact_disp);
  osc->AddContactPoint("toe_right", front_contact_disp);
  osc->AddContactPoint("toe_right", rear_contact_disp);
  // Center of mass tracking
  // cout << "Adding center of mass tracking\n";
  Vector3d desired_com(0, 0, 0.89);
  MatrixXd W_com = MatrixXd::Identity(3, 3);
  W_com(0, 0) = 200;//2
  W_com(1, 1) = 200;//2
  W_com(2, 2) = 2000;//2000
  MatrixXd K_p_com = 50 * MatrixXd::Identity(3, 3);
  MatrixXd K_d_com = 10 * MatrixXd::Identity(3, 3);
  ComTrackingData center_of_mass_traj("lipm_traj", 3,
      K_p_com, K_d_com, W_com,
      &tree_with_springs, &tree_without_springs);
  osc->AddConstTrackingData(&center_of_mass_traj, desired_com);
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
      K_p_pelvis, K_d_pelvis, W_pelvis,
      &tree_with_springs, &tree_without_springs);
  pelvis_rot_traj.AddFrameToTrack("pelvis");
  VectorXd pelvis_desired_quat(4);
  pelvis_desired_quat << 1, 0, 0, 0;
  osc->AddConstTrackingData(&pelvis_rot_traj, pelvis_desired_quat);
  // Build OSC problem
  osc->Build();
  // Connect ports
  builder.Connect(state_receiver->get_output_port(0),
                  osc->get_robot_output_input_port());
  builder.Connect(osc->get_output_port(0),
                  command_sender->get_input_port(0));

  // Create the diagram
  auto owned_diagram = builder.Build();
  owned_diagram->set_name(("osc standing controller"));

  // Run lcm-driven simulation
  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop
      (&lcm_local,
       std::move(owned_diagram),
       state_receiver,
       FLAGS_channel_x,
       false);
  loop.Simulate();

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::DoMain(argc, argv); }
