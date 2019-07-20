#include <math.h>

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

int DoMain() {
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
  DRAKE_DEMAND(pelvis_idx != -1);

  double time_shift = 0;

  // Create Operational space control
  auto osc = builder.AddSystem<systems::controllers::OperationalSpaceControl>(
               &tree_with_springs, &tree_without_springs, false, true);
  // Get body index
  int pelvis_idx_w_spr = pelvis_idx;
  int pelvis_idx_wo_spr = GetBodyIndexFromName(
                            tree_without_springs, "pelvis");
  int left_toe_idx_wo_spr = GetBodyIndexFromName(
                              tree_without_springs, "toe_left");
  int right_toe_idx_wo_spr = GetBodyIndexFromName(
                               tree_without_springs, "toe_right");

  int n_v = tree_without_springs.get_num_velocities();
  // Cost
  // cout << "Adding cost\n";
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
  osc->AddContactPoint(left_toe_idx_wo_spr, front_contact_disp);
  osc->AddContactPoint(left_toe_idx_wo_spr, rear_contact_disp);
  osc->AddContactPoint(right_toe_idx_wo_spr, front_contact_disp);
  osc->AddContactPoint(right_toe_idx_wo_spr, rear_contact_disp);
  // Center of mass tracking
  // cout << "Adding center of mass tracking\n";
  Vector3d desired_com(0, 0, 0.89);
  MatrixXd W_com = MatrixXd::Identity(3, 3);
  W_com(0, 0) = 2;
  W_com(1, 1) = 2;
  W_com(2, 2) = 2000;
  MatrixXd K_p_com = 50 * MatrixXd::Identity(3, 3);
  MatrixXd K_d_com = 10 * MatrixXd::Identity(3, 3);
  TransTaskSpaceTrackingData center_of_mass_traj("lipm_traj", 3,
      K_p_com, K_d_com, W_com, true, false, true);
  center_of_mass_traj.SetConstantTraj(desired_com);
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
  VectorXd pelvis_desired_quat(4);
  pelvis_desired_quat << 1, 0, 0, 0;
  pelvis_rot_traj.SetConstantTraj(pelvis_desired_quat);
  osc->AddTrackingData(&pelvis_rot_traj);
  // Build OSC problem
  osc->ConstructOSC();
  // Connect ports
  builder.Connect(state_receiver->get_output_port(0),
                  osc->get_robot_output_input_port());
  builder.Connect(osc->get_output_port(0),
                  command_sender->get_input_port(0));

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

  return 0;
}


}

int main() { return dairlib::DoMain(); }
