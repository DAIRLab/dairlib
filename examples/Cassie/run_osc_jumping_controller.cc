#include <drake/lcmt_contact_results_for_viz.hpp>
#include <drake/multibody/parsing/parser.h>
#include <drake/systems/lcm/lcm_interface_system.h>
#include <gflags/gflags.h>
#include "attic/multibody/rigidbody_utils.h"
#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "examples/Cassie/cassie_utils.h"
#include "examples/Cassie/simulator_drift.h"
#include "systems/controllers/osc/operational_space_control.h"
#include "systems/controllers/time_based_fsm.h"
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
using std::map;
using std::string;
using std::vector;

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using drake::geometry::SceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::DiagramBuilder;
using drake::systems::TriggerType;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::lcm::TriggerTypeSet;

using multibody::GetBodyIndexFromName;
using systems::controllers::ComTrackingData;
using systems::controllers::JointSpaceTrackingData;
using systems::controllers::RotTaskSpaceTrackingData;
using systems::controllers::TransTaskSpaceTrackingData;

DEFINE_double(drift_rate, 0.0, "Drift rate for floating-base state");

DEFINE_string(channel_x, "CASSIE_STATE",
              "The name of the channel which receives state");
DEFINE_string(channel_u, "CASSIE_INPUT",
              "The name of the channel which publishes command");
DEFINE_double(balance_height, 1.125,
              "Balancing height for Cassie before attempting the jump");

DEFINE_bool(print_osc, false, "whether to print the osc debug message or not");
DEFINE_bool(is_two_phase, false,
            "true: only right/left single support"
            "false: both double and single support");

// Currently the controller runs at the rate between 500 Hz and 200 Hz, so the
// publish rate of the robot state needs to be less than 500 Hz. Otherwise, the
// performance seems to degrade due to this. (Recommended publish rate: 200 Hz)
// Maybe we need to update the lcm driven loop to clear the queue of lcm message
// if it's more than one message?

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Drake system initialization stuff
  drake::systems::DiagramBuilder<double> builder;
  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");
  MultibodyPlant<double> plant_with_springs(0.0);
  MultibodyPlant<double> plant_without_springs(0.0);
  Parser parser_with_springs(&plant_with_springs, &scene_graph);
  Parser parser_without_springs(&plant_without_springs, &scene_graph);

  parser_with_springs.AddModelFromFile(
      FindResourceOrThrow("examples/Cassie/urdf/cassie_v2.urdf"));
  parser_without_springs.AddModelFromFile(
      FindResourceOrThrow("examples/Cassie/urdf/cassie_fixed_springs.urdf"));
  plant_with_springs.mutable_gravity_field().set_gravity_vector(
      -9.81 * Eigen::Vector3d::UnitZ());
  plant_without_springs.mutable_gravity_field().set_gravity_vector(
      -9.81 * Eigen::Vector3d::UnitZ());
  plant_with_springs.Finalize();
  plant_without_springs.Finalize();

  int n_q = plant_without_springs.num_positions();
  int n_v = plant_without_springs.num_velocities();
  int n_x = n_q + n_v;

  std::cout << "nq: " << n_q << " n_v: " << n_v << " n_x: " << n_x << std::endl;
  // Create maps for joints
  map<string, int> pos_map =
      multibody::makeNameToPositionsMap(plant_without_springs);
  map<string, int> vel_map =
      multibody::makeNameToVelocitiesMap(plant_without_springs);
  map<string, int> act_map =
      multibody::makeNameToActuatorsMap(plant_without_springs);

  // Cassie parameters
  Vector3d front_contact_disp(-0.0457, 0.112, 0);
  Vector3d rear_contact_disp(0.088, 0, 0);
  Vector3d mid_contact_disp = (front_contact_disp + rear_contact_disp) / 2;

  // Get body indices for cassie with springs
  auto pelvis_idx = plant_with_springs.GetBodyByName("pelvis").index();
  auto l_toe_idx = plant_with_springs.GetBodyByName("toe_left").index();
  auto r_toe_idx = plant_with_springs.GetBodyByName("toe_right").index();

  // Note that we didn't add drift to yaw angle here because it requires
  // changing SimulatorDrift.



  auto lcm = builder.AddSystem < drake::systems::lcm::LcmInterfaceSystem();
  auto contact_results_sub = builder.AddSystem(
      LcmSubscriberSystem::Make<drake::lcmt_contact_results_for_viz>(
          "CONTACT_RESULTS", lcm));
  auto state_sub = builder.AddSystem(
      LcmSubscriberSystem::Make<lcmt_robot_output>(FLAGS_channel_x, lcm));
  auto state_receiver =
      builder.AddSystem<systems::RobotOutputReceiver>(plant_without_springs);
  // Create Operational space control
  auto com_traj_generator = builder.AddSystem<CoMTraj>(
      plant_with_springs, pelvis_idx, l_toe_idx, r_toe_idx, center_of_mass_traj,
      FLAGS_balance_height);
  auto l_foot_traj_generator = builder.AddSystem<FlightFootTraj>(
      plant_with_springs, pelvis_idx, l_toe_idx);
  auto r_foot_traj_generator = builder.AddSystem<FlightFootTraj>(
      plant_with_springs, pelvis_idx, r_toe_idx);
  auto pelvis_orientation_traj_generator = builder.AddSystem<TorsoTraj>(
      plant_with_springs, pelvis_orientation_traj);
  auto fsm = builder.AddSystem<dairlib::examples::JumpingFintieStateMachine>(
      plant_with_springs);
  // Create command sender.
  auto command_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          FLAGS_channel_u, &lcm, TriggerTypeSet({TriggerType::kForced})));
  auto command_sender =
      builder.AddSystem<systems::RobotCommandSender>(plant_with_springs);
  auto osc = builder.AddSystem<systems::controllers::OperationalSpaceControl>(
      plant_with_springs, plant_without_springs, true,
      FLAGS_print_osc /*print_tracking_info*/);







  // OSC setup

  // Cost
  MatrixXd Q_accel = 0.001 * MatrixXd::Identity(n_v, n_v);
  osc->SetAccelerationCostForAllJoints(Q_accel);
  // Soft constraint
  // w_contact_relax shouldn't be too big, cause we want tracking error to be
  // important
  double w_contact_relax = 20000;
  osc->SetWeightOfSoftContactConstraint(w_contact_relax);
  double mu = 0.4;
  // Friction coefficient
  osc->SetContactFriction(mu);
  vector<int> stance_modes = {0, 2};
  for (int mode : stance_modes) {
    osc->AddStateAndContactPoint(mode, "toe_left", front_contact_disp);
    osc->AddStateAndContactPoint(mode, "toe_left", rear_contact_disp);
    osc->AddStateAndContactPoint(mode, "toe_right", front_contact_disp);
    osc->AddStateAndContactPoint(mode, "toe_right", rear_contact_disp);
  }

  // Center of mass tracking
  MatrixXd W_com = MatrixXd::Identity(3, 3);
  W_com(0, 0) = 2;
  W_com(1, 1) = 2;
  W_com(2, 2) = 2000;
  MatrixXd K_p_com = 50 * MatrixXd::Identity(3, 3);
  MatrixXd K_d_com = 10 * MatrixXd::Identity(3, 3);
  ComTrackingData center_of_mass_traj("lipm_traj", 3, K_p_com, K_d_com, W_com,
      &tree_with_springs,
      &tree_without_springs);
  osc->AddTrackingData(&center_of_mass_traj);

  // Build OSC problem
  osc->Build();
  // Connect ports
  builder.Connect(state_receiver->get_output_port(0),
                  osc->get_robot_output_input_port());
  builder.Connect(fsm->get_output_port(0), osc->get_fsm_input_port());
  builder.Connect(lipm_traj_generator->get_output_port(0),
                  osc->get_tracking_data_input_port("lipm_traj"));
  builder.Connect(cp_traj_generator->get_output_port(0),
                  osc->get_tracking_data_input_port("cp_traj"));
  builder.Connect(head_traj_gen->get_output_port(0),
                  osc->get_tracking_data_input_port("pelvis_balance_traj"));
  builder.Connect(head_traj_gen->get_output_port(0),
                  osc->get_tracking_data_input_port("pelvis_heading_traj"));
  builder.Connect(osc->get_output_port(0), command_sender->get_input_port(0));

  // Create the diagram
  auto owned_diagram = builder.Build();
  owned_diagram->set_name("osc walking controller");

  // Run lcm-driven simulation
  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm3, std::move(owned_diagram), state_receiver, FLAGS_channel_x,
      false);
  loop.Simulate();

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::DoMain(argc, argv); }
