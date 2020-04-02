#include <drake/lcmt_contact_results_for_viz.hpp>
#include <drake/multibody/parsing/parser.h>
#include <drake/systems/lcm/lcm_interface_system.h>
#include <gflags/gflags.h>
#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_cassie_mujoco_contact.hpp"
#include "examples/Cassie/cassie_utils.h"
#include "examples/Cassie/osc_jump/com_traj_generator.h"
#include "examples/Cassie/osc_jump/flight_foot_traj_generator.h"
#include "examples/Cassie/osc_jump/jumping_event_based_fsm.h"
#include "examples/Cassie/osc_jump/pelvis_orientation_traj_generator.h"
#include "lcm/lcm_trajectory.h"
#include "systems/controllers/osc/operational_space_control_mbp.h"
#include "systems/controllers/osc/osc_tracking_data.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/robot_lcm_systems.h"
#include "drake/multibody/joints/floating_base_types.h"
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
using drake::trajectories::PiecewisePolynomial;
using examples::JumpingEventFsm;
using examples::Cassie::osc_jump::COMTrajGenerator;
using examples::Cassie::osc_jump::FlightFootTrajGenerator;
using examples::Cassie::osc_jump::PPolyPassthrough;
using multibody::GetBodyIndexFromName;
using multibody::MultibodyDistanceConstraint;
using systems::controllers::ComTrackingDataMBP;
using systems::controllers::JointSpaceTrackingDataMBP;
using systems::controllers::RotTaskSpaceTrackingDataMBP;
using systems::controllers::TransTaskSpaceTrackingDataMBP;

namespace examples {

DEFINE_double(publish_rate, 1000.0, "Target publish rate for OSC");

DEFINE_string(channel_x, "CASSIE_STATE",
              "The name of the channel which receives state");
DEFINE_string(channel_u, "OSC_JUMPING",
              "The name of the channel which publishes command");
DEFINE_double(balance_height, 1.125,
              "Balancing height for Cassie before attempting the jump");

DEFINE_bool(print_osc, false, "whether to print the osc debug message or not");
DEFINE_bool(is_two_phase, false,
            "true: only right/left single support"
            "false: both double and single support");
DEFINE_string(traj_name, "", "File to load saved trajectories from");

DEFINE_double(delay_time, 0.0, "time to wait before executing jump");
DEFINE_double(x_offset, 0.0, "Offset to add to the CoM trajectory");
DEFINE_bool(contact_based_fsm, true, "The contact based fsm transitions "
                                         "between states using contact data.");
DEFINE_string(simulator, "DRAKE", "Simulator used, important for determining "
                                  "how to interpret contact information");
// DEFINE_double(x_offset, 0.18, "Offset to add to the CoM trajectory");

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
  // Initialize using timestepping to avoid joint limit warnings
  MultibodyPlant<double> plant_with_springs(1e-5);
  MultibodyPlant<double> plant_without_springs(1e-5);
  Parser parser_with_springs(&plant_with_springs, &scene_graph);
  Parser parser_without_springs(&plant_without_springs, &scene_graph);

  /**** Initialize the plants ****/

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

  // Create maps for joints
  map<string, int> pos_map =
      multibody::makeNameToPositionsMap(plant_without_springs);
  map<string, int> vel_map =
      multibody::makeNameToVelocitiesMap(plant_without_springs);
  map<string, int> act_map =
      multibody::makeNameToActuatorsMap(plant_without_springs);

  /**** Get trajectory from optimization ****/
  const LcmTrajectory& loaded_traj = LcmTrajectory(
      "/home/yangwill/Documents/research/projects/cassie/jumping/saved_trajs/" +
      FLAGS_traj_name);

  const LcmTrajectory::Trajectory& lcm_com_traj =
      loaded_traj.getTrajectory("center_of_mass_trajectory");
  const LcmTrajectory::Trajectory& lcm_l_foot_traj =
      loaded_traj.getTrajectory("left_foot_trajectory");
  const LcmTrajectory::Trajectory& lcm_r_foot_traj =
      loaded_traj.getTrajectory("right_foot_trajectory");
  const LcmTrajectory::Trajectory& lcm_pelvis_rot_traj =
      loaded_traj.getTrajectory("pelvis_rot_trajectory");

  PiecewisePolynomial<double> com_traj =
      PiecewisePolynomial<double>::CubicHermite(
      lcm_com_traj.time_vector, lcm_com_traj.datapoints.topRows(3),
      lcm_com_traj.datapoints.bottomRows(3));
  const PiecewisePolynomial<double>& l_foot_trajectory =
      PiecewisePolynomial<double>::CubicHermite(
          lcm_l_foot_traj.time_vector, lcm_l_foot_traj.datapoints.topRows(3),
          lcm_l_foot_traj.datapoints.bottomRows(3));
  const PiecewisePolynomial<double>& r_foot_trajectory =
      PiecewisePolynomial<double>::CubicHermite(
          lcm_r_foot_traj.time_vector, lcm_r_foot_traj.datapoints.topRows(3),
          lcm_r_foot_traj.datapoints.bottomRows(3));
  const PiecewisePolynomial<double>& pelvis_rot_trajectory =
      PiecewisePolynomial<double>::CubicShapePreserving(
          lcm_pelvis_rot_traj.time_vector,
          lcm_pelvis_rot_traj.datapoints.topRows(4));

  double flight_time = FLAGS_delay_time + 0.285773;  // For the time-based FSM
  double land_time = FLAGS_delay_time + 0.67272637;
  std::vector<double> transition_times = {FLAGS_delay_time, flight_time,
                                          land_time};

  // Cassie parameters
  Vector3d front_contact_disp(-0.0457, 0.112, 0);
  Vector3d rear_contact_disp(0.088, 0, 0);

  // Get body indices for cassie with springs
  auto pelvis_idx = plant_with_springs.GetBodyByName("pelvis").index();

  //  double x_offset = com_traj.value(0)(0) -
  //                    (r_foot_trajectory.value(0)(0) +
  //                     (front_contact_disp[0] + rear_contact_disp[0]) / 2);
  Vector3d support_center_offset;
  support_center_offset << FLAGS_x_offset, 0.0, 0.0;
  std::vector<double> breaks = com_traj.get_segment_times();
  VectorXd breaks_vector = Eigen::Map<VectorXd>(breaks.data(), breaks.size());
  MatrixXd offset_points = support_center_offset.replicate(1, breaks.size());
  PiecewisePolynomial<double> offset_traj =
      PiecewisePolynomial<double>::ZeroOrderHold(breaks_vector, offset_points);
  com_traj = com_traj + offset_traj;
  std::cout << "Target balance height: " << lcm_com_traj.datapoints.col(0)(2)
            << std::endl;


  /**** Initialize all the leaf systems ****/
  SIMULATOR type;
  if(FLAGS_simulator == "DRAKE")
    type = DRAKE;
  else if (FLAGS_simulator == "MUJOCO")
    type = MUJOCO;

  drake::lcm::DrakeLcm lcm;
  auto state_receiver =
      builder.AddSystem<systems::RobotOutputReceiver>(plant_with_springs);
  auto com_traj_generator = builder.AddSystem<COMTrajGenerator>(
      plant_with_springs, pelvis_idx, front_contact_disp, rear_contact_disp,
      com_traj, com_traj.value(0)(2), FLAGS_delay_time);
  auto l_foot_traj_generator = builder.AddSystem<FlightFootTrajGenerator>(
      plant_with_springs, "hip_left", true, l_foot_trajectory,
      FLAGS_delay_time);
  auto r_foot_traj_generator = builder.AddSystem<FlightFootTrajGenerator>(
      plant_with_springs, "hip_right", false, r_foot_trajectory,
      FLAGS_delay_time);
  auto pelvis_rot_traj_generator = builder.AddSystem<PPolyPassthrough>(
      plant_with_springs, pelvis_rot_trajectory, "pelvis_rot_tracking_data",
      FLAGS_delay_time);
  auto fsm = builder.AddSystem<dairlib::examples::JumpingEventFsm>(
      plant_with_springs, transition_times,
      FLAGS_contact_based_fsm, BALANCE, type);
  auto command_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          FLAGS_channel_u, &lcm, TriggerTypeSet({TriggerType::kForced})));
  auto command_sender =
      builder.AddSystem<systems::RobotCommandSender>(plant_with_springs);
  auto osc =
      builder.AddSystem<systems::controllers::OperationalSpaceControlMBP>(
          plant_with_springs, plant_without_springs, true,
          FLAGS_print_osc); /*print_tracking_info*/
  auto osc_debug_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_osc_output>(
          "OSC_DEBUG", &lcm, TriggerTypeSet({TriggerType::kForced})));

  LcmSubscriberSystem* contact_results_sub;
  if(FLAGS_simulator == "DRAKE"){
    contact_results_sub = builder.AddSystem(
        LcmSubscriberSystem::Make<drake::lcmt_contact_results_for_viz>(
            "CASSIE_CONTACT_DRAKE", &lcm));
  }
  else if (FLAGS_simulator == "MUJOCO"){
    contact_results_sub = builder.AddSystem(
        LcmSubscriberSystem::Make<dairlib::lcmt_cassie_mujoco_contact>(
            "CASSIE_CONTACT_MUJOCO", &lcm));
  }
  else if (FLAGS_simulator == "GAZEBO"){
    // TODO(yangwill): Set up contact results in Gazebo
  }
  else{
    std::cerr << "Unknown simulator type!" << std::endl;
  }

  /**** OSC setup ****/
  // Cost
  MatrixXd Q_accel = 1e-6 * MatrixXd::Identity(n_v, n_v);
  osc->SetAccelerationCostForAllJoints(Q_accel);
  // Soft constraint on contacts
  double w_contact_relax = 20000;
  osc->SetWeightOfSoftContactConstraint(w_contact_relax);
  double mu = 0.4;
  // Contact information for OSC
  osc->SetContactFriction(mu);
  vector<FSM_STATE> stance_modes = {BALANCE, CROUCH, LAND};
  for (FSM_STATE mode : stance_modes) {
    osc->AddStateAndContactPoint(mode, "toe_left", front_contact_disp);
    osc->AddStateAndContactPoint(mode, "toe_left", rear_contact_disp);
    osc->AddStateAndContactPoint(mode, "toe_right", front_contact_disp);
    osc->AddStateAndContactPoint(mode, "toe_right", rear_contact_disp);
  }

  double achilles_length = .5012;

  const auto& thigh_left = plant_without_springs.GetBodyByName("thigh_left");
  const auto& heel_spring_left =
      plant_without_springs.GetBodyByName("heel_spring_left");
  const auto& thigh_right = plant_without_springs.GetBodyByName("thigh_right");
  const auto& heel_spring_right =
      plant_without_springs.GetBodyByName("heel_spring_right");

  Vector3d pt_on_heel_spring = Vector3d(.11877, -.01, 0.0);
  Vector3d pt_on_thigh_left = Vector3d(0.0, 0.0, 0.045);
  Vector3d pt_on_thigh_right = Vector3d(0.0, 0.0, -0.045);

  MultibodyDistanceConstraint left_thigh_bar_constraint =
      MultibodyDistanceConstraint(plant_without_springs, thigh_left,
                                  pt_on_thigh_left, heel_spring_left,
                                  pt_on_heel_spring, achilles_length);
  MultibodyDistanceConstraint right_thigh_bar_constraint =
      MultibodyDistanceConstraint(plant_without_springs, thigh_right,
                                  pt_on_thigh_right, heel_spring_right,
                                  pt_on_heel_spring, achilles_length);

  osc->AddDistanceConstraint(left_thigh_bar_constraint);
  osc->AddDistanceConstraint(right_thigh_bar_constraint);

  /**** Tracking Data for OSC *****/
  // Center of mass tracking
  MatrixXd W_com = MatrixXd::Identity(3, 3);
  W_com(0, 0) = 2000;
  W_com(1, 1) = 200;
  W_com(2, 2) = 2000;
  MatrixXd K_p_com = 64 * MatrixXd::Identity(3, 3);
  MatrixXd K_d_com = 16 * MatrixXd::Identity(3, 3);
  ComTrackingDataMBP com_tracking_data("com_traj", 3, K_p_com, K_d_com, W_com,
                                       &plant_with_springs,
                                       &plant_without_springs);
  for (FSM_STATE mode : stance_modes) {
    com_tracking_data.AddStateToTrack(mode);
  }
  osc->AddTrackingData(&com_tracking_data);

  // Feet tracking
  MatrixXd W_swing_foot = 1 * MatrixXd::Identity(3, 3);
  W_swing_foot(0, 0) = 1000;
  W_swing_foot(1, 1) = 1000;
  W_swing_foot(2, 2) = 1000;
  MatrixXd K_p_sw_ft = 36 * MatrixXd::Identity(3, 3);
  MatrixXd K_d_sw_ft = 12 * MatrixXd::Identity(3, 3);

  TransTaskSpaceTrackingDataMBP left_foot_tracking_data(
      "l_foot_traj", 3, K_p_sw_ft, K_d_sw_ft, W_swing_foot, &plant_with_springs,
      &plant_without_springs);
  TransTaskSpaceTrackingDataMBP right_foot_tracking_data(
      "r_foot_traj", 3, K_p_sw_ft, K_d_sw_ft, W_swing_foot, &plant_with_springs,
      &plant_without_springs);
  left_foot_tracking_data.AddStateAndPointToTrack(FLIGHT, "toe_left");
  right_foot_tracking_data.AddStateAndPointToTrack(FLIGHT, "toe_right");

  double w_pelvis_balance = 20;
  double w_heading = 10;
  double k_p_pelvis_balance = 16;  // 100
  double k_d_pelvis_balance = 8;   // 80
  double k_p_heading = 16;         // 50
  double k_d_heading = 8;          // 40
  Matrix3d W_pelvis = w_pelvis_balance * MatrixXd::Identity(3, 3);
  W_pelvis(2, 2) = w_heading;
  Matrix3d K_p_pelvis = k_p_pelvis_balance * 2 * MatrixXd::Identity(3, 3);
  K_p_pelvis(2, 2) = k_p_heading;
  Matrix3d K_d_pelvis = k_d_pelvis_balance * MatrixXd::Identity(3, 3);
  K_d_pelvis(2, 2) = k_d_heading;
  RotTaskSpaceTrackingDataMBP pelvis_rot_tracking_data(
      "pelvis_rot_tracking_data", 3, K_p_pelvis, K_d_pelvis, W_pelvis,
      &plant_with_springs, &plant_without_springs);

  for (FSM_STATE mode : stance_modes) {
    pelvis_rot_tracking_data.AddStateAndFrameToTrack(mode, "pelvis");
  }

  osc->AddTrackingData(&pelvis_rot_tracking_data);

  osc->AddTrackingData(&left_foot_tracking_data);
  osc->AddTrackingData(&right_foot_tracking_data);

  // Build OSC problem
  osc->Build();

  std::cout << "Built OSC" << std::endl;

  /*****Connect ports*****/

  // State receiver connections (Connected through LCM driven loop)

  // OSC connections
  builder.Connect(fsm->get_output_port(0), osc->get_fsm_input_port());
  builder.Connect(state_receiver->get_output_port(0),
                  osc->get_robot_output_input_port());
  builder.Connect(com_traj_generator->get_output_port(0),
                  osc->get_tracking_data_input_port("com_traj"));
  builder.Connect(l_foot_traj_generator->get_output_port(0),
                  osc->get_tracking_data_input_port("l_foot_traj"));
  builder.Connect(r_foot_traj_generator->get_output_port(0),
                  osc->get_tracking_data_input_port("r_foot_traj"));
  builder.Connect(
      pelvis_rot_traj_generator->get_output_port(0),
      osc->get_tracking_data_input_port("pelvis_rot_tracking_data"));

  // FSM connections
  builder.Connect(contact_results_sub->get_output_port(),
                  fsm->get_contact_input_port());
  builder.Connect(state_receiver->get_output_port(0),
                  fsm->get_state_input_port());

  // Trajectory generator connections
  builder.Connect(state_receiver->get_output_port(0),
                  com_traj_generator->get_state_input_port());
  builder.Connect(state_receiver->get_output_port(0),
                  l_foot_traj_generator->get_state_input_port());
  builder.Connect(state_receiver->get_output_port(0),
                  r_foot_traj_generator->get_state_input_port());
  builder.Connect(state_receiver->get_output_port(0),
                  pelvis_rot_traj_generator->get_state_input_port());
  builder.Connect(fsm->get_output_port(0),
                  com_traj_generator->get_fsm_input_port());
  builder.Connect(fsm->get_output_port(0),
                  l_foot_traj_generator->get_fsm_input_port());
  builder.Connect(fsm->get_output_port(0),
                  r_foot_traj_generator->get_fsm_input_port());

  // Publisher connections
  builder.Connect(osc->get_osc_output_port(),
      command_sender->get_input_port(0));
  builder.Connect(command_sender->get_output_port(0),
                  command_pub->get_input_port());
  builder.Connect(osc->get_osc_debug_port(), osc_debug_pub->get_input_port());

  // Run lcm-driven simulation
  // Create the diagram
  auto owned_diagram = builder.Build();
  owned_diagram->set_name(("osc jumping controller"));

  // Run lcm-driven simulation
  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm, std::move(owned_diagram), state_receiver, FLAGS_channel_x, true);
  loop.Simulate();

  return 0;
}
}  // namespace examples
}  // namespace dairlib

int main(int argc, char* argv[]) {
  return dairlib::examples::DoMain(argc, argv);
}
