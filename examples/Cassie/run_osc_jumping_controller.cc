#include <drake/lcmt_contact_results_for_viz.hpp>
#include <drake/multibody/parsing/parser.h>
#include <gflags/gflags.h>

#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "examples/Cassie/cassie_utils.h"
#include "examples/Cassie/osc_jump/com_traj_generator.h"
#include "examples/Cassie/osc_jump/flight_foot_traj_generator.h"
#include "examples/Cassie/osc_jump/jumping_event_based_fsm.h"
#include "examples/Cassie/osc_jump/pelvis_orientation_traj_generator.h"
#include "lcm/lcm_trajectory.h"
#include "systems/controllers/osc/operational_space_control.h"
#include "systems/controllers/osc/osc_tracking_data.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/primitives/gaussian_noise_pass_through.h"
#include "systems/robot_lcm_systems.h"

#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"

namespace dairlib {

using std::cout;
using std::endl;
using std::map;
using std::pair;
using std::string;
using std::vector;

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using drake::geometry::SceneGraph;
using drake::multibody::Frame;
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
using examples::Cassie::osc_jump::PelvisOrientationTrajGenerator;
using systems::controllers::ComTrackingData;
using systems::controllers::JointSpaceTrackingData;
using systems::controllers::RotTaskSpaceTrackingData;
using systems::controllers::TransTaskSpaceTrackingData;

namespace examples {

DEFINE_double(publish_rate, 1000.0, "Target publish rate for OSC");
DEFINE_string(channel_x, "CASSIE_STATE_SIMULATION",
              "The name of the channel which receives state");
DEFINE_string(channel_u, "CASSIE_INPUT",
              "The name of the channel which publishes command");
DEFINE_bool(print_osc, false, "whether to print the osc debug message or not");
DEFINE_string(traj_name, "", "File to load saved trajectories from");
DEFINE_double(delay_time, 0.0, "time to wait before executing jump");
DEFINE_double(x_offset, 0.0, "Offset to add to the CoM trajectory");
DEFINE_bool(contact_based_fsm, true,
            "The contact based fsm transitions "
            "between states using contact data.");
DEFINE_double(transition_delay, 0.0,
              "Time to wait after trigger to "
              "transition between FSM states.");
DEFINE_string(simulator, "DRAKE",
              "Simulator used, important for determining how to interpret "
              "contact information. Other options include MUJOCO and soon to "
              "include GAZEBO.");
DEFINE_bool(add_noise, false,
            "Whether to add gaussian noise to state "
            "inputted to controller");
DEFINE_int32(init_fsm_state, BALANCE, "Initial state of the FSM");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Build the controller diagram
  DiagramBuilder<double> builder;

  // Built the Cassie MBPs
  drake::multibody::MultibodyPlant<double> plant_w_springs(0.0);
  addCassieMultibody(&plant_w_springs, nullptr, true,
                     "examples/Cassie/urdf/cassie_v2.urdf",
                     true /*spring model*/, false /*loop closure*/);
  drake::multibody::MultibodyPlant<double> plant_wo_springs(0.0);
  addCassieMultibody(&plant_wo_springs, nullptr, true,
                     "examples/Cassie/urdf/cassie_fixed_springs.urdf", false,
                     false);
  plant_w_springs.Finalize();
  plant_wo_springs.Finalize();

  auto context_w_spr = plant_w_springs.CreateDefaultContext();
  auto context_wo_spr = plant_wo_springs.CreateDefaultContext();

  int nq = plant_wo_springs.num_positions();
  int nv = plant_wo_springs.num_velocities();
  int nx = nq + nv;
  int n_modes = 3;
  // Create maps for joints
  map<string, int> pos_map =
      multibody::makeNameToPositionsMap(plant_wo_springs);
  map<string, int> vel_map =
      multibody::makeNameToVelocitiesMap(plant_wo_springs);
  map<string, int> act_map =
      multibody::makeNameToActuatorsMap(plant_wo_springs);

  auto left_toe = LeftToeFront(plant_wo_springs);
  auto left_heel = LeftToeRear(plant_wo_springs);
  auto right_toe = RightToeFront(plant_wo_springs);
  auto right_heel = RightToeRear(plant_wo_springs);

  /**** Get trajectory from optimization ****/
  const LcmTrajectory& original_traj = LcmTrajectory(
      "/home/yangwill/Documents/research/projects/cassie/jumping/saved_trajs/" +
      FLAGS_traj_name);
  const LcmTrajectory& processed_trajs = LcmTrajectory(
      "/home/yangwill/Documents/research/projects/cassie/jumping/saved_trajs/" +
      FLAGS_traj_name + "_processed");

  const LcmTrajectory::Trajectory lcm_com_traj =
      processed_trajs.GetTrajectory("center_of_mass_trajectory");
  const LcmTrajectory::Trajectory lcm_l_foot_traj =
      processed_trajs.GetTrajectory("left_foot_trajectory");
  const LcmTrajectory::Trajectory lcm_r_foot_traj =
      processed_trajs.GetTrajectory("right_foot_trajectory");
  const LcmTrajectory::Trajectory lcm_pelvis_rot_traj =
      processed_trajs.GetTrajectory("pelvis_rot_trajectory");
  vector<PiecewisePolynomial<double>> state_trajs;
  for (int i = 0; i < n_modes; ++i) {
    const LcmTrajectory::Trajectory state_traj_i = original_traj.GetTrajectory(
        "cassie_jumping_trajectory_x_u" + std::to_string(i));
    state_trajs.push_back(PiecewisePolynomial<double>::CubicHermite(
        state_traj_i.time_vector, state_traj_i.datapoints.topRows(nx),
        state_traj_i.datapoints.topRows(2 * nx).bottomRows(nx)));
  }

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
  PiecewisePolynomial<double> pelvis_rot_trajectory;
  pelvis_rot_trajectory = PiecewisePolynomial<double>::CubicHermite(
      lcm_pelvis_rot_traj.time_vector,
      lcm_pelvis_rot_traj.datapoints.topRows(4),
      lcm_pelvis_rot_traj.datapoints.bottomRows(4));

  // For the time-based FSM
  double flight_time = FLAGS_delay_time + state_trajs[0].end_time();
  double land_time = FLAGS_delay_time + state_trajs[1].end_time();
  std::vector<double> transition_times = {FLAGS_delay_time, flight_time,
                                          land_time};

  Vector3d support_center_offset;
  support_center_offset << FLAGS_x_offset, 0.0, 0.0;
  std::vector<double> breaks = com_traj.get_segment_times();
  VectorXd breaks_vector = Eigen::Map<VectorXd>(breaks.data(), breaks.size());
  MatrixXd offset_points = support_center_offset.replicate(1, breaks.size());
  PiecewisePolynomial<double> offset_traj =
      PiecewisePolynomial<double>::ZeroOrderHold(breaks_vector, offset_points);
  com_traj = com_traj + offset_traj;

  /**** Initialize all the leaf systems ****/
  drake::lcm::DrakeLcm lcm;

  vector<pair<const Vector3d, const Frame<double>&>> contact_points;
  contact_points.push_back(LeftToeFront(plant_wo_springs));
  contact_points.push_back(RightToeFront(plant_wo_springs));
  contact_points.push_back(LeftToeRear(plant_wo_springs));
  contact_points.push_back(RightToeRear(plant_wo_springs));

  auto state_receiver =
      builder.AddSystem<systems::RobotOutputReceiver>(plant_w_springs);
  auto com_traj_generator = builder.AddSystem<COMTrajGenerator>(
      plant_w_springs, contact_points, com_traj, FLAGS_delay_time);
  auto l_foot_traj_generator = builder.AddSystem<FlightFootTrajGenerator>(
      plant_w_springs, "hip_left", true, l_foot_trajectory, FLAGS_delay_time);
  auto r_foot_traj_generator = builder.AddSystem<FlightFootTrajGenerator>(
      plant_w_springs, "hip_right", false, r_foot_trajectory, FLAGS_delay_time);
  auto pelvis_rot_traj_generator =
      builder.AddSystem<PelvisOrientationTrajGenerator>(
          plant_w_springs, pelvis_rot_trajectory, "pelvis_rot_tracking_data",
          FLAGS_delay_time);
  auto fsm = builder.AddSystem<dairlib::examples::JumpingEventFsm>(
      plant_w_springs, transition_times, FLAGS_contact_based_fsm,
      FLAGS_transition_delay, (FSM_STATE)FLAGS_init_fsm_state);
  auto command_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          FLAGS_channel_u, &lcm, TriggerTypeSet({TriggerType::kForced})));
  auto command_sender =
      builder.AddSystem<systems::RobotCommandSender>(plant_w_springs);
  auto osc = builder.AddSystem<systems::controllers::OperationalSpaceControl>(
      plant_w_springs, plant_wo_springs, context_w_spr.get(),
      context_wo_spr.get(), true, FLAGS_print_osc); /*print_tracking_info*/
  auto osc_debug_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_osc_output>(
          "OSC_DEBUG", &lcm, TriggerTypeSet({TriggerType::kForced})));

  LcmSubscriberSystem* contact_results_sub = nullptr;
  if (FLAGS_simulator == "DRAKE") {
    contact_results_sub = builder.AddSystem(
        LcmSubscriberSystem::Make<drake::lcmt_contact_results_for_viz>(
            "CASSIE_CONTACT_DRAKE", &lcm));
  } else if (FLAGS_simulator == "MUJOCO") {
    contact_results_sub = builder.AddSystem(
        LcmSubscriberSystem::Make<drake::lcmt_contact_results_for_viz>(
            "CASSIE_CONTACT_MUJOCO", &lcm));
  } else if (FLAGS_simulator == "GAZEBO") {
    // TODO(yangwill): Set up contact results in Gazebo
  } else {
    std::cerr << "Unknown simulator type!" << std::endl;
  }

  /**** OSC setup ****/
  // Cost
  MatrixXd Q_accel = 1e-6 * MatrixXd::Identity(nv, nv);
  osc->SetAccelerationCostForAllJoints(Q_accel);
  // Soft constraint on contacts
  double w_contact_relax = 20000;
  osc->SetWeightOfSoftContactConstraint(w_contact_relax);

  // Contact information for OSC
  double mu = 0.4;
  osc->SetContactFriction(mu);

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
  vector<FSM_STATE> stance_modes = {BALANCE, CROUCH, LAND};
  for (FSM_STATE mode : stance_modes) {
    osc->AddStateAndContactPoint(mode, &left_toe_evaluator);
    osc->AddStateAndContactPoint(mode, &left_heel_evaluator);
    osc->AddStateAndContactPoint(mode, &right_toe_evaluator);
    osc->AddStateAndContactPoint(mode, &right_heel_evaluator);
  }

  multibody::KinematicEvaluatorSet<double> evaluators(plant_wo_springs);
  auto left_loop = LeftLoopClosureEvaluator(plant_wo_springs);
  auto right_loop = RightLoopClosureEvaluator(plant_wo_springs);
  evaluators.add_evaluator(&left_loop);
  evaluators.add_evaluator(&right_loop);
  osc->AddKinematicConstraint(&evaluators);

  /**** Tracking Data for OSC *****/
  // Center of mass tracking
  MatrixXd W_com = MatrixXd::Identity(3, 3);
  W_com(0, 0) = 2000;
  W_com(1, 1) = 200;
  W_com(2, 2) = 2000;
  MatrixXd K_p_com = 64 * MatrixXd::Identity(3, 3);
  MatrixXd K_d_com = 16 * MatrixXd::Identity(3, 3);
  ComTrackingData com_tracking_data("com_traj", K_p_com, K_d_com, W_com,
                                    plant_w_springs, plant_wo_springs);
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

  TransTaskSpaceTrackingData left_foot_tracking_data(
      "l_foot_traj", K_p_sw_ft, K_d_sw_ft, W_swing_foot, plant_w_springs,
      plant_wo_springs);
  TransTaskSpaceTrackingData right_foot_tracking_data(
      "r_foot_traj", K_p_sw_ft, K_d_sw_ft, W_swing_foot, plant_w_springs,
      plant_wo_springs);
  left_foot_tracking_data.AddStateAndPointToTrack(FLIGHT, "toe_left");
  right_foot_tracking_data.AddStateAndPointToTrack(FLIGHT, "toe_right");

  // Pelvis orientation tracking
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
  RotTaskSpaceTrackingData pelvis_rot_tracking_data(
      "pelvis_rot_tracking_data", K_p_pelvis, K_d_pelvis, W_pelvis,
      plant_w_springs, plant_wo_springs);

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
  drake::systems::LeafSystem<double>* controller_state_input = state_receiver;
  std::cout << "Running with noise: " << FLAGS_add_noise << std::endl;
  if (FLAGS_add_noise) {
    MatrixXd pos_cov = MatrixXd::Zero(plant_w_springs.num_positions(),
                                      plant_w_springs.num_positions());
    pos_cov(4, 4) = 0.0;
    MatrixXd vel_cov = MatrixXd::Zero(plant_w_springs.num_velocities(),
                                      plant_w_springs.num_velocities());
    vel_cov(5, 5) = 0.0;
    auto gaussian_noise = builder.AddSystem<systems::GaussianNoisePassThrough>(
        plant_w_springs.num_positions(), plant_w_springs.num_velocities(),
        plant_w_springs.num_actuators(), pos_cov, vel_cov);
    builder.Connect(state_receiver->get_output_port(0),
                    gaussian_noise->get_input_port());
    controller_state_input = gaussian_noise;
  }

  // OSC connections
  builder.Connect(fsm->get_output_port(0), osc->get_fsm_input_port());
  builder.Connect(controller_state_input->get_output_port(0),
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
  builder.Connect(controller_state_input->get_output_port(0),
                  fsm->get_state_input_port());

  // Trajectory generator connections
  builder.Connect(controller_state_input->get_output_port(0),
                  com_traj_generator->get_state_input_port());
  builder.Connect(controller_state_input->get_output_port(0),
                  l_foot_traj_generator->get_state_input_port());
  builder.Connect(controller_state_input->get_output_port(0),
                  r_foot_traj_generator->get_state_input_port());
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
