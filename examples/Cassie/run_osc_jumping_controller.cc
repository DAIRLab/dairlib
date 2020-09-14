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
#include "lcm/dircon_saved_trajectory.h"
#include "lcm/lcm_trajectory.h"
#include "systems/controllers/osc/operational_space_control.h"
#include "systems/controllers/osc/osc_tracking_data.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/primitives/gaussian_noise_pass_through.h"
#include "systems/robot_lcm_systems.h"
#include "yaml-cpp/yaml.h"

#include "drake/common/yaml/yaml_read_archive.h"
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
using examples::osc_jump::COMTrajGenerator;
using examples::osc_jump::FlightFootTrajGenerator;
using examples::osc_jump::JumpingEventFsm;
using examples::osc_jump::PelvisOrientationTrajGenerator;
using systems::controllers::ComTrackingData;
using systems::controllers::JointSpaceTrackingData;
using systems::controllers::RotTaskSpaceTrackingData;
using systems::controllers::TransTaskSpaceTrackingData;

namespace examples {

DEFINE_double(publish_rate, 1000.0, "Target publish rate for OSC");
DEFINE_string(channel_x, "CASSIE_STATE_SIMULATION",
              "The name of the channel which receives state");
DEFINE_string(channel_u, "OSC_JUMPING",
              "The name of the channel which publishes command");
DEFINE_bool(print_osc, false, "whether to print the osc debug message or not");
DEFINE_string(folder_path,
              "/home/yangwill/Documents/research/projects/cassie"
              "/jumping/saved_trajs/",
              "Folder path for where the trajectory names are stored");
DEFINE_string(traj_name, "", "File to load saved trajectories from");
DEFINE_string(mode_name, "state_input_trajectory",
              "Base name of each trajectory");
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
              "include contact results from the GM contact estimator.");
DEFINE_bool(add_noise, false,
            "Whether to add gaussian noise to state "
            "inputted to controller");
DEFINE_int32(init_fsm_state, osc_jump::BALANCE, "Initial state of the FSM");
DEFINE_string(gains_filename, "examples/Cassie/osc_jump/osc_jumping_gains.yaml",
              "Filepath containing gains");

struct OSCJumpingGains {
  // costs
  double w_input;
  double w_accel;
  double w_soft_constraint;
  double x_offset;
  // center of mass tracking
  std::vector<double> CoMW;
  std::vector<double> CoMKp;
  std::vector<double> CoMKd;
  // pelvis orientation tracking
  std::vector<double> PelvisRotW;
  std::vector<double> PelvisRotKp;
  std::vector<double> PelvisRotKd;
  // flight foot tracking
  std::vector<double> FlightFootW;
  std::vector<double> FlightFootKp;
  std::vector<double> FlightFootKd;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(w_input));
    a->Visit(DRAKE_NVP(w_accel));
    a->Visit(DRAKE_NVP(w_soft_constraint));
    a->Visit(DRAKE_NVP(x_offset));
    a->Visit(DRAKE_NVP(CoMW));
    a->Visit(DRAKE_NVP(CoMKp));
    a->Visit(DRAKE_NVP(CoMKd));
    a->Visit(DRAKE_NVP(PelvisRotW));
    a->Visit(DRAKE_NVP(PelvisRotKp));
    a->Visit(DRAKE_NVP(PelvisRotKd));
    a->Visit(DRAKE_NVP(FlightFootW));
    a->Visit(DRAKE_NVP(FlightFootKp));
    a->Visit(DRAKE_NVP(FlightFootKd));
  }
};

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

  // Get contact frames and position (doesn't matter whether we use
  // plant_w_springs or plant_wo_springs because the contact frames exit in both
  // plants)
  auto left_toe = LeftToeFront(plant_wo_springs);
  auto left_heel = LeftToeRear(plant_wo_springs);
  auto right_toe = RightToeFront(plant_wo_springs);
  auto right_heel = RightToeRear(plant_wo_springs);

  int nq = plant_wo_springs.num_positions();
  int nv = plant_wo_springs.num_velocities();
  int nx = nq + nv;

  // Create maps for joints
  map<string, int> pos_map =
      multibody::makeNameToPositionsMap(plant_wo_springs);
  map<string, int> vel_map =
      multibody::makeNameToVelocitiesMap(plant_wo_springs);
  map<string, int> act_map =
      multibody::makeNameToActuatorsMap(plant_wo_springs);

  std::vector<std::pair<const Vector3d, const drake::multibody::Frame<double>&>>
      feet_contact_points = {left_toe, right_toe};

  /**** Convert the gains from the yaml struct to Eigen Matrices ****/
  OSCJumpingGains gains;
  const YAML::Node& root =
      YAML::LoadFile(FindResourceOrThrow(FLAGS_gains_filename));
  drake::yaml::YamlReadArchive(root).Accept(&gains);

  MatrixXd W_com = Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      gains.CoMW.data(), 3, 3);
  MatrixXd K_p_com = Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      gains.CoMKp.data(), 3, 3);
  MatrixXd K_d_com = Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      gains.CoMKd.data(), 3, 3);
  MatrixXd W_pelvis = Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      gains.PelvisRotW.data(), 3, 3);
  MatrixXd K_p_pelvis = Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      gains.PelvisRotKp.data(), 3, 3);
  MatrixXd K_d_pelvis = Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      gains.PelvisRotKd.data(), 3, 3);
  MatrixXd W_flight_foot = Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      gains.FlightFootW.data(), 3, 3);
  MatrixXd K_p_flight_foot = Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      gains.FlightFootKp.data(), 3, 3);
  MatrixXd K_d_flight_foot = Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      gains.FlightFootKd.data(), 3, 3);

  /**** Get trajectory from optimization ****/
  const DirconTrajectory& dircon_trajectory =
      DirconTrajectory(FLAGS_folder_path + FLAGS_traj_name);
  const LcmTrajectory& processed_trajs =
      LcmTrajectory(FLAGS_folder_path + FLAGS_traj_name + "_processed");

  const LcmTrajectory::Trajectory lcm_com_traj =
      processed_trajs.GetTrajectory("center_of_mass_trajectory");
  const LcmTrajectory::Trajectory lcm_l_foot_traj =
      processed_trajs.GetTrajectory("left_foot_trajectory");
  const LcmTrajectory::Trajectory lcm_r_foot_traj =
      processed_trajs.GetTrajectory("right_foot_trajectory");
  const LcmTrajectory::Trajectory lcm_pelvis_rot_traj =
      processed_trajs.GetTrajectory("pelvis_rot_trajectory");

  std::cout << "Loading output trajectories: " << std::endl;
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
  pelvis_rot_trajectory = PiecewisePolynomial<double>::FirstOrderHold(
      lcm_pelvis_rot_traj.time_vector,
      lcm_pelvis_rot_traj.datapoints.topRows(4));

  // For the time-based FSM
  //  double flight_time = FLAGS_delay_time +
  //  dircon_trajectory.GetStateBreaks(1)(0); double land_time =
  //  FLAGS_delay_time + dircon_trajectory.GetStateBreaks(2)(0);
  //  std::vector<double> transition_times = {FLAGS_delay_time, flight_time,
  //                                          land_time};
  std::vector<double> transition_times = {0.0, FLAGS_delay_time,
                                          FLAGS_delay_time + 300.0,
                                          FLAGS_delay_time + 600.0};

  Vector3d support_center_offset;
  support_center_offset << gains.x_offset, 0.0, 0.0;
  std::vector<double> breaks = com_traj.get_segment_times();
  VectorXd breaks_vector = Eigen::Map<VectorXd>(breaks.data(), breaks.size());
  MatrixXd offset_points = support_center_offset.replicate(1, breaks.size());
  PiecewisePolynomial<double> offset_traj =
      PiecewisePolynomial<double>::ZeroOrderHold(breaks_vector, offset_points);
  com_traj = com_traj + offset_traj;

  /**** Initialize all the leaf systems ****/
  drake::lcm::DrakeLcm lcm;

  auto state_receiver =
      builder.AddSystem<systems::RobotOutputReceiver>(plant_w_springs);
  auto com_traj_generator = builder.AddSystem<COMTrajGenerator>(
      plant_w_springs, context_w_spr.get(), com_traj, feet_contact_points,
      FLAGS_delay_time);
  auto l_foot_traj_generator = builder.AddSystem<FlightFootTrajGenerator>(
      plant_w_springs, context_w_spr.get(), "hip_left", true, l_foot_trajectory,
      FLAGS_delay_time);
  auto r_foot_traj_generator = builder.AddSystem<FlightFootTrajGenerator>(
      plant_w_springs, context_w_spr.get(), "hip_right", false,
      r_foot_trajectory, FLAGS_delay_time);
  auto pelvis_rot_traj_generator =
      builder.AddSystem<PelvisOrientationTrajGenerator>(
          pelvis_rot_trajectory, "pelvis_rot_tracking_data", FLAGS_delay_time);
  auto fsm = builder.AddSystem<JumpingEventFsm>(
      plant_w_springs, transition_times, FLAGS_contact_based_fsm,
      FLAGS_transition_delay, (osc_jump::FSM_STATE)FLAGS_init_fsm_state);
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
          "OSC_DEBUG_JUMPING", &lcm, TriggerTypeSet({TriggerType::kForced})));
  auto controller_switch_receiver = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_controller_switch>("INPUT_SWITCH",
                                                                 &lcm));

  LcmSubscriberSystem* contact_results_sub = nullptr;
  if (FLAGS_simulator == "DRAKE") {
    contact_results_sub = builder.AddSystem(
        LcmSubscriberSystem::Make<drake::lcmt_contact_results_for_viz>(
            "CASSIE_CONTACT_DRAKE", &lcm));
  } else if (FLAGS_simulator == "MUJOCO") {
    contact_results_sub = builder.AddSystem(
        LcmSubscriberSystem::Make<drake::lcmt_contact_results_for_viz>(
            "CASSIE_CONTACT_MUJOCO", &lcm));
  } else if (FLAGS_simulator == "DISPATCHER") {
    contact_results_sub = builder.AddSystem(
        LcmSubscriberSystem::Make<drake::lcmt_contact_results_for_viz>(
            "CASSIE_CONTACT_FOR_FSM_DISPATCHER", &lcm));
    // TODO(yangwill): Add PR for GM contact observer, currently in
    // gm_contact_estimator branch
  } else {
    std::cerr << "Unknown simulator type!" << std::endl;
  }

  /**** OSC setup ****/
  // Cost
  MatrixXd Q_accel = gains.w_accel * MatrixXd::Identity(nv, nv);
  osc->SetAccelerationCostForAllJoints(Q_accel);
  // Soft constraint on contacts
  double w_contact_relax = gains.w_soft_constraint;
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
  vector<osc_jump::FSM_STATE> stance_modes = {osc_jump::BALANCE,
                                              osc_jump::CROUCH, osc_jump::LAND};
  for (auto mode : stance_modes) {
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
  //  MatrixXd W_com = MatrixXd::Identity(3, 3);
  //  W_com(0, 0) = 2000;
  //  W_com(1, 1) = 200;
  //  W_com(2, 2) = 2000;
  //  MatrixXd K_p_com = 64 * MatrixXd::Identity(3, 3);
  //  MatrixXd K_d_com = 16 * MatrixXd::Identity(3, 3);
  ComTrackingData com_tracking_data("com_traj", K_p_com, K_d_com, W_com,
                                    plant_w_springs, plant_wo_springs);
  for (auto mode : stance_modes) {
    com_tracking_data.AddStateToTrack(mode);
  }
  osc->AddTrackingData(&com_tracking_data);

  // Feet tracking
  //  MatrixXd W_swing_foot = 1 * MatrixXd::Identity(3, 3);
  //  W_swing_foot(0, 0) = 1000;
  //  W_swing_foot(1, 1) = 1000;
  //  W_swing_foot(2, 2) = 1000;
  //  MatrixXd K_p_sw_ft = 36 * MatrixXd::Identity(3, 3);
  //  MatrixXd K_d_sw_ft = 12 * MatrixXd::Identity(3, 3);

  TransTaskSpaceTrackingData left_foot_tracking_data(
      "l_foot_traj", K_p_flight_foot, K_d_flight_foot, W_flight_foot,
      plant_w_springs, plant_wo_springs);
  TransTaskSpaceTrackingData right_foot_tracking_data(
      "r_foot_traj", K_p_flight_foot, K_d_flight_foot, W_flight_foot,
      plant_w_springs, plant_wo_springs);
  left_foot_tracking_data.AddStateAndPointToTrack(osc_jump::FLIGHT, "toe_left");
  right_foot_tracking_data.AddStateAndPointToTrack(osc_jump::FLIGHT,
                                                   "toe_right");

  // Pelvis orientation tracking
  //  double w_pelvis_balance = 20;
  //  double w_heading = 10;
  //  double k_p_pelvis_balance = 16;  // 100
  //  double k_d_pelvis_balance = 8;   // 80
  //  double k_p_heading = 16;         // 50
  //  double k_d_heading = 8;          // 40
  //  Matrix3d W_pelvis = w_pelvis_balance * MatrixXd::Identity(3, 3);
  //  W_pelvis(2, 2) = w_heading;
  //  Matrix3d K_p_pelvis = k_p_pelvis_balance * 2 * MatrixXd::Identity(3, 3);
  //  K_p_pelvis(2, 2) = k_p_heading;
  //  Matrix3d K_d_pelvis = k_d_pelvis_balance * MatrixXd::Identity(3, 3);
  //  K_d_pelvis(2, 2) = k_d_heading;
  RotTaskSpaceTrackingData pelvis_rot_tracking_data(
      "pelvis_rot_tracking_data", K_p_pelvis, K_d_pelvis, W_pelvis,
      plant_w_springs, plant_wo_springs);

  for (auto mode : stance_modes) {
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
  builder.Connect(controller_switch_receiver->get_output_port(),
                  fsm->get_switch_input_port());

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
