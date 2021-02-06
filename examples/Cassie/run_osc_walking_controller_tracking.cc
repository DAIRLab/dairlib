#include <drake/multibody/parsing/parser.h>
#include <gflags/gflags.h>

#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "examples/Cassie/cassie_utils.h"
#include "examples/Cassie/osc/osc_walking_gains.h"
#include "examples/Cassie/osc/swing_toe_traj_generator.h"
#include "examples/Cassie/osc_jump/pelvis_orientation_traj_generator.h"
#include "examples/Cassie/osc_walk/com_traj_generator.h"
#include "examples/Cassie/osc_walk/swing_foot_traj_generator.h"
#include "examples/Cassie/osc_walk/walking_event_based_fsm.h"
#include "lcm/dircon_saved_trajectory.h"
#include "lcm/lcm_trajectory.h"
#include "systems/controllers/osc/operational_space_control.h"
#include "systems/controllers/osc/osc_tracking_data.h"
#include "systems/controllers/time_based_fsm.h"
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
using examples::osc_jump::PelvisOrientationTrajGenerator;
using examples::osc_walk::COMTrajGenerator;
using examples::osc_walk::SwingFootTrajGenerator;
using examples::osc_walk::WalkingEventFsm;
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
DEFINE_string(folder_path, "examples/Cassie/saved_trajectories/",
              "Folder path for where the trajectory names are stored");
DEFINE_string(gains_filename, "examples/Cassie/osc/osc_walking_gains.yaml",
              "Filepath containing gains");

DEFINE_string(traj_name, "", "File to load saved trajectories from");
DEFINE_string(
    simulator, "DRAKE",
    "Simulator used, important for determining how to interpret "
    "contact information. Other options include MUJOCO and possibly GAZEBO.");
DEFINE_bool(add_noise, false,
            "Whether to add gaussian noise to state "
            "inputted to controller");
//DEFINE_int32(init_fsm_state, osc_walk::DOUBLE_L_LO, "Initial state of the FSM");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Read-in the parameters
  OSCWalkingGains gains;
  const YAML::Node& root =
      YAML::LoadFile(FindResourceOrThrow(FLAGS_gains_filename));
  drake::yaml::YamlReadArchive(root).Accept(&gains);

  // Build the controller diagram
  DiagramBuilder<double> builder;

  // Built the Cassie MBPs
  drake::multibody::MultibodyPlant<double> plant_w_spr(0.0);
  addCassieMultibody(&plant_w_spr, nullptr, true,
                     "examples/Cassie/urdf/cassie_v2.urdf",
                     true /*spring model*/, false /*loop closure*/);
  plant_w_spr.Finalize();

  auto context_w_spr = plant_w_spr.CreateDefaultContext();
  auto context_wo_spr = plant_w_spr.CreateDefaultContext();

  int nq = plant_w_spr.num_positions();
  int nv = plant_w_spr.num_velocities();
  int nx = nq + nv;
  int n_modes = 3;
  // Create maps for joints
  map<string, int> pos_map = multibody::makeNameToPositionsMap(plant_w_spr);
  map<string, int> vel_map = multibody::makeNameToVelocitiesMap(plant_w_spr);
  map<string, int> act_map = multibody::makeNameToActuatorsMap(plant_w_spr);

  auto left_toe = LeftToeFront(plant_w_spr);
  auto left_heel = LeftToeRear(plant_w_spr);
  auto right_toe = RightToeFront(plant_w_spr);
  auto right_heel = RightToeRear(plant_w_spr);

  /**** Get trajectory from optimization ****/
  const DirconTrajectory& dircon_trajectory = DirconTrajectory(
      FindResourceOrThrow(FLAGS_folder_path + FLAGS_traj_name));
  const LcmTrajectory& processed_trajs = LcmTrajectory(
      FindResourceOrThrow(FLAGS_folder_path + FLAGS_traj_name + "_processed"));

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
  PiecewisePolynomial<double> l_foot_trajectory =
      PiecewisePolynomial<double>::CubicHermite(
          lcm_l_foot_traj.time_vector, lcm_l_foot_traj.datapoints.topRows(3),
          lcm_l_foot_traj.datapoints.bottomRows(3));
  PiecewisePolynomial<double> r_foot_trajectory =
      PiecewisePolynomial<double>::CubicHermite(
          lcm_r_foot_traj.time_vector, lcm_r_foot_traj.datapoints.topRows(3),
          lcm_r_foot_traj.datapoints.bottomRows(3));
  PiecewisePolynomial<double> pelvis_rot_trajectory;
  pelvis_rot_trajectory = PiecewisePolynomial<double>::FirstOrderHold(
      lcm_pelvis_rot_traj.time_vector,
      lcm_pelvis_rot_traj.datapoints.topRows(4));
  PiecewisePolynomial<double> state_traj =
      dircon_trajectory.ReconstructStateTrajectory();

  // For the time-based FSM
  double total_time = state_traj.end_time();
  double r_td_time = 0.5 * state_traj.end_time();
  double l_td_time = 0.5 * state_traj.end_time();
  std::vector<double> transition_times = {r_td_time, l_td_time};
  int left_stance_state = 0;
  int right_stance_state = 1;
  vector<int> fsm_states = {left_stance_state, right_stance_state};

  /**** Initialize all the leaf systems ****/
  drake::lcm::DrakeLcm lcm;

  double time_offset = total_time;
  auto state_receiver =
      builder.AddSystem<systems::RobotOutputReceiver>(plant_w_spr);
  auto pelvis_traj_generator = builder.AddSystem<COMTrajGenerator>(
      plant_w_spr, context_w_spr.get(), com_traj);
  auto l_foot_traj_generator = builder.AddSystem<SwingFootTrajGenerator>(
      plant_w_spr, context_w_spr.get(), "toe_right", true, l_foot_trajectory,
      time_offset);
  auto r_foot_traj_generator = builder.AddSystem<SwingFootTrajGenerator>(
      plant_w_spr, context_w_spr.get(), "toe_left", false, r_foot_trajectory);
  auto pelvis_rot_traj_generator =
      builder.AddSystem<PelvisOrientationTrajGenerator>(pelvis_rot_trajectory,
                                                        "pelvis_rot_traj");
  auto fsm = builder.AddSystem<systems::TimeBasedFiniteStateMachine>(
      plant_w_spr, fsm_states, transition_times, 0.0, gains.impact_threshold);
  auto command_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          FLAGS_channel_u, &lcm, TriggerTypeSet({TriggerType::kForced})));
  auto command_sender =
      builder.AddSystem<systems::RobotCommandSender>(plant_w_spr);
  auto osc = builder.AddSystem<systems::controllers::OperationalSpaceControl>(
      plant_w_spr, plant_w_spr, context_w_spr.get(), context_wo_spr.get(),
      true); /*print_tracking_info*/
  auto osc_debug_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_osc_output>(
          "OSC_DEBUG_WALKING", &lcm, TriggerTypeSet({TriggerType::kForced})));

  /*** OSC setup ***/
  // Cost
  MatrixXd Q_accel = gains.w_accel * MatrixXd::Identity(nv, nv);
  osc->SetAccelerationCostForAllJoints(Q_accel);
  // Soft constraint on contacts
  osc->SetWeightOfSoftContactConstraint(gains.w_soft_constraint);

  // Contact information for OSC
  osc->SetContactFriction(gains.mu);

  auto left_toe_evaluator = multibody::WorldPointEvaluator(
      plant_w_spr, left_toe.first, left_toe.second, Matrix3d::Identity(),
      Vector3d::Zero(), {1, 2});
  auto left_heel_evaluator = multibody::WorldPointEvaluator(
      plant_w_spr, left_heel.first, left_heel.second, Matrix3d::Identity(),
      Vector3d::Zero(), {0, 1, 2});
  auto right_toe_evaluator = multibody::WorldPointEvaluator(
      plant_w_spr, right_toe.first, right_toe.second, Matrix3d::Identity(),
      Vector3d::Zero(), {1, 2});
  auto right_heel_evaluator = multibody::WorldPointEvaluator(
      plant_w_spr, right_heel.first, right_heel.second, Matrix3d::Identity(),
      Vector3d::Zero(), {0, 1, 2});
  vector<osc_walk::FSM_STATE> all_modes = {osc_walk::LEFT_STANCE,
                                           osc_walk::RIGHT_STANCE};

  /*** Contact Constraints ***/
  osc->AddStateAndContactPoint(osc_walk::RIGHT_STANCE, &right_toe_evaluator);
  osc->AddStateAndContactPoint(osc_walk::RIGHT_STANCE, &right_heel_evaluator);
  osc->AddStateAndContactPoint(osc_walk::LEFT_STANCE, &left_toe_evaluator);
  osc->AddStateAndContactPoint(osc_walk::LEFT_STANCE, &left_heel_evaluator);

  /*** Four bar constraint ***/
  multibody::KinematicEvaluatorSet<double> evaluators(plant_w_spr);
  auto left_loop = LeftLoopClosureEvaluator(plant_w_spr);
  auto right_loop = RightLoopClosureEvaluator(plant_w_spr);
  evaluators.add_evaluator(&left_loop);
  evaluators.add_evaluator(&right_loop);
  osc->AddKinematicConstraint(&evaluators);

  /*** Tracking Data for OSC ***/
  // Pelvis tracking
  TransTaskSpaceTrackingData pelvis_trans_tracking_data(
      "com_traj", gains.K_p_com, gains.K_d_com, gains.W_com, plant_w_spr,
      plant_w_spr);

  for (auto mode : all_modes) {
    pelvis_trans_tracking_data.AddStateAndPointToTrack(mode, "pelvis");
  }

  // Feet tracking
  TransTaskSpaceTrackingData left_foot_tracking_data(
      "l_foot_traj", gains.K_p_swing_foot, gains.K_d_swing_foot,
      gains.W_swing_foot, plant_w_spr, plant_w_spr);
  TransTaskSpaceTrackingData right_foot_tracking_data(
      "r_foot_traj", gains.K_p_swing_foot, gains.K_d_swing_foot,
      gains.W_swing_foot, plant_w_spr, plant_w_spr);
  right_foot_tracking_data.AddStateAndPointToTrack(osc_walk::LEFT_STANCE,
                                                   "toe_right");
  left_foot_tracking_data.AddStateAndPointToTrack(osc_walk::RIGHT_STANCE,
                                                  "toe_left");

  // Pelvis rotation tracking (pitch and roll)
  RotTaskSpaceTrackingData pelvis_rot_tracking_data(
      "pelvis_rot_traj", gains.K_p_pelvis_balance, gains.K_d_pelvis_balance,
      gains.W_pelvis_balance, plant_w_spr, plant_w_spr);
  pelvis_rot_tracking_data.AddFrameToTrack("pelvis");

  // Swing toe joint trajectory
  vector<std::pair<const Vector3d, const Frame<double>&>> left_foot_points = {
      left_heel, left_toe};
  vector<std::pair<const Vector3d, const Frame<double>&>> right_foot_points = {
      right_heel, right_toe};
  auto left_toe_angle_traj_gen =
      builder.AddSystem<cassie::osc::SwingToeTrajGenerator>(
          plant_w_spr, context_w_spr.get(), pos_map["toe_left"],
          left_foot_points, "left_toe_angle_traj");
  auto right_toe_angle_traj_gen =
      builder.AddSystem<cassie::osc::SwingToeTrajGenerator>(
          plant_w_spr, context_w_spr.get(), pos_map["toe_right"],
          right_foot_points, "right_toe_angle_traj");
  // Swing toe tracking
  JointSpaceTrackingData swing_toe_traj_left(
      "left_toe_angle_traj", gains.K_p_swing_toe, gains.K_d_swing_toe,
      gains.W_swing_toe, plant_w_spr, plant_w_spr);
  JointSpaceTrackingData swing_toe_traj_right(
      "right_toe_angle_traj", gains.K_p_swing_toe, gains.K_d_swing_toe,
      gains.W_swing_toe, plant_w_spr, plant_w_spr);
  swing_toe_traj_right.AddStateAndJointToTrack(left_stance_state, "toe_right",
                                               "toe_rightdot");
  swing_toe_traj_left.AddStateAndJointToTrack(right_stance_state, "toe_left",
                                              "toe_leftdot");

  osc->AddTrackingData(&swing_toe_traj_left);
  osc->AddTrackingData(&swing_toe_traj_right);
  osc->AddTrackingData(&pelvis_trans_tracking_data);
  osc->AddTrackingData(&left_foot_tracking_data);
  osc->AddTrackingData(&right_foot_tracking_data);
  osc->AddTrackingData(&pelvis_rot_tracking_data);

  // Build OSC problem
  osc->Build();
  std::cout << "Built OSC" << std::endl;

  // State receiver connections (Connected through LCM driven loop)
  drake::systems::LeafSystem<double>* controller_state_input = state_receiver;

  /*** Connect ports ***/
  // OSC connections
  builder.Connect(fsm->get_output_port(0), osc->get_fsm_input_port());
  builder.Connect(controller_state_input->get_output_port(0),
                  osc->get_robot_output_input_port());
  builder.Connect(pelvis_traj_generator->get_output_port(0),
                  osc->get_tracking_data_input_port("com_traj"));
  builder.Connect(l_foot_traj_generator->get_output_port(0),
                  osc->get_tracking_data_input_port("l_foot_traj"));
  builder.Connect(r_foot_traj_generator->get_output_port(0),
                  osc->get_tracking_data_input_port("r_foot_traj"));
  builder.Connect(pelvis_rot_traj_generator->get_output_port(0),
                  osc->get_tracking_data_input_port("pelvis_rot_traj"));
  builder.Connect(left_toe_angle_traj_gen->get_output_port(0),
                  osc->get_tracking_data_input_port("left_toe_angle_traj"));
  builder.Connect(right_toe_angle_traj_gen->get_output_port(0),
                  osc->get_tracking_data_input_port("right_toe_angle_traj"));
  builder.Connect(fsm->get_output_port_impact(),
                  osc->get_near_impact_input_port());

  // FSM connections
  builder.Connect(controller_state_input->get_output_port(0),
                  fsm->get_input_port_state());

  // Trajectory generator connections
  builder.Connect(controller_state_input->get_output_port(0),
                  pelvis_traj_generator->get_state_input_port());
  builder.Connect(controller_state_input->get_output_port(0),
                  l_foot_traj_generator->get_state_input_port());
  builder.Connect(controller_state_input->get_output_port(0),
                  r_foot_traj_generator->get_state_input_port());
  builder.Connect(fsm->get_output_port(0),
                  pelvis_traj_generator->get_fsm_input_port());
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
  owned_diagram->set_name(("osc walking controller"));

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
