#include <drake/multibody/parsing/parser.h>
#include <gflags/gflags.h>

#include "common/find_resource.h"
#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "examples/Cassie/cassie_utils.h"
#include "examples/Cassie/osc/swing_toe_traj_generator.h"
#include "examples/Cassie/osc_jump/basic_trajectory_passthrough.h"
#include "examples/Cassie/osc_jump/flight_foot_traj_generator.h"
#include "examples/Cassie/osc_jump/pelvis_trans_traj_generator.h"
#include "examples/Cassie/osc_jump/toe_angle_traj_generator.h"
#include "examples/Cassie/osc_run/joint_space_running_gains.h"
#include "examples/Cassie/osc_run/osc_running_gains.h"
#include "examples/impact_invariant_control/impact_aware_time_based_fsm.h"
#include "lcm/dircon_saved_trajectory.h"
#include "lcm/lcm_trajectory.h"
#include "multibody/kinematic/fixed_joint_evaluator.h"
#include "multibody/multibody_utils.h"
#include "systems/controllers/osc/joint_space_tracking_data.h"
#include "systems/controllers/osc/operational_space_control.h"
#include "systems/controllers/osc/osc_tracking_data.h"
#include "systems/controllers/osc/trans_space_tracking_data.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/robot_lcm_systems.h"
#include "yaml-cpp/yaml.h"

#include "drake/common/yaml/yaml_read_archive.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"

namespace dairlib {

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
using drake::systems::TriggerTypeSet;
using drake::trajectories::PiecewisePolynomial;
using examples::osc_jump::BasicTrajectoryPassthrough;
using examples::osc_jump::FlightFootTrajGenerator;
using examples::osc_jump::PelvisTransTrajGenerator;
using multibody::FixedJointEvaluator;
using osc::SwingToeTrajGenerator;
using systems::controllers::JointSpaceTrackingData;
using systems::controllers::TransTaskSpaceTrackingData;

namespace examples {

DEFINE_string(channel_x, "CASSIE_STATE_SIMULATION",
              "The name of the channel which receives state");
DEFINE_string(channel_u, "CASSIE_INPUT",
              "The name of the channel which publishes command");
DEFINE_string(folder_path, "examples/Cassie/saved_trajectories/",
              "Folder path for where the trajectory names are stored");
DEFINE_string(traj_name, "running_0.00",
              "File to load saved trajectories from");
DEFINE_string(gains_filename,
              "examples/Cassie/osc_run/joint_space_running_gains.yaml",
              "Filepath containing gains");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Build the controller diagram
  DiagramBuilder<double> builder;

  // Built the Cassie MBPs
  drake::multibody::MultibodyPlant<double> plant(0.0);
  addCassieMultibody(&plant, nullptr, true,
                     "examples/Cassie/urdf/cassie_v2.urdf",
                     false /*spring model*/, false /*loop closure*/);
  drake::multibody::MultibodyPlant<double> plant_wo_spr(0.0);
  addCassieMultibody(&plant_wo_spr, nullptr, true,
                     "examples/Cassie/urdf/cassie_fixed_springs.urdf",
                     false /*spring model*/, false /*loop closure*/);
  plant.Finalize();
  plant_wo_spr.Finalize();

  auto plant_context = plant.CreateDefaultContext();

  // Get contact frames and position
  auto left_toe = LeftToeFront(plant);
  auto left_heel = LeftToeRear(plant);
  auto right_toe = RightToeFront(plant);
  auto right_heel = RightToeRear(plant);

  int nv = plant.num_velocities();

  // Create maps for joints
  map<string, int> pos_map = multibody::makeNameToPositionsMap(plant);
  map<string, int> vel_map = multibody::makeNameToVelocitiesMap(plant);
  map<string, int> act_map = multibody::makeNameToActuatorsMap(plant);

  map<string, int> pos_map_wo_spr =
      multibody::makeNameToPositionsMap(plant_wo_spr);

  std::vector<std::pair<const Vector3d, const drake::multibody::Frame<double>&>>
      feet_contact_points = {left_toe, right_toe, left_heel, right_heel};

  /**** Get trajectory from optimization ****/
  const DirconTrajectory& dircon_trajectory = DirconTrajectory(
      FindResourceOrThrow(FLAGS_folder_path + FLAGS_traj_name));

  PiecewisePolynomial<double> state_traj =
      dircon_trajectory.ReconstructStateTrajectory();
  state_traj.ConcatenateInTime(
      dircon_trajectory.ReconstructMirrorStateTrajectory(
          state_traj.end_time()));

  /**** OSC Gains ****/
  OSCGains gains{};
  const YAML::Node& root =
      YAML::LoadFile(FindResourceOrThrow(FLAGS_gains_filename));
  drake::yaml::YamlReadArchive::Options yaml_options;
  yaml_options.allow_yaml_with_no_cpp = true;
  drake::yaml::YamlReadArchive(root, yaml_options).Accept(&gains);

  /**** FSM and contact mode configuration ****/
  int left_stance_state = 0;
  int right_stance_state = 1;
  int air_phase = 2;
  double left_support_duration = dircon_trajectory.GetStateBreaks(1)(0) * 2;
  double right_support_duration = left_support_duration;
  double air_phase_duration = dircon_trajectory.GetStateBreaks(2)(0) -
                              dircon_trajectory.GetStateBreaks(1)(0);
  vector<int> fsm_states = {left_stance_state, air_phase, right_stance_state,
                            air_phase, left_stance_state};
  vector<double> state_durations = {
      left_support_duration / 2, air_phase_duration, right_support_duration,
      air_phase_duration, left_support_duration / 2};

  auto fsm = builder.AddSystem<ImpactTimeBasedFiniteStateMachine>(
      plant, fsm_states, state_durations, 0.0,
      gains.impact_threshold);

  /**** Initialize all the leaf systems ****/
  drake::lcm::DrakeLcm lcm("udpm://239.255.76.67:7667?ttl=0");

  auto state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(plant);
  auto command_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          FLAGS_channel_u, &lcm, TriggerTypeSet({TriggerType::kForced})));
  auto command_sender = builder.AddSystem<systems::RobotCommandSender>(plant);
  auto osc = builder.AddSystem<systems::controllers::OperationalSpaceControl>(
      plant, plant, plant_context.get(), plant_context.get(), true);
  auto osc_debug_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_osc_output>(
          "OSC_DEBUG_RUNNING", &lcm, TriggerTypeSet({TriggerType::kForced})));

  /**** OSC setup ****/
  // Cost
  MatrixXd Q_accel = gains.w_accel * MatrixXd::Identity(nv, nv);
  osc->SetAccelerationCostForAllJoints(Q_accel);
  // Soft constraint on contacts
  double w_contact_relax = gains.w_soft_constraint;
  osc->SetWeightOfSoftContactConstraint(w_contact_relax);

  // Contact information for OSC
  osc->SetContactFriction(gains.mu);

  auto left_toe_evaluator = multibody::WorldPointEvaluator(
      plant, left_toe.first, left_toe.second, Matrix3d::Identity(),
      Vector3d::Zero(), {1, 2});
  auto left_heel_evaluator = multibody::WorldPointEvaluator(
      plant, left_heel.first, left_heel.second, Matrix3d::Identity(),
      Vector3d::Zero(), {0, 1, 2});
  auto right_toe_evaluator = multibody::WorldPointEvaluator(
      plant, right_toe.first, right_toe.second, Matrix3d::Identity(),
      Vector3d::Zero(), {1, 2});
  auto right_heel_evaluator = multibody::WorldPointEvaluator(
      plant, right_heel.first, right_heel.second, Matrix3d::Identity(),
      Vector3d::Zero(), {0, 1, 2});

  osc->AddStateAndContactPoint(0, &left_toe_evaluator);
  osc->AddStateAndContactPoint(0, &left_heel_evaluator);
  osc->AddStateAndContactPoint(1, &right_toe_evaluator);
  osc->AddStateAndContactPoint(1, &right_heel_evaluator);

  multibody::KinematicEvaluatorSet<double> evaluators(plant);
  auto left_loop = LeftLoopClosureEvaluator(plant);
  auto right_loop = RightLoopClosureEvaluator(plant);
  evaluators.add_evaluator(&left_loop);
  evaluators.add_evaluator(&right_loop);

  // Fix the springs in the dynamics
  auto pos_idx_map = multibody::makeNameToPositionsMap(plant);
  auto vel_idx_map = multibody::makeNameToVelocitiesMap(plant);
  auto left_fixed_knee_spring =
      FixedJointEvaluator(plant, pos_idx_map.at("knee_joint_left"),
                          vel_idx_map.at("knee_joint_leftdot"), 0);
  auto right_fixed_knee_spring =
      FixedJointEvaluator(plant, pos_idx_map.at("knee_joint_right"),
                          vel_idx_map.at("knee_joint_rightdot"), 0);
  auto left_fixed_ankle_spring =
      FixedJointEvaluator(plant, pos_idx_map.at("ankle_spring_joint_left"),
                          vel_idx_map.at("ankle_spring_joint_leftdot"), 0);
  auto right_fixed_ankle_spring =
      FixedJointEvaluator(plant, pos_idx_map.at("ankle_spring_joint_right"),
                          vel_idx_map.at("ankle_spring_joint_rightdot"), 0);
  evaluators.add_evaluator(&left_fixed_knee_spring);
  evaluators.add_evaluator(&right_fixed_knee_spring);
  evaluators.add_evaluator(&left_fixed_ankle_spring);
  evaluators.add_evaluator(&right_fixed_ankle_spring);

  osc->AddKinematicConstraint(&evaluators);

  std::vector<BasicTrajectoryPassthrough*> joint_trajs;
  std::vector<std::shared_ptr<JointSpaceTrackingData>> joint_tracking_data_vec;
  /**** Tracking Data *****/
  std::cout << "Creating joint space controller. " << std::endl;
  /**** Convert the gains from the yaml struct to Eigen Matrices ****/
  JointSpaceRunningGains joint_space_gains;
  drake::yaml::YamlReadArchive(root).Accept(&joint_space_gains);

  std::vector<std::string> actuated_joint_names = {
      "hip_roll_left",  "hip_roll_right",  "hip_yaw_left", "hip_yaw_right",
      "hip_pitch_left", "hip_pitch_right", "knee_left",    "knee_right",
      "toe_left",       "toe_right"};

  for (int joint_idx = 0; joint_idx < actuated_joint_names.size();
       ++joint_idx) {
    string joint_name = actuated_joint_names[joint_idx];
    MatrixXd W = joint_space_gains.JointW[joint_idx] * MatrixXd::Identity(1, 1);
    MatrixXd K_p =
        joint_space_gains.JointKp[joint_idx] * MatrixXd::Identity(1, 1);
    MatrixXd K_d =
        joint_space_gains.JointKd[joint_idx] * MatrixXd::Identity(1, 1);
    joint_tracking_data_vec.push_back(std::make_shared<JointSpaceTrackingData>(
        joint_name + "_traj", K_p, K_d, W, plant, plant));
    joint_tracking_data_vec[joint_idx]->AddJointToTrack(joint_name,
                                                        joint_name + "dot");
    auto joint_traj = dircon_trajectory.ReconstructJointTrajectory(
        pos_map_wo_spr[joint_name]);
    auto mirror_joint_traj = dircon_trajectory.ReconstructMirrorJointTrajectory(
        pos_map_wo_spr[joint_name]);
    mirror_joint_traj.shiftRight(joint_traj.end_time());
    joint_traj.ConcatenateInTime(mirror_joint_traj);
    auto joint_traj_generator = builder.AddSystem<BasicTrajectoryPassthrough>(
        joint_traj, joint_name + "_traj");
    joint_trajs.push_back(joint_traj_generator);
    osc->AddTrackingData(joint_tracking_data_vec[joint_idx].get());

    builder.Connect(joint_trajs[joint_idx]->get_output_port(),
                    osc->get_tracking_data_input_port(joint_name + "_traj"));
  }

  // Build OSC problem
  osc->Build();
  std::cout << "Built OSC" << std::endl;

  /*****Connect ports*****/

  // OSC connections
  builder.Connect(fsm->get_output_port_fsm(), osc->get_fsm_input_port());
  builder.Connect(fsm->get_output_port_impact(),
                  osc->get_near_impact_input_port());
  builder.Connect(state_receiver->get_output_port(0),
                  osc->get_robot_output_input_port());
  builder.Connect(fsm->get_output_port_clock(), osc->get_clock_input_port());
  // FSM connections
  builder.Connect(state_receiver->get_output_port(0),
                  fsm->get_input_port_state());

  // Publisher connections
  builder.Connect(osc->get_osc_output_port(),
                  command_sender->get_input_port(0));
  builder.Connect(command_sender->get_output_port(0),
                  command_pub->get_input_port());
  builder.Connect(osc->get_osc_debug_port(), osc_debug_pub->get_input_port());

  // Run lcm-driven simulation
  // Create the diagram
  auto owned_diagram = builder.Build();
  owned_diagram->set_name(("osc_running_controller"));

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
