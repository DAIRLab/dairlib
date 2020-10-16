#include <gflags/gflags.h>

#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_target_standing_height.hpp"
#include "examples/Cassie/cassie_utils.h"
#include "examples/Cassie/osc/standing_com_traj.h"
#include "examples/Cassie/osc/standing_pelvis_orientation_traj.h"
#include "multibody/kinematic/kinematic_evaluator_set.h"
#include "multibody/multibody_utils.h"
#include "systems/controllers/osc/operational_space_control.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/robot_lcm_systems.h"
#include "yaml-cpp/yaml.h"

#include "drake/common/yaml/yaml_read_archive.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"

namespace dairlib {

using std::cout;
using std::endl;

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

using systems::controllers::ComTrackingData;
using systems::controllers::JointSpaceTrackingData;
using systems::controllers::RotTaskSpaceTrackingData;
using systems::controllers::TransTaskSpaceTrackingData;

DEFINE_string(channel_x, "CASSIE_STATE_SIMULATION",
              "LCM channel for receiving state. "
              "Use CASSIE_STATE_SIMULATION to get state from simulator, and "
              "use CASSIE_STATE_DISPATCHER to get state from state estimator");
DEFINE_string(channel_u, "CASSIE_INPUT",
              "The name of the channel which publishes command");
DEFINE_string(
    cassie_out_channel, "CASSIE_OUTPUT_ECHO",
    "The name of the channel to receive the cassie out structure from.");
DEFINE_bool(print_osc, false, "whether to print the osc debug message or not");
DEFINE_double(cost_weight_multiplier, 0.001,
              "A cosntant times with cost weight of OSC traj tracking");
DEFINE_double(height, .8, "The initial COM height (m)");
DEFINE_string(gains_filename, "examples/Cassie/osc/osc_standing_gains.yaml",
              "Filepath containing gains");

// Currently the controller runs at the rate between 500 Hz and 200 Hz, so the
// publish rate of the robot state needs to be less than 500 Hz. Otherwise, the
// performance seems to degrade due to this. (Recommended publish rate: 200 Hz)
// Maybe we need to update the lcm driven loop to clear the queue of lcm message
// if it's more than one message?

struct OSCStandingGains {
  int rows;
  int cols;
  double w_input;
  double w_accel;
  double w_soft_constraint;
  double HipYawKp;
  double HipYawKd;
  double HipYawW;
  std::vector<double> CoMKp;
  std::vector<double> CoMKd;
  std::vector<double> PelvisRotKp;
  std::vector<double> PelvisRotKd;
  std::vector<double> CoMW;
  std::vector<double> PelvisW;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(rows));
    a->Visit(DRAKE_NVP(cols));
    a->Visit(DRAKE_NVP(w_input));
    a->Visit(DRAKE_NVP(w_accel));
    a->Visit(DRAKE_NVP(w_soft_constraint));
    a->Visit(DRAKE_NVP(CoMKp));
    a->Visit(DRAKE_NVP(CoMKd));
    a->Visit(DRAKE_NVP(PelvisRotKp));
    a->Visit(DRAKE_NVP(PelvisRotKd));
    a->Visit(DRAKE_NVP(HipYawKp));
    a->Visit(DRAKE_NVP(HipYawKd));
    a->Visit(DRAKE_NVP(CoMW));
    a->Visit(DRAKE_NVP(PelvisW));
    a->Visit(DRAKE_NVP(HipYawW));
  }
};

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Build Cassie MBP
  drake::multibody::MultibodyPlant<double> plant_w_springs(0.0);
  addCassieMultibody(&plant_w_springs, nullptr, true /*floating base*/,
                     "examples/Cassie/urdf/cassie_v2.urdf",
                     true /*spring model*/, false /*loop closure*/);
  plant_w_springs.Finalize();
  // Build fix-spring Cassie MBP
  drake::multibody::MultibodyPlant<double> plant_wo_springs(0.0);
  addCassieMultibody(&plant_wo_springs, nullptr, true,
                     "examples/Cassie/urdf/cassie_fixed_springs.urdf", false,
                     false);
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

  // Build the controller diagram
  DiagramBuilder<double> builder;

  drake::lcm::DrakeLcm lcm_local("udpm://239.255.76.67:7667?ttl=0");
  OSCStandingGains gains;
  const YAML::Node& root =
      YAML::LoadFile(FindResourceOrThrow(FLAGS_gains_filename));
  drake::yaml::YamlReadArchive(root).Accept(&gains);

  MatrixXd K_p_com = Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      gains.CoMKp.data(), gains.rows, gains.cols);
  MatrixXd K_d_com = Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      gains.CoMKd.data(), gains.rows, gains.cols);
  MatrixXd K_p_pelvis = Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      gains.PelvisRotKp.data(), gains.rows, gains.cols);
  MatrixXd K_d_pelvis = Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      gains.PelvisRotKd.data(), gains.rows, gains.cols);
  MatrixXd K_p_hip_yaw = gains.HipYawKp * MatrixXd::Identity(1, 1);
  MatrixXd K_d_hip_yaw = gains.HipYawKd * MatrixXd::Identity(1, 1);
  MatrixXd W_com = Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      gains.CoMW.data(), gains.rows, gains.cols);
  MatrixXd W_pelvis = Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      gains.PelvisW.data(), gains.rows, gains.cols);
  MatrixXd W_hip_yaw = gains.HipYawW * MatrixXd::Identity(1, 1);
  std::cout << "w input (not used): \n" << gains.w_input << std::endl;
  std::cout << "w accel: \n" << gains.w_accel << std::endl;
  std::cout << "w soft constraint: \n" << gains.w_soft_constraint << std::endl;
  std::cout << "COM Kp: \n" << K_p_com << std::endl;
  std::cout << "COM Kd: \n" << K_d_com << std::endl;
  std::cout << "Pelvis Rot Kp: \n" << K_p_pelvis << std::endl;
  std::cout << "Pelvis Rot Kd: \n" << K_d_pelvis << std::endl;
  std::cout << "COM W: \n" << W_com << std::endl;
  std::cout << "Pelvis W: \n" << W_pelvis << std::endl;

  // Create Lcm subsriber for lcmt_target_standing_height
  auto target_height_receiver = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_target_standing_height>(
          "TARGET_HEIGHT", &lcm_local));

  // Create state receiver.
  auto state_receiver =
      builder.AddSystem<systems::RobotOutputReceiver>(plant_w_springs);

  auto cassie_out_receiver =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_cassie_out>(
          FLAGS_cassie_out_channel, &lcm_local));

  // Create command sender.
  auto command_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          FLAGS_channel_u, &lcm_local, TriggerTypeSet({TriggerType::kForced})));
  auto command_sender =
      builder.AddSystem<systems::RobotCommandSender>(plant_w_springs);

  builder.Connect(command_sender->get_output_port(0),
                  command_pub->get_input_port());

  // Create osc debug sender.
  auto osc_debug_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_osc_output>(
          "OSC_DEBUG_STANDING", &lcm_local,
          TriggerTypeSet({TriggerType::kForced})));

  // Create desired center of mass traj
  std::vector<std::pair<const Vector3d, const drake::multibody::Frame<double>&>>
      feet_contact_points = {left_toe, left_heel, right_toe, right_heel};
  auto com_traj_generator = builder.AddSystem<cassie::osc::StandingComTraj>(
      plant_w_springs, context_w_spr.get(), feet_contact_points, FLAGS_height);
  auto pelvis_rot_traj_generator =
      builder.AddSystem<cassie::osc::StandingPelvisOrientationTraj>(
          plant_w_springs, context_w_spr.get(), feet_contact_points,
          "pelvis_rot_traj");
  builder.Connect(state_receiver->get_output_port(0),
                  com_traj_generator->get_input_port_state());
  builder.Connect(state_receiver->get_output_port(0),
                  pelvis_rot_traj_generator->get_input_port_state());
  builder.Connect(cassie_out_receiver->get_output_port(),
                  pelvis_rot_traj_generator->get_input_port_radio());
  builder.Connect(cassie_out_receiver->get_output_port(),
                  com_traj_generator->get_input_port_radio());
  builder.Connect(target_height_receiver->get_output_port(),
                  com_traj_generator->get_input_port_target_height());

  // Create Operational space control
  auto osc = builder.AddSystem<systems::controllers::OperationalSpaceControl>(
      plant_w_springs, plant_wo_springs, context_w_spr.get(),
      context_wo_spr.get(), false, FLAGS_print_osc);

  // Distance constraint
  multibody::KinematicEvaluatorSet<double> evaluators(plant_wo_springs);
  auto left_loop = LeftLoopClosureEvaluator(plant_wo_springs);
  auto right_loop = RightLoopClosureEvaluator(plant_wo_springs);
  evaluators.add_evaluator(&left_loop);
  evaluators.add_evaluator(&right_loop);
  osc->AddKinematicConstraint(&evaluators);
  // Soft constraint
  // We don't want w_contact_relax to be too big, cause we want tracking
  // error to be important
  double w_contact_relax = gains.w_soft_constraint;
  osc->SetWeightOfSoftContactConstraint(w_contact_relax);
  // Friction coefficient
  double mu = 0.8;
  osc->SetContactFriction(mu);
  // Add contact points (The position doesn't matter. It's not used in OSC)
  auto left_toe_evaluator = multibody::WorldPointEvaluator(
      plant_wo_springs, left_toe.first, left_toe.second, Matrix3d::Identity(),
      Vector3d::Zero(), {1, 2});
  osc->AddContactPoint(&left_toe_evaluator);
  auto left_heel_evaluator = multibody::WorldPointEvaluator(
      plant_wo_springs, left_heel.first, left_heel.second, Matrix3d::Identity(),
      Vector3d::Zero(), {0, 1, 2});
  osc->AddContactPoint(&left_heel_evaluator);
  auto right_toe_evaluator = multibody::WorldPointEvaluator(
      plant_wo_springs, right_toe.first, right_toe.second, Matrix3d::Identity(),
      Vector3d::Zero(), {1, 2});
  osc->AddContactPoint(&right_toe_evaluator);
  auto right_heel_evaluator = multibody::WorldPointEvaluator(
      plant_wo_springs, right_heel.first, right_heel.second,
      Matrix3d::Identity(), Vector3d::Zero(), {0, 1, 2});
  osc->AddContactPoint(&right_heel_evaluator);
  // Cost
  int n_v = plant_wo_springs.num_velocities();
  MatrixXd Q_accel = gains.w_accel * MatrixXd::Identity(n_v, n_v);
  Q_accel(6, 6) = 0.1;
  Q_accel(7, 7) = 0.1;
  Q_accel(8, 8) = 0.1;
  Q_accel(9, 9) = 0.1;
  osc->SetAccelerationCostForAllJoints(Q_accel);
  // Center of mass tracking
  // Weighting x-y higher than z, as they are more important to balancing
  ComTrackingData center_of_mass_traj("com_traj", K_p_com, K_d_com,
                                      W_com * FLAGS_cost_weight_multiplier,
                                      plant_w_springs, plant_wo_springs);
  osc->AddTrackingData(&center_of_mass_traj);
  // Pelvis rotation tracking
  RotTaskSpaceTrackingData pelvis_rot_traj(
      "pelvis_rot_traj", K_p_pelvis, K_d_pelvis,
      W_pelvis * FLAGS_cost_weight_multiplier, plant_w_springs,
      plant_wo_springs);
  pelvis_rot_traj.AddFrameToTrack("pelvis");
  osc->AddTrackingData(&pelvis_rot_traj);

  JointSpaceTrackingData hip_yaw_left_tracking(
      "hip_yaw_left_traj", K_p_hip_yaw, K_d_hip_yaw,
      W_hip_yaw * FLAGS_cost_weight_multiplier, plant_w_springs,
      plant_wo_springs);
  JointSpaceTrackingData hip_yaw_right_tracking(
      "hip_yaw_right_traj", K_p_hip_yaw, K_d_hip_yaw,
      W_hip_yaw * FLAGS_cost_weight_multiplier, plant_w_springs,
      plant_wo_springs);
  hip_yaw_left_tracking.AddJointToTrack("hip_yaw_left", "hip_yaw_leftdot");
  hip_yaw_right_tracking.AddJointToTrack("hip_yaw_right", "hip_yaw_rightdot");
  osc->AddConstTrackingData(&hip_yaw_left_tracking, 0.0 * VectorXd::Ones(1));
  osc->AddConstTrackingData(&hip_yaw_right_tracking, 0.0 * VectorXd::Ones(1));

  // Build OSC problem
  osc->Build();
  // Connect ports
  builder.Connect(state_receiver->get_output_port(0),
                  osc->get_robot_output_input_port());
  builder.Connect(osc->get_osc_output_port(),
                  command_sender->get_input_port(0));
  builder.Connect(osc->get_osc_debug_port(), osc_debug_pub->get_input_port());
  builder.Connect(com_traj_generator->get_output_port(0),
                  osc->get_tracking_data_input_port("com_traj"));
  builder.Connect(pelvis_rot_traj_generator->get_output_port(0),
                  osc->get_tracking_data_input_port("pelvis_rot_traj"));

  // Create the diagram
  auto owned_diagram = builder.Build();
  owned_diagram->set_name(("osc standing controller"));

  // Build lcm-driven simulation
  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm_local, std::move(owned_diagram), state_receiver, FLAGS_channel_x,
      true);

  loop.Simulate();

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::DoMain(argc, argv); }
