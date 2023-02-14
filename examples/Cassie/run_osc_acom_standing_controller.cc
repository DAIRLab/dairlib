#include <gflags/gflags.h>

#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_target_standing_height.hpp"
#include "examples/Cassie/cassie_utils.h"
#include "examples/Cassie/osc/osc_acom_standing_gains.h"
#include "examples/Cassie/osc/standing_com_traj.h"
#include "examples/Cassie/osc/standing_pelvis_orientation_traj.h"
#include "examples/Cassie/systems/cassie_out_to_radio.h"
#include "multibody/kinematic/fixed_joint_evaluator.h"
#include "multibody/kinematic/kinematic_evaluator_set.h"
#include "multibody/multibody_utils.h"
#include "systems/controllers/osc/com_tracking_data.h"
#include "systems/controllers/osc/joint_space_tracking_data.h"
#include "systems/controllers/osc/operational_space_control.h"
#include "systems/controllers/osc/options_tracking_data.h"
#include "systems/controllers/osc/osc_gains.h"
#include "systems/controllers/osc/relative_translation_tracking_data.h"
#include "systems/controllers/osc/rot_space_tracking_data.h"
#include "systems/controllers/osc/trans_space_tracking_data.h"
#include "systems/controllers/osc/acom_tracking_data.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"

#include "drake/common/yaml/yaml_io.h"
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
using drake::systems::TriggerTypeSet;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using multibody::FixedJointEvaluator;
using multibody::WorldYawViewFrame;

using systems::controllers::AcomTrackingData;
using systems::controllers::ComTrackingData;
using systems::controllers::JointSpaceTrackingData;
using systems::controllers::RelativeTranslationTrackingData;
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
DEFINE_double(cost_weight_multiplier, 1.0,
              "A cosntant times with cost weight of OSC traj tracking");
DEFINE_double(height, .8, "The initial COM height (m)");
DEFINE_string(osc_gains_filename, "examples/Cassie/osc/osc_standing_gains.yaml",
              "Filepath containing osc_gains");
DEFINE_string(osqp_settings, "solvers/default_osc_osqp_settings.yaml",
              "Filepath containing qp settings");
DEFINE_bool(use_radio, false, "use the radio to set height or not");

// Currently the controller runs at the rate between 500 Hz and 200 Hz, so the
// publish rate of the robot state needs to be less than 500 Hz. Otherwise, the
// performance seems to degrade due to this. (Recommended publish rate: 200 Hz)
// Maybe we need to update the lcm driven loop to clear the queue of lcm message
// if it's more than one message?

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Build Cassie MBP
  drake::multibody::MultibodyPlant<double> plant(0.0);
  AddCassieMultibody(&plant, nullptr, true /*floating base*/,
                     "examples/Cassie/urdf/cassie_v2_conservative.urdf",
                     true /*spring model*/, false /*loop closure*/);
  plant.Finalize();
  // Build fix-spring Cassie MBP
  //  drake::multibody::MultibodyPlant<double> plant(0.0);
  //  AddCassieMultibody(
  //      &plant, nullptr, true,
  //      "examples/Cassie/urdf/cassie_fixed_springs_conservative.urdf", false,
  //      false);
  //  plant.Finalize();

  auto context_w_spr = plant.CreateDefaultContext();
  //  auto context_wo_spr = plant.CreateDefaultContext();

  // Get contact frames and position (doesn't matter whether we use
  // plant or plant because the contact frames exit in both
  // plants)
  auto left_toe = LeftToeFront(plant);
  auto left_heel = LeftToeRear(plant);
  auto right_toe = RightToeFront(plant);
  auto right_heel = RightToeRear(plant);

  // Build the controller diagram
  DiagramBuilder<double> builder;

  drake::lcm::DrakeLcm lcm_local("udpm://239.255.76.67:7667?ttl=0");
  //  auto osc_gains =
  //      drake::yaml::LoadYamlFile<OSCStandingGains>(FLAGS_osc_gains_filename);
  drake::yaml::LoadYamlOptions yaml_options;
  yaml_options.allow_yaml_with_no_cpp = true;
  OSCGains gains = drake::yaml::LoadYamlFile<OSCGains>(
      FindResourceOrThrow(FLAGS_osc_gains_filename), {}, {}, yaml_options);
  OSCStandingGains osc_gains = drake::yaml::LoadYamlFile<OSCStandingGains>(
      FindResourceOrThrow(FLAGS_osc_gains_filename));

  // Create Lcm subsriber for lcmt_target_standing_height
  auto target_height_receiver = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_target_standing_height>(
          "TARGET_HEIGHT", &lcm_local));

  // Create state receiver.
  auto state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(plant);

  auto cassie_out_receiver =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_cassie_out>(
          FLAGS_cassie_out_channel, &lcm_local));
  auto cassie_out_to_radio = builder.AddSystem<systems::CassieOutToRadio>();
  builder.Connect(*cassie_out_receiver, *cassie_out_to_radio);

  // Create command sender.
  auto command_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          FLAGS_channel_u, &lcm_local, TriggerTypeSet({TriggerType::kForced})));
  auto command_sender = builder.AddSystem<systems::RobotCommandSender>(plant);

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
      plant, context_w_spr.get(), feet_contact_points, FLAGS_height,
      FLAGS_use_radio);
  com_traj_generator->SetCommandFilter(
      osc_gains.center_of_mass_command_filter_alpha);
  auto pelvis_rot_traj_generator =
      builder.AddSystem<cassie::osc::StandingPelvisOrientationTraj>(
          plant, context_w_spr.get(), feet_contact_points, "pelvis_rot_traj");
  pelvis_rot_traj_generator->SetCommandFilter(
      osc_gains.orientation_command_filter_alpha);
  builder.Connect(state_receiver->get_output_port(0),
                  com_traj_generator->get_input_port_state());
  builder.Connect(state_receiver->get_output_port(0),
                  pelvis_rot_traj_generator->get_input_port_state());
  builder.Connect(cassie_out_to_radio->get_output_port(),
                  pelvis_rot_traj_generator->get_input_port_radio());
  builder.Connect(cassie_out_to_radio->get_output_port(),
                  com_traj_generator->get_input_port_radio());
  builder.Connect(target_height_receiver->get_output_port(),
                  com_traj_generator->get_input_port_target_height());

  // Create Operational space control
  auto osc = builder.AddSystem<systems::controllers::OperationalSpaceControl>(
      plant, plant, context_w_spr.get(), context_w_spr.get(), false);

  // Distance constraint
  multibody::KinematicEvaluatorSet<double> evaluators(plant);
  auto left_loop = LeftLoopClosureEvaluator(plant);
  auto right_loop = RightLoopClosureEvaluator(plant);
  evaluators.add_evaluator(&left_loop);
  evaluators.add_evaluator(&right_loop);

  // Fix the springs in the dynamics
  auto pos_idx_map = multibody::MakeNameToPositionsMap(plant);
  auto vel_idx_map = multibody::MakeNameToVelocitiesMap(plant);
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

  // Friction coefficient
  osc->SetContactFriction(gains.mu);
  // Add contact points
  auto left_toe_evaluator = multibody::WorldPointEvaluator(
      plant, left_toe.first, left_toe.second, Matrix3d::Identity(),
      Vector3d::Zero(), {1, 2});
  osc->AddContactPoint(&left_toe_evaluator);
  auto left_heel_evaluator = multibody::WorldPointEvaluator(
      plant, left_heel.first, left_heel.second, Matrix3d::Identity(),
      Vector3d::Zero(), {0, 1, 2});
  osc->AddContactPoint(&left_heel_evaluator);
  auto right_toe_evaluator = multibody::WorldPointEvaluator(
      plant, right_toe.first, right_toe.second, Matrix3d::Identity(),
      Vector3d::Zero(), {1, 2});
  osc->AddContactPoint(&right_toe_evaluator);
  auto right_heel_evaluator = multibody::WorldPointEvaluator(
      plant, right_heel.first, right_heel.second, Matrix3d::Identity(),
      Vector3d::Zero(), {0, 1, 2});
  osc->AddContactPoint(&right_heel_evaluator);
  // Cost
  int n_v = plant.num_velocities();

  osc->SetAccelerationCostWeights(gains.w_accel * gains.W_acceleration);
  osc->SetInputSmoothingCostWeights(gains.w_input_reg *
                                    gains.W_input_regularization);
  osc->SetInputCostWeights(gains.w_input * gains.W_input_regularization);
  osc->SetLambdaHolonomicRegularizationWeight(gains.w_lambda *
                                              gains.W_lambda_h_regularization);
  osc->SetJointLimitWeight(1.0);

  auto pelvis_view_frame = std::make_shared<WorldYawViewFrame<double>>(plant.GetBodyByName("pelvis"));
  auto pelvis_tracking_data = std::make_unique<TransTaskSpaceTrackingData>(
      "pelvis_trans_traj", MatrixXd::Zero(3, 3), MatrixXd::Zero(3, 3),
      MatrixXd::Zero(3, 3), plant, plant);
  auto stance_foot_tracking_data = std::make_unique<TransTaskSpaceTrackingData>(
      "stance_ft_traj", MatrixXd::Zero(3, 3), MatrixXd::Zero(3, 3),
      MatrixXd::Zero(3, 3), plant, plant);
  pelvis_tracking_data->AddPointToTrack("pelvis");
  stance_foot_tracking_data->AddPointToTrack("toe_left");
  auto pelvis_trans_rel_tracking_data =
      std::make_unique<RelativeTranslationTrackingData>(
          "pelvis_trans_traj", osc_gains.K_p_pelvis_rot,
          osc_gains.K_d_pelvis_rot, osc_gains.W_pelvis_rot, plant, plant,
          pelvis_tracking_data.get(), stance_foot_tracking_data.get());
  if (osc_gains.center_of_mass_filter_tau > 0) {
    pelvis_trans_rel_tracking_data->SetLowPassFilter(
        osc_gains.center_of_mass_filter_tau, {0, 1, 2});
  }
  pelvis_trans_rel_tracking_data->SetViewFrame(pelvis_view_frame);
  Eigen::MatrixXd W_pelvis_rot = osc_gains.W_pelvis_rot;
  Eigen::MatrixXd W_acom = osc_gains.W_acom;
  if (osc_gains.AcomSelectionX) {
    W_pelvis_rot(0,0) = 0;
  } else {
    W_acom(0,0) = 0;
  }
  if (osc_gains.AcomSelectionY) {
    W_pelvis_rot(1,1) = 0;
  } else {
    W_acom(1,1) = 0;
  }
  if (osc_gains.AcomSelectionZ) {
    W_pelvis_rot(2,2) = 0;
  } else {
    W_acom(2,2) = 0;
  }

  auto pelvis_rot_tracking_data = std::make_unique<RotTaskSpaceTrackingData>(
      "pelvis_rot_traj", osc_gains.K_p_pelvis_rot, osc_gains.K_d_pelvis_rot,
      W_pelvis_rot, plant, plant);
  pelvis_rot_tracking_data->AddFrameToTrack("pelvis");
  if (osc_gains.rot_filter_tau > 0) {
    pelvis_rot_tracking_data->SetLowPassFilter(osc_gains.rot_filter_tau,
                                               {0, 1, 2});
  }
  auto acom_tracking_data = std::make_unique<AcomTrackingData>(
      "acom_traj", osc_gains.K_p_acom, osc_gains.K_d_acom,
      W_acom, plant, plant);
  acom_tracking_data->AddStateToTrack(-1);
  if (osc_gains.rot_filter_tau > 0) {
    acom_tracking_data->SetLowPassFilter(osc_gains.rot_filter_tau,
                                               {0, 1, 2});
  }
  // To test the standing controller with various initial Cassie yaw, we need to
  // modify the disired yaw to local frame.
  // acom_tracking_data->SetViewFrame(pelvis_view_frame);
  osc->AddTrackingData(std::move(pelvis_trans_rel_tracking_data));
  osc->AddTrackingData(std::move(pelvis_rot_tracking_data));
  osc->AddTrackingData(std::move(acom_tracking_data));

  auto left_hip_yaw_traj = std::make_unique<JointSpaceTrackingData>(
      "left_hip_yaw_traj", osc_gains.K_d_hip_yaw, osc_gains.K_p_hip_yaw,
      osc_gains.W_hip_yaw, plant, plant);
  auto right_hip_yaw_traj = std::make_unique<JointSpaceTrackingData>(
      "right_hip_yaw_traj", osc_gains.K_d_hip_yaw, osc_gains.K_p_hip_yaw,
      osc_gains.W_hip_yaw, plant, plant);
  left_hip_yaw_traj->AddJointToTrack("hip_yaw_left", "hip_yaw_leftdot");
  osc->AddConstTrackingData(std::move(left_hip_yaw_traj), VectorXd::Zero(1));
  right_hip_yaw_traj->AddJointToTrack("hip_yaw_right", "hip_yaw_rightdot");
  osc->AddConstTrackingData(std::move(right_hip_yaw_traj), VectorXd::Zero(1));

  // Build OSC problem
  osc->Build();
  // Connect ports
  builder.Connect(state_receiver->get_output_port(0),
                  osc->get_input_port_robot_output());
  builder.Connect(osc->get_output_port_osc_command(),
                  command_sender->get_input_port(0));
  builder.Connect(osc->get_output_port_osc_debug(), osc_debug_pub->get_input_port());
  builder.Connect(com_traj_generator->get_output_port(0),
                  osc->get_input_port_tracking_data("pelvis_trans_traj"));
  builder.Connect(pelvis_rot_traj_generator->get_output_port(0),
                  osc->get_input_port_tracking_data("pelvis_rot_traj"));
  builder.Connect(pelvis_rot_traj_generator->get_output_port(0),
                  osc->get_input_port_tracking_data("acom_traj"));

  // Create the diagram
  auto owned_diagram = builder.Build();
  owned_diagram->set_name(("osc acom standing controller"));

  // Build lcm-driven simulation
  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm_local, std::move(owned_diagram), state_receiver, FLAGS_channel_x,
      true);
  DrawAndSaveDiagramGraph(*loop.get_diagram());

  loop.Simulate();

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::DoMain(argc, argv); }
