#include <cmath>

#include <drake/lcm/drake_lcm.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/tree/multibody_element.h>
#include <gflags/gflags.h>

#include "dairlib/lcmt_robot_output.hpp"
#include "examples/franka_ball_rolling/parameters/lcm_channels_params.h"
#include "examples/franka_ball_rolling/parameters/state_estimator_params.h"
#include "examples/franka_ball_rolling/parameters/trajectory_params.h"
#include "examples/franka_ball_rolling/systems/state_estimator.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

namespace dairlib {

using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;

using drake::math::RigidTransform;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  /* -------------------------------------------------------------------------------------------*/
  StateEstimatorParams state_estimator_param = drake::yaml::LoadYamlFile<
      StateEstimatorParams>(
      "examples/franka_ball_rolling/parameters/state_estimator_params.yaml");
  BallRollingTrajectoryParams traj_param =
      drake::yaml::LoadYamlFile<BallRollingTrajectoryParams>(
          "examples/franka_ball_rolling/parameters/trajectory_params.yaml");
  BallRollingLcmChannels lcm_channel_param = drake::yaml::LoadYamlFile<
      BallRollingLcmChannels>(
      "examples/franka_ball_rolling/parameters/lcm_channels_sim_params.yaml");

  DiagramBuilder<double> builder;
  drake::lcm::DrakeLcm drake_lcm;

  MultibodyPlant<double> plant(0.0);
  Parser parser(&plant);
  drake::multibody::ModelInstanceIndex franka_index =
      parser.AddModels(state_estimator_param.franka_model)[0];
  drake::multibody::ModelInstanceIndex ground_index =
      parser.AddModels(state_estimator_param.ground_model)[0];
  drake::multibody::ModelInstanceIndex end_effector_index =
      parser.AddModels(state_estimator_param.end_effector_model)[0];
  drake::multibody::ModelInstanceIndex ball_index =
      parser.AddModels(state_estimator_param.ball_model)[0];

  /// Fix base of finger to world
  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  RigidTransform<double> X_F_EE =
      RigidTransform<double>(state_estimator_param.tool_attachment_frame);
  RigidTransform<double> X_F_G =
      RigidTransform<double>(state_estimator_param.ground_offset_frame);

  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_link0"),
                   X_WI);
  plant.WeldFrames(plant.GetFrameByName("panda_link7"),
                   plant.GetFrameByName("end_effector_base"), X_F_EE);
  plant.WeldFrames(plant.GetFrameByName("panda_link0"),
                   plant.GetFrameByName("ground"), X_F_G);

  plant.Finalize();

  /// subscribe to franka state and ball state
  auto ball_state_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_object_state>(
          lcm_channel_param.true_ball_state_channel, &drake_lcm));
  auto franka_state_reciver =
      builder.AddSystem<systems::RobotOutputReceiver>(plant, franka_index);
  auto true_ball_state_receiver =
      builder.AddSystem<systems::ObjectStateReceiver>(plant, ball_index);
  builder.Connect(ball_state_sub->get_output_port(),
                  true_ball_state_receiver->get_input_port());
  /* -------------------------------------------------------------------------------------------*/
  /// state estimation block
  std::vector<double> p_FIR_values = state_estimator_param.p_FIR_value;
  std::vector<double> v_FIR_values = state_estimator_param.v_FIR_value;

  auto state_estimator = builder.AddSystem<systems::StateEstimator>(
      p_FIR_values, v_FIR_values, state_estimator_param, traj_param);
  builder.Connect(franka_state_reciver->get_output_port(0),
                  state_estimator->get_input_port_franka());

  /// connect state_susbcriber ball position port
  auto to_estimated_ball_position =
      builder.AddSystem<systems::TrueBallToEstimatedBall>(
          state_estimator_param.ball_noise_stddev,
          1.0 / state_estimator_param.estimation_rate, state_estimator_param,
          traj_param);
  builder.Connect(true_ball_state_receiver->get_output_port(0),
                  to_estimated_ball_position->get_input_port_true_ball());
  builder.Connect(to_estimated_ball_position->get_output_port_estimated_ball(),
                  state_estimator->get_input_port_ball());

  drake::lcm::DrakeLcm* pub_lcm;
  pub_lcm = &drake_lcm;

  MultibodyPlant<double> plant_franka(0.0);
  Parser parser_franka(&plant_franka);
  parser_franka.AddModels(state_estimator_param.franka_model);
  plant_franka.WeldFrames(plant_franka.world_frame(),
                          plant_franka.GetFrameByName("panda_link0"), X_WI);
  plant_franka.Finalize();

  auto sender_franka =
      builder.AddSystem<systems::RobotOutputSender>(plant_franka, true);
  auto franka_output_pub =
      builder.AddSystem(LcmPublisherSystem::Make<lcmt_robot_output>(
          lcm_channel_param.franka_state_channel, pub_lcm,
          {drake::systems::TriggerType::kForced}));
  // hard code port index for now
  builder.Connect(state_estimator->get_output_port_franka_state(),
                  sender_franka->get_input_port(0));
  builder.Connect(state_estimator->get_output_port_franka_effort(),
                  sender_franka->get_input_port(1));
  builder.Connect(*sender_franka, *franka_output_pub);

  MultibodyPlant<double> plant_object(0.0);
  Parser parser_object(&plant_object);
  drake::multibody::ModelInstanceIndex object_index =
      parser_object.AddModels(state_estimator_param.ball_model)[0];
  plant_object.Finalize();
  auto sender_object =
      builder.AddSystem<systems::ObjectStateSender>(plant_object, object_index);
  auto object_state_pub =
      builder.AddSystem(LcmPublisherSystem::Make<lcmt_object_state>(
          lcm_channel_param.estimated_ball_state_channel, pub_lcm,
          {drake::systems::TriggerType::kForced}));
  // hard code port index for now
  builder.Connect(state_estimator->get_output_port_object_state(),
                  sender_object->get_input_port(0));
  builder.Connect(*sender_object, *object_state_pub);

  /* ------------------------------- build diagram
   * ------------------------------------------------*/
  auto diagram = builder.Build();
  diagram->set_name(("Diagram_State_Estimation"));
  auto diagram_context = diagram->CreateDefaultContext();
  DrawAndSaveDiagramGraph(
      *diagram, "examples/franka_ball_rolling/diagram_lcm_state_estimator");

  systems::LcmDrivenLoop<lcmt_robot_output> loop(
      &drake_lcm, std::move(diagram), franka_state_reciver,
      lcm_channel_param.franka_output_channel, true);
  loop.Simulate(std::numeric_limits<double>::infinity());

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::DoMain(argc, argv); }
