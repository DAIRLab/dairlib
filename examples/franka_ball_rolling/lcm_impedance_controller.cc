#include <math.h>

#include <vector>

#include <drake/lcm/drake_lcm.h>
#include <drake/math/rigid_transform.h>
#include <drake/multibody/parsing/parser.h>
#include <gflags/gflags.h>

#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "examples/franka_ball_rolling/parameters/impedance_controller_params.h"
#include "examples/franka_ball_rolling/parameters/lcm_channels_params.h"
#include "multibody/multibody_utils.h"
#include "systems/controllers/impedance_controller.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"

#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

namespace dairlib {

using drake::geometry::SceneGraph;
using drake::math::RigidTransform;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::Context;
using drake::systems::DiagramBuilder;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;

using Eigen::MatrixXd;
using Eigen::VectorXd;

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  /* -------------------------------------------------------------------------------------------*/
  ImpedanceControllerParams impedance_param =
      drake::yaml::LoadYamlFile<ImpedanceControllerParams>(
          "examples/franka_ball_rolling/parameters/"
          "impedance_controller_params.yaml");
  BallRollingLcmChannels lcm_channel_param = drake::yaml::LoadYamlFile<
      BallRollingLcmChannels>(
      "examples/franka_ball_rolling/parameters/lcm_channels_sim_params.yaml");

  drake::lcm::DrakeLcm drake_lcm;

  MultibodyPlant<double> plant(0.0);
  Parser parser(&plant);
  drake::multibody::ModelInstanceIndex franka_index =
      parser.AddModels(impedance_param.franka_model)[0];
  drake::multibody::ModelInstanceIndex end_effector_index =
      parser.AddModels(impedance_param.end_effector_model)[0];

  /// Fix base of finger to world
  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  RigidTransform<double> X_F_EE =
      RigidTransform<double>(impedance_param.tool_attachment_frame);

  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_link0"),
                   X_WI);
  plant.WeldFrames(plant.GetFrameByName("panda_link7"),
                   plant.GetFrameByName("end_effector_base"), X_F_EE);
  plant.Finalize();

  DiagramBuilder<double> builder;
  auto state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(plant);
  /* -------------------------------------------------------------------------------------------*/

  auto context = plant.CreateDefaultContext();

  MatrixXd K = impedance_param.K;
  MatrixXd B = impedance_param.B;

  MatrixXd K_null = impedance_param.K_null;
  MatrixXd B_null = impedance_param.B_null;
  VectorXd qd_null = impedance_param.q_null_desired;

  bool gravity_compensation = impedance_param.gravity_compensation_flag;

  auto impedance_controller =
      builder.AddSystem<systems::controllers::ImpedanceController>(
          plant, *context, K, B, K_null, B_null, qd_null, gravity_compensation);

  /* -------------------------------------------------------------------------------------------*/
  /// get c3 planner command to track by impedance controller
  auto planner_command_subscriber = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_ball_rolling_command>(
          "CONTROLLER_INPUT", &drake_lcm));
  auto planner_command_receiver =
      builder.AddSystem<systems::BallRollingCommandReceiver>(
          7 + 6, plant.num_actuators(franka_index), 1);
  builder.Connect(planner_command_subscriber->get_output_port(0),
                  planner_command_receiver->get_input_port_planner_command());
  builder.Connect(planner_command_receiver->get_output_port_target_state(),
                  impedance_controller->get_input_port_planner_target());
  builder.Connect(planner_command_receiver->get_output_port_contact_torque(),
                  impedance_controller->get_input_port_contact_feedforward());

  /* -------------------------------------------------------------------------------------------*/
  builder.Connect(state_receiver->get_output_port(0),
                  impedance_controller->get_input_port(0));

  auto control_sender = builder.AddSystem<systems::RobotCommandSender>(plant);
  builder.Connect(impedance_controller->get_output_port(),
                  control_sender->get_input_port());

  auto control_publisher =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          lcm_channel_param.franka_input_channel, &drake_lcm,
          {drake::systems::TriggerType::kForced}, 0.0));
  builder.Connect(control_sender->get_output_port(),
                  control_publisher->get_input_port());
  /* -------------------------------------------------------------------------------------------*/

  auto diagram = builder.Build();
  diagram->set_name(("Diagram_Impedance_Controller"));
  DrawAndSaveDiagramGraph(
      *diagram,
      "examples/franka_ball_rolling/diagram_lcm_impedance_controller");

  auto context_d = diagram->CreateDefaultContext();
  auto& receiver_context =
      diagram->GetMutableSubsystemContext(*state_receiver, context_d.get());
  (void)receiver_context;  // suppressed unused variable warning

  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &drake_lcm, std::move(diagram), state_receiver,
      lcm_channel_param.franka_output_channel, true);

  /// initialize message
  std::vector<double> target_state_initial(7 + 6, 0);
  std::vector<double> feedforward_torque_initial(
      plant.num_actuators(franka_index), 0);
  std::vector<double> contact_force_initial(1, 0);

  dairlib::lcmt_ball_rolling_command init_msg;
  init_msg.target_state = target_state_initial;
  init_msg.feedforward_torque = feedforward_torque_initial;
  init_msg.contact_force = contact_force_initial;
  init_msg.num_target_state_variables = 7 + 6;
  init_msg.num_feedforward_torque_variables = plant.num_actuators(franka_index);
  init_msg.num_contact_force_variables = 1;
  init_msg.utime = 0.0;

  /// assign initial message
  auto& diagram_context = loop.get_diagram_mutable_context();
  auto& ik_subscriber_context = loop.get_diagram()->GetMutableSubsystemContext(
      *planner_command_subscriber, &diagram_context);
  // Note that currently the LcmSubscriber stores the lcm message in the first
  // state of the leaf system (we hard coded index 0 here)
  auto& mutable_state =
      ik_subscriber_context
          .get_mutable_abstract_state<dairlib::lcmt_ball_rolling_command>(0);
  mutable_state = init_msg;

  loop.Simulate(std::numeric_limits<double>::infinity());

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { dairlib::DoMain(argc, argv); }
