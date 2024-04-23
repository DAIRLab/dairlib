#include <vector>
#include <math.h>
#include <gflags/gflags.h>

#include <drake/math/rigid_transform.h>
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include <drake/lcm/drake_lcm.h>
#include <drake/multibody/parsing/parser.h>

#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_c3.hpp"

#include "multibody/multibody_utils.h"

#include "systems/robot_lcm_systems.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/controllers/impedance_controller.h"
#include "systems/system_utils.h"

#include "examples/franka_ball_rolling/parameters/impedance_controller_params.h"
#include "examples/franka_ball_rolling/systems/gravity_compensator.h"


namespace dairlib {

using drake::geometry::SceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::math::RigidTransform;
using drake::systems::DiagramBuilder;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::Context;
using drake::multibody::Parser;

using Eigen::VectorXd;
using Eigen::MatrixXd;

int DoMain(int argc, char* argv[]){
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  /* -------------------------------------------------------------------------------------------*/
  ImpedanceControllerParams impedance_param = drake::yaml::LoadYamlFile<ImpedanceControllerParams>(
    "examples/franka_ball_rolling/parameters/impedance_controller_params.yaml");

  /// TODO: in the end should remove ball related stuff since impedance controller is only franka related
  drake::lcm::DrakeLcm drake_lcm;

  MultibodyPlant<double> plant(0.0);
  Parser parser(&plant);
  drake::multibody::ModelInstanceIndex franka_index = parser.AddModels(impedance_param.franka_model)[0];
  drake::multibody::ModelInstanceIndex end_effector_index = parser.AddModels(impedance_param.end_effector_model)[0];
  
  /// Fix base of finger to world
  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  RigidTransform<double> X_F_EE = RigidTransform<double>(impedance_param.tool_attachment_frame);

  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_link0"), X_WI);
  plant.WeldFrames(plant.GetFrameByName("panda_link7"), plant.GetFrameByName("end_effector_base"), X_F_EE);
  plant.Finalize();

  DiagramBuilder<double> builder;
  auto state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(plant);
  /* -------------------------------------------------------------------------------------------*/

  auto context = plant.CreateDefaultContext();

  MatrixXd translational_stiffness = impedance_param.translational_stiffness.asDiagonal();
  MatrixXd rotational_stiffness = impedance_param.rotational_stiffness.asDiagonal();
  MatrixXd translational_damping = impedance_param.translational_damping.asDiagonal();
  MatrixXd rotational_damping = impedance_param.rotational_damping.asDiagonal();

  /// TODO: should remove the hard coded dimension in the future, possibly using model indices
  /// TODO: or maybe done in load parameter file
  MatrixXd K = MatrixXd::Zero(6,6);
  MatrixXd B = MatrixXd::Zero(6,6);
  K.block(0,0,3,3) << rotational_stiffness;
  K.block(3,3,3,3) << translational_stiffness;
  B.block(0,0,3,3) << rotational_damping;
  B.block(3,3,3,3) << translational_damping;

  MatrixXd stiffness_null = impedance_param.stiffness_null.asDiagonal();
  MatrixXd damping_null = impedance_param.damping_null.asDiagonal();

  MatrixXd K_null = impedance_param.stiffness_null;
  MatrixXd B_null = impedance_param.damping_null;
  VectorXd qd_null = impedance_param.q_null_desired;

  auto impedance_controller = builder.AddSystem<systems::controllers::ImpedanceController>(
          plant, *context, K, B, K_null, B_null, qd_null);
  auto gravity_compensator = builder.AddSystem<systems::GravityCompensator>(plant, *context);

  /* -------------------------------------------------------------------------------------------*/
  /// get trajectory info from c3
  /// TODO: in the end should make the interface easier and not hard code dimension and port index
  /// TODO: make gravity compensation an option in the impedance controller, not a seperate system
  auto planner_command_subscriber = builder.AddSystem(
    LcmSubscriberSystem::Make<dairlib::lcmt_ball_rolling_command>(
      "CONTROLLER_INPUT", &drake_lcm));
  auto planner_command_receiver =
    builder.AddSystem<systems::BallRollingCommandReceiver>(13, 7, 1);
  builder.Connect(planner_command_subscriber->get_output_port(0),
                  planner_command_receiver->get_input_port(0));
  builder.Connect(planner_command_receiver->get_output_port(0),
                  impedance_controller->get_input_port(1));
  builder.Connect(planner_command_receiver->get_output_port(1),
                  impedance_controller->get_input_port(2));

  /* -------------------------------------------------------------------------------------------*/
  builder.Connect(state_receiver->get_output_port(0),
                  impedance_controller->get_input_port(0));

  auto control_sender = builder.AddSystem<systems::RobotCommandSender>(plant);
  builder.Connect(impedance_controller->get_output_port(), gravity_compensator->get_input_port());
  builder.Connect(gravity_compensator->get_output_port(), control_sender->get_input_port());

  auto control_publisher = builder.AddSystem(
        LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          "FRANKA_INPUT", &drake_lcm, 
          {drake::systems::TriggerType::kForced}, 0.0));
  builder.Connect(control_sender->get_output_port(),
        control_publisher->get_input_port());
  /* -------------------------------------------------------------------------------------------*/

  auto diagram = builder.Build();
  diagram->set_name(("Diagram_Impedance_Controller"));
  DrawAndSaveDiagramGraph(*diagram, "examples/franka_ball_rolling/diagram_lcm_impedance_controller");

  auto context_d = diagram->CreateDefaultContext();
  auto& receiver_context = diagram->GetMutableSubsystemContext(*state_receiver, context_d.get());
  (void) receiver_context; // suppressed unused variable warning

  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &drake_lcm, std::move(diagram), state_receiver, "FRANKA_OUTPUT", true);
  
  /// initialize message
  /// TODO: in the end should find a more elegant way to do this, align with CONTROLLER_INPUT settings
  std::vector<double> target_state_initial(13, 0);
  std::vector<double> feedforward_torque_initial(7, 0);
  std::vector<double> contact_force_initial(1, 0);

//  msg_data[0] = impedance_param.initial_start(0);
//  msg_data[1] = impedance_param.initial_start(1);
//  msg_data[2] = impedance_param.initial_start(2);
//  msg_data[3] = 0;
//  msg_data[4] = 1;
//  msg_data[5] = 0;
//  msg_data[6] = 0;
//  msg_data[7] = 1;
//  msg_data[8] = 0;
//  msg_data[9] = 0;
//  msg_data[10] = 0;
//  msg_data[32] = msg_data[7];
//  msg_data[33] = msg_data[8];
//  msg_data[34] = msg_data[9];
//  msg_data[35] = msg_data[7];
//  msg_data[36] = msg_data[8];
//  msg_data[37] = msg_data[9];

  dairlib::lcmt_ball_rolling_command init_msg;
  init_msg.target_state = target_state_initial;
  init_msg.feedforward_torque = feedforward_torque_initial;
  init_msg.contact_force = contact_force_initial;
  init_msg.num_target_state_variables = 13;
  init_msg.num_feedforward_torque_variables = 7;
  init_msg.num_contact_force_variables = 1;
  init_msg.utime = 0.0;

  /// assign initial message
  auto& diagram_context = loop.get_diagram_mutable_context();
  auto& ik_subscriber_context =
      loop.get_diagram()->GetMutableSubsystemContext(*planner_command_subscriber,
                                                      &diagram_context);
  // Note that currently the LcmSubscriber stores the lcm message in the first
  // state of the leaf system (we hard coded index 0 here)
  auto& mutable_state =
      ik_subscriber_context
          .get_mutable_abstract_state<dairlib::lcmt_ball_rolling_command>(0);
  mutable_state = init_msg;

  loop.Simulate(std::numeric_limits<double>::infinity());

  return 0;
}

} // namespace dairlib

int main(int argc, char* argv[]) { dairlib::DoMain(argc, argv);}
