#include <vector>
#include <math.h>
#include <gflags/gflags.h>

#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include <drake/lcm/drake_lcm.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/common/trajectories/piecewise_polynomial.h>
#include <drake/math/rigid_transform.h>
#include "drake/systems/primitives/trajectory_source.h"


#include "systems/robot_lcm_systems.h"
#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_c3.hpp"
#include "multibody/multibody_utils.h"

#include "examples/franka_ball_rolling/parameters/impedance_controller_params.h"
#include "examples/franka_ball_rolling/systems/gravity_compensator.h"
#include "systems/controllers/impedance_controller.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/system_utils.h"


DEFINE_string(channel, "FRANKA_OUTPUT",
              "LCM channel for receiving state. "
              "Use FRANKA_OUTPUT to get state from simulator, and "
              "use FRANKA_ROS_OUTPUT to get state from from franka_ros");

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
using drake::trajectories::PiecewisePolynomial;

using Eigen::VectorXd;
using Eigen::MatrixXd;

int DoMain(int argc, char* argv[]){
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  ImpedanceControllerParams impedance_param = drake::yaml::LoadYamlFile<ImpedanceControllerParams>(
    "examples/franka_ball_rolling/parameters/impedance_controller_params.yaml");

  drake::lcm::DrakeLcm drake_lcm;

  MultibodyPlant<double> plant(0.0);
  Parser parser(&plant);
  parser.AddModelFromFile(impedance_param.franka_model);
  parser.AddModelFromFile(impedance_param.offset_model);
  parser.AddModelFromFile(impedance_param.ground_model);
  parser.AddModelFromFile(impedance_param.end_effector_model);
  parser.AddModelFromFile(impedance_param.ball_model);
  
  /// Fix base of finger to world
  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  RigidTransform<double> X_F_EE = RigidTransform<double>(impedance_param.tool_attachment_frame);
  RigidTransform<double> X_F_G = RigidTransform<double>(impedance_param.ground_offset_frame);

  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_link0"), X_WI);
  plant.WeldFrames(plant.GetFrameByName("panda_link7"), plant.GetFrameByName("end_effector_base"), X_F_EE);
  plant.WeldFrames(plant.GetFrameByName("panda_link0"), plant.GetFrameByName("visual_table_offset"), X_WI);
  plant.WeldFrames(plant.GetFrameByName("panda_link0"), plant.GetFrameByName("ground"), X_F_G);
  plant.Finalize();

  DiagramBuilder<double> builder;
  auto state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(plant);

  /* -------------------------------------------------------------------------------------------*/
  // additional plant_contact linked with scene_graph and additional diagram_contact is built
  // all for the geometry and contact information needed for feedforward force_feedback in the
  // impedance controller, it's update in impedance controller should be synchronized with the
  // general plant!

  DiagramBuilder<double> builder_contact;
  auto [plant_contact, scene_graph] = AddMultibodyPlantSceneGraph(&builder_contact, 0.0);
  Parser parser_contact(&plant_contact);
  parser_contact.AddModelFromFile(impedance_param.franka_model);
  parser_contact.AddModelFromFile(impedance_param.offset_model);
  parser_contact.AddModelFromFile(impedance_param.ground_model);
  parser_contact.AddModelFromFile(impedance_param.end_effector_model);
  parser_contact.AddModelFromFile(impedance_param.ball_model);

  plant_contact.WeldFrames(plant_contact.world_frame(), plant_contact.GetFrameByName("panda_link0"), X_WI);
  plant_contact.WeldFrames(plant_contact.GetFrameByName("panda_link7"), plant_contact.GetFrameByName("end_effector_base"), X_F_EE);
  plant_contact.WeldFrames(plant_contact.GetFrameByName("panda_link0"), plant_contact.GetFrameByName("visual_table_offset"), X_WI);
  plant_contact.WeldFrames(plant_contact.GetFrameByName("panda_link0"), plant_contact.GetFrameByName("ground"), X_F_G);
  plant_contact.Finalize();

  auto diagram_contact = builder_contact.Build();
  auto diagram_context_contact = diagram_contact->CreateDefaultContext();
  auto& context_contact = diagram_contact->GetMutableSubsystemContext(plant_contact, diagram_context_contact.get());

  /* -------------------------------------------------------------------------------------------*/

  auto context = plant.CreateDefaultContext();

  MatrixXd translational_stiffness = impedance_param.translational_stiffness.asDiagonal();
  MatrixXd rotational_stiffness = impedance_param.rotational_stiffness.asDiagonal();
  MatrixXd translational_damping = impedance_param.translational_damping.asDiagonal();
  MatrixXd rotational_damping = impedance_param.rotational_damping.asDiagonal();

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
  VectorXd qd = impedance_param.q_null_desired;

  drake::geometry::GeometryId sphere_geoms = 
    plant_contact.GetCollisionGeometriesForBody(plant_contact.GetBodyByName("sphere"))[0];
  drake::geometry::GeometryId EE_geoms = 
    plant_contact.GetCollisionGeometriesForBody(plant_contact.GetBodyByName("end_effector_tip"))[0];
  std::vector<drake::geometry::GeometryId> contact_geoms = {EE_geoms, sphere_geoms};

  int num_friction_directions = impedance_param.num_friction_directions;
  auto controller = builder.AddSystem<systems::controllers::ImpedanceController>(
          plant, plant_contact, *context, context_contact, K, B, K_null, B_null, qd,
          contact_geoms, num_friction_directions);
  auto gravity_compensator = builder.AddSystem<systems::GravityCompensator>(plant, *context);

  /* -------------------------------------------------------------------------------------------*/
  /// get trajectory info from c3
  auto c3_subscriber = builder.AddSystem(
    LcmSubscriberSystem::Make<dairlib::lcmt_c3>(
      "CONTROLLER_INPUT", &drake_lcm));
  auto c3_receiver = 
    builder.AddSystem<systems::RobotC3Receiver>(14, 9, 6, 9);
  builder.Connect(c3_subscriber->get_output_port(0),
    c3_receiver->get_input_port(0));
  builder.Connect(c3_receiver->get_output_port(0),
    controller->get_input_port(1));

  /* -------------------------------------------------------------------------------------------*/

  builder.Connect(state_receiver->get_output_port(0), 
    controller->get_input_port(0));
  
  if (FLAGS_channel == "FRANKA_OUTPUT"){
    auto control_sender = builder.AddSystem<systems::RobotCommandSender>(plant);
    builder.Connect(controller->get_output_port(), gravity_compensator->get_input_port());
    builder.Connect(gravity_compensator->get_output_port(), control_sender->get_input_port());

    auto control_publisher = builder.AddSystem(
        LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          "FRANKA_INPUT", &drake_lcm, 
          {drake::systems::TriggerType::kForced}, 0.0));
    builder.Connect(control_sender->get_output_port(),
        control_publisher->get_input_port());
  }
  /* -------------------------------------------------------------------------------------------*/

  auto diagram = builder.Build();
//  DrawAndSaveDiagramGraph(*diagram, "examples/franka_ball_rolling/lcm_impedance_controller");
//  DrawAndSaveDiagramGraph(*diagram_contact, "examples/franka_ball_rolling/lcm_impedance_controller_contact");


  auto context_d = diagram->CreateDefaultContext();
  auto& receiver_context = diagram->GetMutableSubsystemContext(*state_receiver, context_d.get());
  (void) receiver_context; // suppressed unused variable warning

  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &drake_lcm, std::move(diagram), state_receiver, FLAGS_channel, true);
  
  /// initialize message
  std::vector<double> msg_data(38, 0);
  msg_data[0] = impedance_param.initial_start(0);
  msg_data[1] = impedance_param.initial_start(1);
  msg_data[2] = impedance_param.initial_start(2);
  msg_data[3] = 0;
  msg_data[4] = 1;
  msg_data[5] = 0;
  msg_data[6] = 0;
  msg_data[7] = 1;
  msg_data[8] = 0;
  msg_data[9] = 0;
  msg_data[10] = 0;
  msg_data[11] = impedance_param.traj_radius * sin(M_PI * impedance_param.phase / 180.0) + impedance_param.x_c;
  msg_data[12] = impedance_param.traj_radius * cos(M_PI * impedance_param.phase / 180.0) + impedance_param.y_c;
  msg_data[13] = impedance_param.ball_radius - impedance_param.ground_offset_frame(2);;
  msg_data[32] = msg_data[7];
  msg_data[33] = msg_data[8];
  msg_data[34] = msg_data[9];
  msg_data[35] = msg_data[7];
  msg_data[36] = msg_data[8];
  msg_data[37] = msg_data[9];

  dairlib::lcmt_c3 init_msg;
  init_msg.data = msg_data;
  init_msg.data_size = 38;
  init_msg.utime = 0.0;

  /// assign initial message
  auto& diagram_context = loop.get_diagram_mutable_context();
  auto& ik_subscriber_context =
      loop.get_diagram()->GetMutableSubsystemContext(*c3_subscriber,
                                                      &diagram_context);
  // Note that currently the LcmSubscriber stores the lcm message in the first
  // state of the leaf system (we hard coded index 0 here)
  auto& mutable_state =
      ik_subscriber_context
          .get_mutable_abstract_state<dairlib::lcmt_c3>(0);
  mutable_state = init_msg;

  loop.Simulate(std::numeric_limits<double>::infinity());

  return 0;
}

} // namespace dairlib

int main(int argc, char* argv[]) { dairlib::DoMain(argc, argv);}
