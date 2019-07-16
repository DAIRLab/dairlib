// Kp and 'Rotational' Kp
#define K_P 1
#define K_OMEGA 0.7

// Kd and 'Rotational' Kd
#define K_D 1
#define K_R 0.3

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_lcm.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/parsing/parser.h"

#include "systems/controllers/endeffector_velocity_controller.h"
#include "systems/controllers/endeffector_position_controller.h"

namespace dairlib {

// This function creates a controller for a Kuka LBR Iiwa arm by connecting an
// EndEffectorPositionController to an EndEffectorVelocityController to control
// the individual joint torques as to move the endeffector
// to a desired position.
int do_main(int argc, char* argv[]) {
  // Creating end effector trajectory
  // TODO make this modular
  const std::vector<double> times {0.0, 25.0, 35.0, 45.0, 55.0, 65.0,
                                   75.0, 85.0, 95.0, 105.0, 115};

  std::vector<Eigen::MatrixXd> points(times.size());

  Eigen::Vector3d AS, A0, A1, A2, A3, A4, A5, A6, A7, A8, A9;

  AS << -0.23, -0.2, 0.25;
  A0 << -0.23, -0.2, 0.25;

  A1 << -0.23, -0.6, 0.25;
  A2 << 0.23, -0.6, 0.25;

  A3 << 0.23, -0.2, 0.25;

  A4 << 0.23, -0.2, 0.25;

  A5 << 0.23, -0.6, 0.25;

  A6 << -0.23, -0.6, 0.25;

  A7 << -0.23, -0.2, 0.25;

  A8 << -0.23, -0.2, 0.25;
  A9 << -0.23, -0.2, 0.25;

  points[0] = AS;
  points[1] = A0;
  points[2] = A1;
  points[3] = A2;
  points[4] = A3;
  points[5] = A4;
  points[6] = A5;
  points[7] = A6;
  points[8] = A7;
  points[9] = A8;
  points[10] = A9;

  auto ee_trajectory = drake::trajectories::PiecewisePolynomial<
      double>::FirstOrderHold(times, points);

  // Creating end effector orientation trajectory
  const std::vector<double> orient_times {0, 115};
  std::vector<Eigen::MatrixXd> orient_points(orient_times.size());
  Eigen::Vector4d start_o, end_o;
  start_o << 0, 0, 1, 0;
  end_o << 0, 0, 1, 0;

  orient_points[0] = start_o;
  orient_points[1] = end_o;

  auto orientation_trajectory = drake::trajectories::PiecewisePolynomial<
      double>::FirstOrderHold(orient_times, orient_points);

  // Initialize Kuka model URDF-- from Drake kuka simulation files
  std::string kModelPath = "../drake/manipulation/models/iiwa_description"
                           "/iiwa7/iiwa7_no_collision.sdf";
  const std::string urdf_string = FindResourceOrThrow(kModelPath);

  // MultibodyPlants are created here, then passed by reference
  // to the controller blocks for internal modelling.
  const auto X_WI = drake::math::RigidTransform<double>::Identity();
  std::unique_ptr<MultibodyPlant<double>> owned_plant =
      std::make_unique<MultibodyPlant<double>>();

  drake::multibody::Parser plant_parser(owned_plant.get());
  plant_parser.AddModelFromFile(urdf_string, "iiwa");
  owned_plant->WeldFrames(owned_plant->world_frame(),
                          owned_plant->GetFrameByName("iiwa_link_0"), X_WI);
  owned_plant->Finalize();

  drake::systems::DiagramBuilder<double> builder;

  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>();

  // Adding status subscriber and receiver blocks
  auto status_subscriber = builder.AddSystem(
      drake::systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_iiwa_status>(
          "IIWA_STATUS", lcm));
  auto status_receiver = builder.AddSystem<
      drake::examples::kuka_iiwa_arm::IiwaStatusReceiver>();

  // The coordinates for the end effector with respect to the last joint,
  // used to determine location of end effector
  Eigen::Vector3d eeContactFrame;
  eeContactFrame << 0.0, 0, 0.09;

  const std::string link_7 = "iiwa_link_7";

  // Adding position controller block
  auto position_controller = builder.AddSystem<
      systems::EndEffectorPositionController>(
          *owned_plant, link_7, eeContactFrame, K_P, K_OMEGA);

  // Adding Velocity Controller block
  auto velocity_controller = builder.AddSystem<
      systems::EndEffectorVelocityController>(
          *owned_plant, link_7, eeContactFrame, K_D, K_R);


  // Adding linear position Trajectory Source
  auto input_trajectory = builder.AddSystem<drake::systems::TrajectorySource>(
      ee_trajectory);
  // Adding orientation Trajectory Source
  auto input_orientation = builder.AddSystem<drake::systems::TrajectorySource>(
      orientation_trajectory);


  // Adding command publisher and broadcaster blocks
  auto command_sender = builder.AddSystem<
      drake::examples::kuka_iiwa_arm::IiwaCommandSender>();
  auto command_publisher = builder.AddSystem(
      drake::systems::lcm::LcmPublisherSystem::Make<drake::lcmt_iiwa_command>(
          "IIWA_COMMAND", lcm, 1.0/200.0));

  // Torque Controller-- includes virtual springs and damping.
  VectorXd ConstPositionCommand;

  // The virtual spring stiffness in Nm/rad.
  ConstPositionCommand.resize(7);
  ConstPositionCommand << 0, 0, 0, 0, 0, 0, 0;

  auto positionCommand = builder.AddSystem<
      drake::systems::ConstantVectorSource>(ConstPositionCommand);

  builder.Connect(status_subscriber->get_output_port(),
                  status_receiver->get_input_port());
  builder.Connect(status_receiver->get_position_measured_output_port(),
                  velocity_controller->get_joint_pos_input_port());
  builder.Connect(status_receiver->get_velocity_estimated_output_port(),
                  velocity_controller->get_joint_vel_input_port());
  //Connecting q input from status receiver to position controller
  builder.Connect(status_receiver->get_position_measured_output_port(),
                  position_controller->get_joint_pos_input_port());
  //Connecting x_desired input from trajectory to position controller
  builder.Connect(input_trajectory->get_output_port(),
                  position_controller->get_endpoint_pos_input_port());
  //Connecting desired orientation to position controller
  builder.Connect(input_orientation->get_output_port(),
                  position_controller->get_endpoint_orient_input_port());
  //Connecting position (twist) controller to trajectory/velocity controller;
  builder.Connect(position_controller->get_endpoint_cmd_output_port(),
                  velocity_controller->get_endpoint_twist_input_port());

  builder.Connect(velocity_controller->get_endpoint_torque_output_port(),
                  command_sender->get_torque_input_port());

  builder.Connect(positionCommand->get_output_port(),
                  command_sender->get_position_input_port());
  builder.Connect(command_sender->get_output_port(),
                  command_publisher->get_input_port());

  auto diagram = builder.Build();

  drake::systems::Simulator<double> simulator(*diagram);

  simulator.set_publish_every_time_step(false);
  simulator.set_target_realtime_rate(1.0);
  simulator.Initialize();
  simulator.AdvanceTo(ee_trajectory.end_time());
  return 0;
}

} // Namespace dairlib

int main(int argc, char* argv[]) {
  return dairlib::do_main(argc, argv);
}
