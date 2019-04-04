
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/manipulation/util/sim_diagram_builder.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_lcm.h"
#include "drake/systems/primitives/trajectory_source.h"

#include "systems/controllers/iiwa_velocity_controller.h"
#include "systems/controllers/iiwa_position_controller.h"

namespace dairlib {

int do_main(int argc, char* argv[]) {

  drake::lcm::DrakeLcm lcm;
  drake::systems::DiagramBuilder<double> builder;

  // Creating end effector trajectory
  // TODO make this modular
  const std::vector<double> times {0.0, 25.0, 35.0, 45.0, 55.0, 65.0, 75.0, 85.0, 95.0, 105.0, 115};

  std::vector<Eigen::MatrixXd> points(times.size());
  
  Eigen::Vector3d AS, A0, A1, A2, A3, A4, A5, A6, A7, A8, A9;

  AS << -0.23, -0.2, 0.05;
  A0 << -0.23, -0.2, 0.05;

  A1 << -0.23, -0.6, 0.05;
  A2 << 0.23, -0.6, 0.05;

  A3 << 0.23, -0.2, 0.05;
  
  A4 << 0.23, -0.2, 0.05;

  A5 << 0.23, -0.6, 0.05;

  A6 << -0.23, -0.6, 0.05;

  A7 << -0.23, -0.2, 0.05;

  A8 << -0.23, -0.2, 0.05;
  A9 << -0.23, -0.2, 0.05;

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


  auto ee_trajectory = drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(times, points);

  // Adding status subscriber and receiver blocks
  auto status_subscriber = builder.AddSystem(
    drake::systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_iiwa_status>(
      "IIWA_STATUS", &lcm));
  auto status_receiver = builder.AddSystem<drake::examples::kuka_iiwa_arm::IiwaStatusReceiver>();

  // Adding Velocity Controller block
  auto velocity_controller = builder.AddSystem<systems::KukaIiwaVelocityController>();

  // Adding position controller block
  auto position_controller = builder.AddSystem<systems::KukaIiwaPositionController>();

  // Adding Trajectory Source
  // TODO: create ee_trajectory
  auto input_trajectory = builder.AddSystem<drake::systems::TrajectorySource>(ee_trajectory);

  // Adding command publisher and broadcaster blocks
  auto command_sender = builder.AddSystem<drake::examples::kuka_iiwa_arm::IiwaCommandSender>();
  auto command_publisher = builder.AddSystem(
    drake::systems::lcm::LcmPublisherSystem::Make<drake::lcmt_iiwa_command>(
      "IIWA_COMMAND", &lcm));
  // Setting command publisher publish period
  command_publisher->set_publish_period(1.0/200.0);


  builder.Connect(status_subscriber->get_output_port(),
                  status_receiver->get_input_port(0));
  builder.Connect(status_receiver->get_position_commanded_output_port(),
                  velocity_controller->get_input_port(0));
  builder.Connect(status_receiver->get_velocity_estimated_output_port(),
                  velocity_controller->get_input_port(1));
  
  //Connecting q input from status receiver to position controller
  builder.Connect(status_receiver->get_position_measured_output_port(),
                  position_controller->get_input_port(0));
  //Connecting x_desired input from trajectory to position controller
  builder.Connect(input_trajectory->get_output_port(),
                  position_controller->get_input_port(1));

  //Connecting position controller to trajectory/velocity controller;
  builder.Connect(position_controller->get_output_port(0),
                  velocity_controller->get_input_port(2));

  //Connecting desired orientation from position controller to trajectory/velocity controller;
  builder.Connect(position_controller->get_output_port(1),
                  velocity_controller->get_input_port(3));

  builder.Connect(velocity_controller->get_output_port(0),
                  command_sender->get_torque_input_port());

  builder.Connect(status_receiver->get_position_measured_output_port(),
                  command_sender->get_position_input_port());
  builder.Connect(command_sender->get_output_port(0),
                  command_publisher->get_input_port());

  auto diagram = builder.Build();
  drake::systems::Simulator<double> simulator(*diagram);

  simulator.set_publish_every_time_step(false);
  simulator.set_publish_at_initialization(false);
  simulator.set_target_realtime_rate(1.0);
  simulator.Initialize();

  lcm.StartReceiveThread();

  simulator.StepTo(ee_trajectory.end_time());
  return 0;
}

} // Namespace dairlib

int main(int argc, char* argv[]) {
  return dairlib::do_main(argc, argv);
}
