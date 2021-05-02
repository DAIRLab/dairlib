// System that sends a short burst of iiwa command LCM messages with
// all fields equal to 0.

#define AMPLITUDE 0
#define FREQUENCY 3
#define MAX_VELOCITY 0

#include <vector>
#include <iostream>
#include <iomanip>
#include <ctime>

#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/multiplexer.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/sine.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/examples/kuka_iiwa_arm/iiwa_lcm.h"
#include "drake/common/find_resource.h"
#include "drake/common/text_logging.h"

#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/sinks/basic_file_sink.h"

#include "systems/controllers/safe_velocity_controller.h"

namespace dairlib {

void setup_log();

int do_main(int argc, char* argv[]) {
  drake::systems::DiagramBuilder<double> builder;
  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>();

  // Adding status subscriber and receiver blocks
  auto status_subscriber = builder.AddSystem(
    drake::systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_iiwa_status>(
      "IIWA_STATUS", lcm));
  auto status_receiver = builder.AddSystem<drake::examples::kuka_iiwa_arm::IiwaStatusReceiver>();

  // Adding command publisher and broadcaster blocks
  auto command_sender = builder.AddSystem<drake::examples::kuka_iiwa_arm::IiwaCommandSender>();
  auto command_publisher = builder.AddSystem(
    drake::systems::lcm::LcmPublisherSystem::Make<drake::lcmt_iiwa_command>(
      "IIWA_COMMAND", lcm, 1.0/200.0));

  const int num_iiwa_joints = 7;
  Eigen::VectorXd zeros = Eigen::VectorXd::Zero(num_iiwa_joints);
  auto zeros_source = builder.AddSystem<drake::systems::ConstantVectorSource>(zeros);

  auto velocity_controller = builder.AddSystem<systems::SafeVelocityController>(
      MAX_VELOCITY, num_iiwa_joints);

  Eigen::VectorXd zeros_seven = Eigen::VectorXd::Zero(7);
  auto constant_position_src = builder.AddSystem<drake::systems::ConstantVectorSource>(zeros_seven);

  builder.Connect(status_subscriber->get_output_port(),
                  status_receiver->get_input_port());

  builder.Connect(zeros_source->get_output_port(),
                  velocity_controller->get_joint_torques_input_port());

  builder.Connect(status_receiver->get_velocity_estimated_output_port(),
                velocity_controller->get_joint_velocities_input_port());

  builder.Connect(velocity_controller->get_joint_torques_output_port(),
                  command_sender->get_torque_input_port());

  // builder.Connect(status_receiver->get_position_measured_output_port(),
  //                 command_sender->get_position_input_port());

  builder.Connect(constant_position_src->get_output_port(),
                  command_sender->get_position_input_port());

  builder.Connect(command_sender->get_output_port(),
                  command_publisher->get_input_port());

  auto diagram = builder.Build();
  drake::systems::Simulator<double> simulator(*diagram);

  simulator.set_publish_every_time_step(false);
  simulator.set_publish_at_initialization(false);
  simulator.set_target_realtime_rate(1.0);
  simulator.Initialize();

  simulator.AdvanceTo(0.5);
  return 0;
}

} // Namespace dairlib

int main(int argc, char* argv[]) {
  return dairlib::do_main(argc, argv);
}
