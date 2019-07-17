#define AMPLITUDE 1
#define FREQUENCY 3
#define MAX_VELOCITY 5

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
#include "drake/manipulation/util/sim_diagram_builder.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_lcm.h"
#include "drake/common/find_resource.h"
#include "drake/multibody/parsers/urdf_parser.h"
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

  Eigen::VectorXd zeros_low = Eigen::VectorXd::Zero(4);
  auto zeros_low_source = builder.AddSystem<drake::systems::ConstantVectorSource>(zeros_low);

  Eigen::VectorXd zeros_high = Eigen::VectorXd::Zero(2);
  auto zeros_high_source = builder.AddSystem<drake::systems::ConstantVectorSource>(zeros_high);

  auto sine_source = builder.AddSystem<drake::systems::Sine>(AMPLITUDE, FREQUENCY, 0, 1, true);
  //setup_log();

  std::vector<int> input_sizes = {4, 1, 2};
  auto mux = builder.AddSystem<drake::systems::Multiplexer>(input_sizes);

  const int num_iiwa_joints = 7;
  auto velocity_controller = builder.AddSystem<systems::SafeVelocityController>(
      MAX_VELOCITY, num_iiwa_joints);

  Eigen::VectorXd zeros_seven = Eigen::VectorXd::Zero(7);
  auto constant_position_src = builder.AddSystem<drake::systems::ConstantVectorSource>(zeros_seven);

  builder.Connect(zeros_low_source->get_output_port(),
                  mux->get_input_port(0));
  builder.Connect(sine_source->get_output_port(0),
                  mux->get_input_port(1));
  builder.Connect(zeros_high_source->get_output_port(),
                  mux->get_input_port(2));

  builder.Connect(status_subscriber->get_output_port(),
                  status_receiver->get_input_port());

  builder.Connect(mux->get_output_port(0),
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

  simulator.StepTo(20);
  return 0;
}

void setup_log() {
  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);

  std::ostringstream oss;
  oss << std::put_time(&tm, "%d_%m_%Y-%H_%M_%S");
  auto time_str = oss.str();

  auto file_str = "logs/test_config_" + time_str + ".txt";

  auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(file_str, true);
  drake::log()->sinks().push_back(file_sink);

  drake::log()->info("Testing with amplitude {} and frequency {}", AMPLITUDE, FREQUENCY);
  drake::log()->info("Max velocity {}", MAX_VELOCITY);
  drake::log()->info("Has spdlog: {}", drake::logging::kHaveSpdlog);

}

} // Namespace dairlib

int main(int argc, char* argv[]) {
  return dairlib::do_main(argc, argv);
}
