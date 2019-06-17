// Kp and 'Rotational' Kp
#define K_P 1000
#define K_OMEGA 50

// Kd and 'Rotational' Kd
#define K_D 25
#define K_R 3

#define NUM_JOINTS 7
#define ENDEFFECTOR_BODY_ID 10

#include <vector>
#include <iostream>
#include <iomanip>
#include <ctime>

#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/primitives/multiplexer.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/sine.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
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

//#include "systems/controllers/endeffector_velocity_controller.h"
#include "systems/controllers/endeffector_position_controller.h"

namespace dairlib {

void setup_log(double amplitude, double frequency);

int do_main(int argc, char* argv[]) {
  drake::lcm::DrakeLcm lcm;
  drake::systems::DiagramBuilder<double> builder;

  // Initialize Kuka model URDF-- from Drake kuka simulation files
  const char* kModelPath = "../drake/manipulation/models/iiwa_description/urdf/iiwa14_polytope_collision.urdf";
  const std::string urdf = FindResourceOrThrow(kModelPath);

  // RigidBodyTrees are created here, then passed by reference to the controller blocks for
  // internal modelling.
  std::unique_ptr<RigidBodyTree<double>> tree = std::make_unique<RigidBodyTree<double>>();
  drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
    urdf, drake::multibody::joints::kFixed, tree.get());

  // Adding status subscriber and receiver blocks
  auto status_subscriber = builder.AddSystem(
    drake::systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_iiwa_status>(
      "IIWA_STATUS", &lcm));
  auto status_receiver = builder.AddSystem<drake::examples::kuka_iiwa_arm::IiwaStatusReceiver>();

  // Adding command publisher and broadcaster blocks
  auto command_sender = builder.AddSystem<drake::examples::kuka_iiwa_arm::IiwaCommandSender>();
  auto command_publisher = builder.AddSystem(
    drake::systems::lcm::LcmPublisherSystem::Make<drake::lcmt_iiwa_command>(
      "IIWA_COMMAND", &lcm, 1.0/200.0));
  // Setting command publisher publish period
  //command_publisher->set_publish_period(1.0/200.0);
  
  Eigen::VectorXd zeros(6);
  zeros << 0, 0, 0, 0, 0, 0;
  auto zeros_source = builder.AddSystem<drake::systems::ConstantVectorSource>(zeros);
    
  double amplitude = 30;
  double frequency = 5;
  auto sine_source = builder.AddSystem<drake::systems::Sine>(amplitude, frequency, 0, 1, true);
  setup_log(amplitude, frequency);

  std::vector<int> input_sizes = {6, 1};
  auto mux = builder.AddSystem<drake::systems::Multiplexer>(input_sizes);

  builder.Connect(zeros_source->get_output_port(),
                  mux->get_input_port(0));
  builder.Connect(sine_source->get_output_port(0),
                  mux->get_input_port(1));

  builder.Connect(status_subscriber->get_output_port(),
                  status_receiver->get_input_port());
  builder.Connect(mux->get_output_port(0),
                  command_sender->get_torque_input_port());

  builder.Connect(status_receiver->get_position_measured_output_port(),
                  command_sender->get_position_input_port());
  builder.Connect(command_sender->get_output_port(),
                  command_publisher->get_input_port());

  auto diagram = builder.Build();
  drake::systems::Simulator<double> simulator(*diagram);

  simulator.set_publish_every_time_step(false);
  simulator.set_publish_at_initialization(false);
  simulator.set_target_realtime_rate(1.0);
  simulator.Initialize();

  lcm.StartReceiveThread();

  simulator.StepTo(20);
  return 0;
}

void setup_log(double amplitude, double frequency) {
  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);

  std::ostringstream oss;
  oss << std::put_time(&tm, "%d_%m_%Y-%H_%M_%S");
  auto time_str = oss.str();

  auto file_str = "logs/test_config_" + time_str + ".txt";

  auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(file_str, true);
  drake::log()->sinks().push_back(file_sink);

  drake::log()->info("Testing with amplitude {} and frequency {}", amplitude, frequency);
  drake::log()->info("Has spdlog: {}", drake::logging::kHaveSpdlog);
  
}

} // Namespace dairlib

int main(int argc, char* argv[]) {
  return dairlib::do_main(argc, argv);
}
