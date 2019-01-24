#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"

#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_pd_config.hpp"
#include "systems/robot_lcm_systems.h"
#include "systems/controllers/linear_controller.h"
#include "systems/controllers/pd_config_lcm.h"
#include "examples/Cassie/cassie_utils.h"

namespace dairlib {

using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::lcm::LcmPublisherSystem;

int doMain() {
  drake::systems::DiagramBuilder<double> builder;
  drake::lcm::DrakeLcm lcm;

  RigidBodyTree<double> tree;
  buildCassieTree(tree);

  const std::string channel_x = "CASSIE_STATE";
  const std::string channel_u = "CASSIE_INPUT";
  const std::string channel_config = "PD_CONFIG";

  // Create state receiver.
  auto state_sub = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_robot_output>(channel_x, &lcm));
  auto state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(tree);
  builder.Connect(state_sub->get_output_port(),
                  state_receiver->get_input_port(0));

  // Create config receiver.
  auto config_sub = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_pd_config>(channel_config, &lcm));
  auto config_receiver = builder.AddSystem<systems::PDConfigReceiver>(tree);
  builder.Connect(config_sub->get_output_port(),
                  config_receiver->get_input_port(0));

  // Create command sender.
  auto command_pub = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(channel_u, &lcm));
  auto command_sender = builder.AddSystem<systems::RobotCommandSender>(tree);

  builder.Connect(command_sender->get_output_port(0),
                  command_pub->get_input_port());


  auto controller = builder.AddSystem<systems::LinearController>(
      tree.get_num_positions(), tree.get_num_velocities(),
      tree.get_num_actuators());
  builder.Connect(state_receiver->get_output_port(0),
                  controller->get_input_port_output());

  builder.Connect(config_receiver->get_output_port(0),
                  controller->get_input_port_config());

  std::cout << controller->get_output_port(0).size() << std::endl;
  std::cout << command_sender->get_input_port(0).size() << std::endl;
  builder.Connect(controller->get_output_port(0),
                  command_sender->get_input_port(0));

  command_pub->set_publish_period(1.0/1000.0);


  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  /// Use the simulator to drive at a fixed rate
  /// If set_publish_every_time_step is true, this publishes twice 
  /// Set realtime rate. Otherwise, runs as fast as possible
  auto stepper = std::make_unique<drake::systems::Simulator<double>>(*diagram, std::move(context));
  stepper->set_publish_every_time_step(false);
  stepper->set_publish_at_initialization(false);
  stepper->set_target_realtime_rate(1.0);
  stepper->Initialize();

  lcm.StartReceiveThread();

  drake::log()->info("controller started");

  stepper->StepTo(std::numeric_limits<double>::infinity());

  return 0;
}


}

int main() { return dairlib::doMain(); }
