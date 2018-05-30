#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"

#include "drake/lcmt_cassie_state.hpp"
#include "drake/lcmt_cassie_input.hpp"
#include "drake/lcmt_cassie_pd_config.hpp"
#include "pd_controller.h"
#include "cassie_controller_lcm.h"
#include "datatypes/cassie_names.h"

namespace drake{

int doMain() {
  drake::systems::DiagramBuilder<double> builder;
  lcm::DrakeLcm lcm;

  const std::string channel_x = "CASSIE_STATE";
  const std::string channel_u = "CASSIE_INPUT";
  const std::string channel_config = "PD_CONFIG";

  // Create state receiver.
  auto state_sub = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<lcmt_cassie_state>(channel_x, &lcm));
  auto state_receiver = builder.AddSystem<CassieStateReceiver>();
  builder.Connect(state_sub->get_output_port(),
                  state_receiver->get_input_port(0));

  // Create config receiver.
  auto config_sub = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<lcmt_cassie_pd_config>(channel_config, &lcm));
  auto config_receiver = builder.AddSystem<CassiePDConfigReceiver>();
  builder.Connect(config_sub->get_output_port(),
                  config_receiver->get_input_port(0));


  // Create command sender.
  auto command_pub = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<lcmt_cassie_input>(channel_u, &lcm));
  auto command_sender = builder.AddSystem<CassieCommandSender>();
  builder.Connect(command_sender->get_output_port(0),
                  command_pub->get_input_port());


  auto controller = builder.AddSystem<CassiePDController>();

  builder.Connect(state_receiver->get_output_port(0),
                  controller->get_input_port(controller->state_input_port()));
  builder.Connect(config_receiver->get_output_port(0),
                  controller->get_input_port(controller->config_input_port()));
  builder.Connect(controller->get_output_port(0),
                  command_sender->get_input_port(0));

  command_pub->set_publish_period(1.0/200.0);


  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  /// Use the simulator to drive at a fixed rate
  /// If set_publish_every_time_step is true, this publishes twice 
  /// Set realtime rate. Otherwise, runs as fast as possible
  auto stepper = std::make_unique<systems::Simulator<double>>(*diagram, std::move(context));
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

int main() { return drake::doMain(); }
