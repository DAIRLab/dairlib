#include <time.h>

#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/primitives/sine.h"

#include "cassie_controller_lcm.h"
#include "dairlib/lcmt_cassie_state.hpp"

namespace drake{

int doMain() {
  drake::systems::DiagramBuilder<double> builder;
  lcm::DrakeLcm lcm;

  const std::string channel_x = "CASSIE_STATE";
  
  //sine vector source
  auto sine_source = builder.AddSystem<systems::Sine<double>>(2,1,0,20);

  // Create state sender.
  auto state_pub = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<dairlib::lcmt_cassie_state>(channel_x, &lcm));
  auto state_sender = builder.AddSystem<CassieStateSender>();

  builder.Connect(sine_source->get_output_port(0), state_sender->get_input_port(0));

  builder.Connect(state_sender->get_output_port(0),
                  state_pub->get_input_port());

  state_pub->set_publish_period(1.0/200.0);


  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  time_t now;
  struct tm newyear;
  time(&now);
  newyear = *localtime(&now);
  newyear.tm_hour = 0; newyear.tm_min = 0; newyear.tm_sec = 0;
  newyear.tm_mon = 0;  newyear.tm_mday = 1;
  double dt = difftime(now, mktime(&newyear));
  context->set_time(dt);

  auto stepper = std::make_unique<systems::Simulator<double>>(*diagram, std::move(context));
  stepper->set_publish_every_time_step(false);
  stepper->set_publish_at_initialization(false);
  stepper->set_target_realtime_rate(1.0);
  stepper->Initialize();

  lcm.StartReceiveThread();

  drake::log()->info("state publisher started");

  stepper->StepTo(std::numeric_limits<double>::infinity());

  return 0;
}


}

int main() { return drake::doMain(); }
