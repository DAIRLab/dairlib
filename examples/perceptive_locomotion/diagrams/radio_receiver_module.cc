#include "radio_receiver_module.h"

#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/framework/diagram_builder.h"

#include "examples/Cassie/systems/cassie_out_to_radio.h"

namespace dairlib::perceptive_locomotion {

RadioReceiverModule::RadioReceiverModule(
    const std::string& cassie_out_channel,
    drake::lcm::DrakeLcmInterface* lcm) {

  drake::systems::DiagramBuilder<double> builder;

  auto cassie_out_sub = builder.AddSystem(
          drake::systems::lcm::LcmSubscriberSystem::Make<lcmt_cassie_out>(
              cassie_out_channel, lcm));

  auto radio_receiver = builder.AddSystem<systems::CassieOutToRadio>();
  auto velocity_commander = builder.AddSystem<VelocityCommander>();

  builder.Connect(*cassie_out_sub, *radio_receiver);
  builder.Connect(*radio_receiver, *velocity_commander);
  builder.ExportOutput(
      velocity_commander->get_output_port(),
      "vdes_xy"
  );

  builder.BuildInto(this);
}

}

