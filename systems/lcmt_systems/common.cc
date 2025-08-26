#include "common.h"

#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/discrete_time_delay.h"

using drake::multibody::MultibodyPlant;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;

namespace dairlib {
namespace systems {
namespace lcmt_systems {

SubvectorPassThrough<double>* AddActuationRecieverAndStateSenderLcm(
    drake::systems::DiagramBuilder<double>* builder,
    const MultibodyPlant<double>& plant,
    drake::systems::lcm::LcmInterfaceSystem* lcm, std::string actuator_channel,
    std::string state_channel, double publish_rate,
    drake::multibody::ModelInstanceIndex model_instance_index,
    bool publish_efforts, double actuator_delay) {
  // Create LCM input for actuators
  auto input_sub =
      builder->AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_robot_input>(
          actuator_channel, lcm));
  auto input_receiver = builder->AddSystem<RobotInputConsumer>(plant);
  auto passthrough = builder->AddSystem<SubvectorPassThrough>(
      input_receiver->get_output_port(0).size(), 0,
      plant.get_actuation_input_port().size());
  builder->Connect(*input_sub, *input_receiver);
  builder->Connect(*input_receiver, *passthrough);

  // Create LCM output for state and efforts
  auto state_pub =
      builder->AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_output>(
          state_channel, lcm, 1.0 / publish_rate));
  auto state_sender =
      builder->AddSystem<RobotOutputGenerator>(plant, model_instance_index, publish_efforts);
  builder->Connect(plant.get_state_output_port(model_instance_index),
                   state_sender->get_input_port_state());

  // Add delay, if used, and associated connections
  if (actuator_delay > 0) {
    auto discrete_time_delay =
        builder->AddSystem<drake::systems::DiscreteTimeDelay>(
            1.0 / publish_rate, actuator_delay * publish_rate,
            plant.num_actuators());
    builder->Connect(*passthrough, *discrete_time_delay);
    builder->Connect(discrete_time_delay->get_output_port(),
                     plant.get_actuation_input_port());

    if (publish_efforts) {
      builder->Connect(discrete_time_delay->get_output_port(),
                       state_sender->get_input_port_effort());
    }
  } else {
    builder->Connect(passthrough->get_output_port(),
                     plant.get_actuation_input_port());
    if (publish_efforts) {
      builder->Connect(passthrough->get_output_port(),
                       state_sender->get_input_port_effort());
    }
  }

  builder->Connect(*state_sender, *state_pub);

  return passthrough;
}

}  // namespace lcmt_systems
}  // namespace systems
}  // namespace dairlib