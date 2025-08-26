#include "robot_input_systems.h"
#include "robot_output_systems.h"
#include "systems/primitives/subvector_pass_through.h"

#include "drake/systems/lcm/lcm_interface_system.h"

namespace dairlib {
namespace systems {
namespace lcmt_systems {
/**
 * @brief Adds and connects the necessary systems for controlling a
 * MultibodyPlant via LCM.
 *
 * This function sets up two main connections:
 *   1. Connects the plant's state output to a RobotOutputSender and LCM
 * publisher, publishing messages on the specified lcmt_robot_output channel.
 *   2. Connects the plant's actuation input port to a RobotInputReceiver and
 * LCM receiver, receiving messages from the specified lcmt_robot_input channel.
 *
 * If an actuator delay is specified, a DiscreteTimeDelay block is inserted
 * between the input receiver and the plant. This delay also affects the efforts
 * sent to the RobotOutputSender if publish_efforts is true.
 *
 * @param builder The DiagramBuilder used to construct the system.
 * @param plant The MultibodyPlant to be controlled.
 * @param lcm The LcmInterfaceSystem for publishing and receiving LCM messages.
 * @param actuator_channel The LCM channel name for receiving lcmt_robot_input
 * messages.
 * @param state_channel The LCM channel name for publishing lcmt_robot_output
 * messages.
 * @param publish_rate The frequency (Hz) at which to publish state messages.
 * @param model_instance_index The model instance to control (default: -1 for
 * all).
 * @param publish_efforts If true, actuator efforts are included in the
 * published output.
 * @param actuator_delay The input delay (in seconds), discretized according to
 * publish_rate.
 * @return SubvectorPassThrough<double>* Pointer to the created
 * SubvectorPassThrough system.
 */
SubvectorPassThrough<double>* AddActuationRecieverAndStateSenderLcm(
    drake::systems::DiagramBuilder<double>* builder,
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::lcm::LcmInterfaceSystem* lcm, std::string actuator_channel,
    std::string state_channel, double publish_rate,
    drake::multibody::ModelInstanceIndex model_instance_index =
        drake::multibody::ModelInstanceIndex(-1),
    bool publish_efforts = true, double actuator_delay = 0);
}  // namespace lcmt_systems
}  // namespace systems
}  // namespace dairlib