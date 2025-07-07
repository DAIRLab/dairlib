#include "systems/lcmt_generators/robot_state_generator.h"

#include "systems/framework/timestamped_vector.h"

using dairlib::systems::TimestampedVector;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiagramBuilder;
using drake::systems::lcm::LcmPublisherSystem;

namespace dairlib {
namespace systems {
namespace lcmt_generators {

RobotStateGenerator::RobotStateGenerator(std::vector<std::string> state_names,
                                         bool is_timestamped_robot_state)
    : n_x_(state_names.size()),
      is_timestamped_robot_state_(is_timestamped_robot_state) {
  if (is_timestamped_robot_state_)
    robot_state_input_ = this->DeclareVectorInputPort(
                                 "robot_state", TimestampedVector<double>(n_x_))
                             .get_index();
  else
    robot_state_input_ =
        this->DeclareVectorInputPort("robot_state", BasicVector<double>(n_x_))
            .get_index();

  drake::lcmt_robot_state robot_state = drake::lcmt_robot_state();
  robot_state.num_joints = n_x_;
  robot_state.utime = 0;
  robot_state.joint_position = std::vector<float>(n_x_);
  robot_state.joint_name = state_names;

  lcmt_robot_state_output_ =
      this->DeclareAbstractOutputPort("lcmt_robot_state", robot_state,
                                      &RobotStateGenerator::GenerateRobotState)
          .get_index();
}

void RobotStateGenerator::GenerateRobotState(
    const drake::systems::Context<double>& context,
    drake::lcmt_robot_state* output) const {
  const auto target_state = this->EvalVectorInput(context, robot_state_input_);
  if (dynamic_cast<const TimestampedVector<double>*>(target_state) != nullptr) {
    DRAKE_DEMAND(target_state->size() == n_x_ + 1);
  } else {
    DRAKE_DEMAND(target_state->size() == n_x_);
  }
  output->utime = context.get_time() * 1e6;
  for (int i = 0; i < n_x_; ++i) {
    output->joint_position[i] = static_cast<float>(target_state->GetAtIndex(i));
  }
}

// Adds this publisher and an LCM publisher system to the diagram builder.
LcmPublisherSystem* RobotStateGenerator::AddLcmPublisherToBuilder(
    DiagramBuilder<double>& builder, std::vector<std::string> state_names,
    bool is_timestamped_robot_state,
    const drake::systems::OutputPort<double>& robot_state_input_port,
    const std::string& channel, drake::lcm::DrakeLcmInterface* lcm,
    const drake::systems::TriggerTypeSet& publish_triggers,
    double publish_period, double publish_offset) {
  // Add and connect the RobotStateGenerator system.
  auto state_publisher = builder.AddSystem<RobotStateGenerator>(
      state_names, is_timestamped_robot_state);
  builder.Connect(robot_state_input_port,
                  state_publisher->get_input_port_robot_state());

  // Add and connect the LCM publisher system.
  auto lcm_state_publisher =
      builder.AddSystem(LcmPublisherSystem::Make<drake::lcmt_robot_state>(
          channel, lcm, publish_triggers, publish_period, publish_offset));
  builder.Connect(state_publisher->get_output_port_lcmt_robot_state(),
                  lcm_state_publisher->get_input_port());
  return lcm_state_publisher;
}

}  // namespace lcmt_generators
}  // namespace systems
}  // namespace dairlib
