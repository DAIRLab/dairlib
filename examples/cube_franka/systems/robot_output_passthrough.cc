#include "robot_output_passthrough.h"

namespace dairlib {
namespace systems {

using drake::multibody::MultibodyPlant;

RobotOutputPassthrough::RobotOutputPassthrough(
    const MultibodyPlant<double>& plant) {
  
  num_positions_ = plant.num_positions();
  num_velocities_ = plant.num_velocities();
  num_efforts_ = plant.num_actuators();

  this->DeclareAbstractInputPort("lcmt_robot_output",
                                 drake::Value<dairlib::lcmt_robot_output>{});
  this->DeclareAbstractOutputPort("lcmt_robot_output",
                                &RobotOutputPassthrough::Passthrough);
}

void RobotOutputPassthrough::Passthrough(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_robot_output* output) const {

  const drake::AbstractValue* input_ptr = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input_ptr != nullptr);
  const auto& input = input_ptr->get_value<dairlib::lcmt_robot_output>();

  output->utime = input.utime;
  output->num_positions = num_positions_;
  output->num_velocities = num_velocities_;
  output->num_efforts = num_efforts_;
  output->position_names.resize(num_positions_);
  output->velocity_names.resize(num_velocities_);
  output->effort_names.resize(num_efforts_);
  output->position.resize(num_positions_);
  output->velocity.resize(num_velocities_);
  output->effort.resize(num_efforts_);

  for (int i = 0; i < num_positions_; i++){
    output->position[i] = input.position[i];
  }
  for (int i = 0; i < num_velocities_; i++){
    output->velocity[i] = input.velocity[i];
  }
  for (int i = 0; i < num_efforts_; i++){
    output->effort[i] = input.effort[i];
  }
}

}  // namespace systems
}  // namespace dairlib