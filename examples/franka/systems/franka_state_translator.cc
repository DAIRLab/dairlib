#include "examples/franka/systems/franka_state_translator.h"

#include <dairlib/lcmt_robot_input.hpp>

#include "lcmtypes/drake/lcmt_panda_status.hpp"
#include "systems/framework/output_vector.h"

namespace dairlib {

using drake::systems::BasicVector;
using drake::systems::Context;
using systems::TimestampedVector;

namespace systems {

FrankaStateOutTranslator::FrankaStateOutTranslator(
    std::vector<std::string> joint_position_names,
    std::vector<std::string> joint_velocity_names,
    std::vector<std::string> joint_actuator_names) {
  this->set_name("franka_state_translator");

  panda_status_ =
      this->DeclareAbstractInputPort("franka_status",
                                     drake::Value<drake::lcmt_panda_status>())
          .get_index();
  DRAKE_DEMAND(joint_position_names.size() == kNumFrankaJoints);
  DRAKE_DEMAND(joint_velocity_names.size() == kNumFrankaJoints);
  DRAKE_DEMAND(joint_actuator_names.size() == kNumFrankaJoints);
  auto default_robot_output_msg = dairlib::lcmt_robot_output();
  default_robot_output_msg.position_names = joint_position_names;
  default_robot_output_msg.velocity_names = joint_velocity_names;
  default_robot_output_msg.effort_names = joint_actuator_names;
  state_output_ = this->DeclareAbstractOutputPort(
                          "lcmt_robot_output", default_robot_output_msg,
                          &FrankaStateOutTranslator::OutputFrankaState)
                      .get_index();
  start_ = std::chrono::steady_clock::now();
}

void FrankaStateOutTranslator::OutputFrankaState(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_robot_output* output) const {
  const auto& panda_status =
      this->EvalInputValue<drake::lcmt_panda_status>(context, panda_status_);
  output->utime =
      (duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start_)).count();
  output->position = panda_status->joint_position;
  output->num_positions = panda_status->num_joints;
  output->num_velocities = panda_status->num_joints;
  output->num_efforts = panda_status->num_joints;
  output->velocity = panda_status->joint_velocity;
  output->effort = panda_status->joint_torque;
}

FrankaEffortsInTranslator::FrankaEffortsInTranslator() {
  this->set_name("franka_command_translator");

  robot_input_ =
      this->DeclareAbstractInputPort("franka_state",
                                     drake::Value<dairlib::lcmt_robot_input>())
          .get_index();
  panda_status_ =
      this->DeclareAbstractInputPort("franka_status",
                                     drake::Value<drake::lcmt_panda_status>())
          .get_index();
  franka_command_output_ =
      this->DeclareAbstractOutputPort(
              "lcmt_panda_command",
              &FrankaEffortsInTranslator::OutputFrankaCommand)
          .get_index();
}

void FrankaEffortsInTranslator::OutputFrankaCommand(
    const drake::systems::Context<double>& context,
    drake::lcmt_panda_command* output) const {
  const auto& robot_input =
      this->EvalInputValue<dairlib::lcmt_robot_input>(context, robot_input_);
  const auto& panda_status =
      this->EvalInputValue<drake::lcmt_panda_status>(context, panda_status_);
  DRAKE_DEMAND(robot_input->efforts.size() == kNumFrankaJoints);
  output->utime = panda_status->utime;
  output->num_joint_torque = robot_input->efforts.size();
  output->joint_torque = robot_input->efforts;
  output->control_mode_expected = drake::lcmt_panda_status::CONTROL_MODE_TORQUE;
}

}  // namespace systems
}  // namespace dairlib
