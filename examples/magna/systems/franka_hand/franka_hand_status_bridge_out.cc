#include "examples/magna/systems/franka_hand/franka_hand_status_bridge_out.h"

#include "dairlib/lcmt_robot_output.hpp"
#include "drake/lcmt_schunk_wsg_status.hpp"

namespace dairlib {
namespace examples {
namespace magna {
namespace systems {
namespace franka_hand {

using drake::systems::Context;

FrankaHandStatusBridgeOut::FrankaHandStatusBridgeOut() {
  franka_hand_state_input_port_ =
      this->DeclareAbstractInputPort("lcmt_schunk_wsg_status",
                                     drake::Value<drake::lcmt_schunk_wsg_status>{})
          .get_index();
  franka_hand_state_output_port_ =
      this->DeclareAbstractOutputPort("lcmt_robot_output",
                                      &FrankaHandStatusBridgeOut::OutputFrankaHandState)
          .get_index();
}

void FrankaHandStatusBridgeOut::OutputFrankaHandState(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_robot_output* output) const {
  const drake::AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto& input_msg = input->get_value<drake::lcmt_schunk_wsg_status>();

  // Convert from schunk status (gripper width in mm) to robot output format
  // Each finger gets half the gripper width
  double finger_position = input_msg.actual_position_mm / 2000.0;  // mm to m, and divide by 2

  output->utime = input_msg.utime;
  output->num_positions = 2;
  output->num_velocities = 2;
  output->num_efforts = 2;

  output->position_names.resize(2);
  output->position_names[0] = "panda_finger_joint1";
  output->position_names[1] = "panda_finger_joint2";
  
  output->position.resize(2);
  output->position[0] = finger_position;
  output->position[1] = finger_position;

  output->velocity_names.resize(2);
  output->velocity_names[0] = "panda_finger_joint1dot";
  output->velocity_names[1] = "panda_finger_joint2dot";
  
  output->velocity.resize(2);
  output->velocity[0] = input_msg.actual_speed_mm_per_s / 2000.0;  // mm/s to m/s, and divide by 2
  output->velocity[1] = input_msg.actual_speed_mm_per_s / 2000.0;

  output->effort_names.resize(2);
  output->effort_names[0] = "panda_finger_joint1";
  output->effort_names[1] = "panda_finger_joint2";
  
  output->effort.resize(2);
  output->effort[0] = input_msg.actual_force / 2.0;  // Divide force by 2 for each finger
  output->effort[1] = input_msg.actual_force / 2.0;

  output->imu_accel[0] = 0.0;
  output->imu_accel[1] = 0.0;
  output->imu_accel[2] = 0.0;
}

}  // namespace franka_hand
}  // namespace systems
}  // namespace magna
}  // namespace examples
}  // namespace dairlib