#pragma once

#include <drake/common/drake_copyable.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/diagram.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/framework/system.h>
#include <drake/lcm/drake_lcm.h>

#include "systems/controllers/osc/joint_space_tracking_data.h"
#include "systems/controllers/osc/relative_translation_tracking_data.h"
#include "systems/controllers/osc/rot_space_tracking_data.h"
#include "systems/controllers/osc/external_force_tracking_data.h"
#include "systems/controllers/osc/trans_space_tracking_data.h"

namespace dairlib {
namespace examples {
namespace controllers {

using systems::controllers::JointSpaceTrackingData;
using systems::controllers::RotTaskSpaceTrackingData;
using systems::controllers::TransTaskSpaceTrackingData;
using systems::controllers::ExternalForceTrackingData;

class FrankaOSCControllerDiagram : public drake::systems::Diagram<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FrankaOSCControllerDiagram)

  /// @param[in] osc_gains_filename filepath containing the osc_running_gains.
  /// @param[in] osqp_settings filepath containing the osqp settings.
  FrankaOSCControllerDiagram(const std::string& controller_settings,
                            const std::string& lcm_channels,
                            drake::lcm::DrakeLcm* lcm);

  /// @return the input port for the plant state.
  const drake::systems::InputPort<double>& get_input_port_robot_state() const {
    return this->get_input_port(franka_state_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_end_effector_position() const {
    return this->get_input_port(end_effector_position_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_end_effector_orientation() const {
    return this->get_input_port(end_effector_orientation_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_end_effector_force() const {
    return this->get_input_port(end_effector_force_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_radio() const {
    return this->get_input_port(radio_port_);
  }

  /// @return the output port for the lcmt_robot_input message.
  const drake::systems::OutputPort<double>& get_output_port_robot_input() const {
    return this->get_output_port(robot_input_port_);
  }

 private:
  drake::multibody::MultibodyPlant<double>* plant_;
  std::unique_ptr<drake::systems::Context<double>> plant_context_;

  const drake::systems::InputPortIndex franka_state_port_ =
      drake::systems::InputPortIndex(0);
  const drake::systems::InputPortIndex end_effector_position_port_ =
      drake::systems::InputPortIndex(1);
  const drake::systems::InputPortIndex end_effector_orientation_port_ =
      drake::systems::InputPortIndex(2);
  const drake::systems::InputPortIndex end_effector_force_port_ =
      drake::systems::InputPortIndex(3);
  const drake::systems::InputPortIndex radio_port_ =
      drake::systems::InputPortIndex(4);
  const drake::systems::OutputPortIndex robot_input_port_ =
      drake::systems::OutputPortIndex(0);
};

}  // namespace controllers
}  // namespace examples
}  // namespace dairlib
