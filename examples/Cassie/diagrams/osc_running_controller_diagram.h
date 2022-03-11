#pragma once

#include <memory>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/system.h"

namespace dairlib {
namespace examples {
namespace controllers {

class OSCRunningControllerDiagram : public drake::systems::Diagram<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(OSCRunningControllerDiagram)

  /// @param[in] osc_gains_filename filepath containing the osc_running_gains.
  /// @param[in] osqp_settings filepath containing the osqp settings.
  OSCRunningControllerDiagram(const std::string& osc_gains_filename,
                              const std::string& osqp_settings_filename);

  /// @return the input port for the plant state.
  const drake::systems::InputPort<double>& get_state_input_port() const {
    return this->get_input_port(state_input_port_index_);
  }

  /// @return the input port for the cassie_out struct (containing radio commands).
  const drake::systems::InputPort<double>& get_cassie_out_input_port() const {
    return this->get_input_port(cassie_out_input_port_index_);
  }

  /// @return the output port for the controller torques.
  const drake::systems::OutputPort<double>& get_control_output_port() const {
    return this->get_output_port(control_output_port_index_);
  }

  /// @return the output port for the failure status of the controller.
  const drake::systems::OutputPort<double>& get_controller_failure_output_port()
      const {
    return this->get_output_port(controller_failure_port_index_);
  }

 private:
  const int state_input_port_index_ = 0;
  const int cassie_out_input_port_index_ = 1;
  const int control_output_port_index_ = 0;
  const int controller_failure_port_index_ = 1;

  const std::string control_channel_name_ = "OSC_RUNNING";
};

}  // namespace controllers
}  // namespace examples
}  // namespace dairlib
