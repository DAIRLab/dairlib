#pragma once

#include <memory>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/system.h"

namespace dairlib {
namespace examples {

class CassieSimDiagram : public drake::systems::Diagram<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CassieSimDiagram)

  /// @param[in] osc_gains_filename filepath containing the osc_running_gains.
  /// @param[in] osqp_settings filepath containing the osqp settings.
  CassieSimDiagram(const std::string& osc_gains_filename,
                              const std::string& osqp_settings_filename);

  /// @return the input port for the actuation command.
  const drake::systems::InputPort<double>& get_actuation_input_port() const {
    return this->get_input_port(actuation_input_port_index_);
  }

  /// @return the input port for the radio struct.
  const drake::systems::InputPort<double>& get_radio_input_port() const {
    return this->get_input_port(radio_input_port_index_);
  }

  /// @return the output port for the plant state as an OutputVector.
  const drake::systems::OutputPort<double>& get_state_output_port() const {
    return this->get_output_port(state_output_port_index_);
  }

  /// @return the output port for the failure status of the controller.
  const drake::systems::OutputPort<double>& get_cassie_out_output_port_index()
      const {
    return this->get_output_port(cassie_out_output_port_index_);
  }

 private:
  const int actuation_input_port_index_ = 0;
  const int radio_input_port_index_ = 1;
  const int state_output_port_index_ = 0;
  const int cassie_out_output_port_index_ = 1;
};

}  // namespace examples
}  // namespace dairlib
