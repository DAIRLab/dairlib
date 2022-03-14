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

  /// @param[in] urdf filepath containing the osc_running_gains.
  CassieSimDiagram(
      const std::string& urdf = "examples/Cassie/urdf/cassie_v2.urdf",
      double mu = 0.8, double stiffness = 1e4, double dissipation_rate = 1e2);

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
  const double actuator_delay = 3e-3; // 3ms
  const double actuator_update_rate =
  const double dt_ = 8e-5;
};

}  // namespace examples
}  // namespace dairlib
