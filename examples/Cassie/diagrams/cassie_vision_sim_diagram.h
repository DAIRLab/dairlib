#pragma once

#include <memory>
#include <utility>

#include <drake/multibody/plant/multibody_plant.h>
#include "examples/Cassie/systems/sim_cassie_sensor_aggregator.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/system.h"

namespace dairlib {
namespace examples {

class CassieVisionSimDiagram : public drake::systems::Diagram<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CassieVisionSimDiagram)

  /// @param[in] urdf filepath containing the osc_running_gains.
  CassieVisionSimDiagram(
      std::unique_ptr<drake::multibody::MultibodyPlant<double>> plant,
      const std::string& urdf = "examples/Cassie/urdf/cassie_v2.urdf",
      bool visualize = false, double mu = 0.8);

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

  /// @return the output port for cassie out messages.
  const drake::systems::OutputPort<double>& get_cassie_out_output_port() const {
    return this->get_output_port(cassie_out_output_port_index_);
  }

  /// @return the camera image output port.
  const drake::systems::OutputPort<double>& get_camera_out_output_port() const {
    return this->get_output_port(camera_out_output_port_index_);
  }

  drake::multibody::MultibodyPlant<double>& get_plant() {
    return *plant_;
  }

 private:

  drake::multibody::MultibodyPlant<double>* plant_;
  const systems::SimCassieSensorAggregator* sensor_aggregator_;
  drake::geometry::SceneGraph<double>* scene_graph_;
  const int actuation_input_port_index_ = 0;
  const int radio_input_port_index_ = 1;
  const int state_output_port_index_ = 0;
  const int cassie_out_output_port_index_ = 1;
  const int camera_out_output_port_index_ = 2;
  const double actuator_delay = 5e-3;        // 5ms
  const double actuator_update_rate = 2e-3;  // 1ms

};
}  // namespace examples
}  // namespace dairlib
