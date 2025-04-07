#pragma once

#include <memory>
#include <utility>

#include <drake/lcm/drake_lcm.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/planning/robot_diagram.h>

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/system.h"

namespace dairlib {
namespace examples {

class FrankaSimDiagram : public drake::systems::Diagram<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FrankaSimDiagram)

  /// @param[in] urdf filepath containing the osc_running_gains.
  FrankaSimDiagram(
      std::unique_ptr<drake::multibody::MultibodyPlant<double>> plant,
      drake::lcm::DrakeLcm* lcm);

  /// @return the input port for the actuation command.
  const drake::systems::InputPort<double>& get_input_port_actuation() const {
    return this->get_input_port(actuation_port_);
  }

  /// @return the output port for the plant state as an OutputVector.
  const drake::systems::OutputPort<double>& get_output_port_tray_state() const {
    return this->get_output_port(tray_state_port_);
  }

  /// @return the output port for the plant state as an OutputVector.
  const drake::systems::OutputPort<double>& get_output_port_franka_state()
      const {
    return this->get_output_port(franka_state_port_);
  }

  const drake::multibody::MultibodyPlant<double>& get_plant() { return *plant_; }

 private:
  drake::multibody::MultibodyPlant<double>* plant_;
  drake::geometry::SceneGraph<double>* scene_graph_;

  drake::systems::InputPortIndex actuation_port_;
  drake::systems::OutputPortIndex tray_state_port_;
  drake::systems::OutputPortIndex franka_state_port_;
};

}  // namespace examples
}  // namespace dairlib
