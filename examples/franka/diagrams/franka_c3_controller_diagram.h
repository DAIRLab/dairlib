#pragma once

#include <memory>
#include <utility>

#include <drake/common/drake_copyable.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/diagram.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/framework/system.h>
#include <drake/lcm/drake_lcm.h>
#include <drake/planning/robot_diagram_builder.h>
#include <drake/planning/robot_diagram.h>

namespace dairlib {
namespace examples {
namespace controllers {

class FrankaC3ControllerDiagram : public drake::systems::Diagram<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FrankaC3ControllerDiagram)

  /// @param[in] osc_gains_filename filepath containing the osc_running_gains.
  /// @param[in] osqp_settings filepath containing the osqp settings.
  FrankaC3ControllerDiagram(const std::string& controller_settings,
                              const std::string& lcm_channels,
                              drake::lcm::DrakeLcm* lcm);

  /// @return the input port for the plant state.
  const drake::systems::InputPort<double>& get_input_port_robot_state() const {
    return this->get_input_port(franka_state_port_);
  }

  /// @return the input port for the plant state.
  const drake::systems::InputPort<double>& get_input_port_object_state() const {
    return this->get_input_port(tray_state_port_);
  }

  /// @return the input port for the cassie_out struct (containing radio
  /// commands).
  const drake::systems::InputPort<double>& get_input_port_radio() const {
    return this->get_input_port(radio_port_);
  }

  /// @return the output port for the lcmt_robot_input message.
  const drake::systems::OutputPort<double>& get_output_port_mpc_plan() const {
    return this->get_output_port(mpc_plan_port_);
  }

 private:
  drake::multibody::MultibodyPlant<double>* plant_franka_;
  drake::multibody::MultibodyPlant<double>* plant_tray_;
//  std::unique_ptr<drake::multibody::MultibodyPlant<double>> plant_for_lcs_;
//  std::unique_ptr<drake::geometry::SceneGraph<double>> scene_graph_lcs_;

  // Storage for the diagram and its plant and scene graph.
  // After Build(), the `builder_` is set to nullptr.
  std::unique_ptr<drake::planning::RobotDiagram<double>> robot_diagram_for_lcs_;
  std::unique_ptr<drake::systems::Context<double>> robot_diagram_root_context_;
//  std::unique_ptr<drake::systems::DiagramBuilder<double>> plant_lcs_builder_;
//  drake::multibody::AddMultibodyPlantSceneGraphResult<double> pair_;
//  drake::multibody::MultibodyPlant<double>& plant_for_lcs_;
//  drake::geometry::SceneGraph<double>& scene_graph_lcs_;
  std::unique_ptr<drake::systems::Context<double>> plant_for_lcs_diagram_context_;
//  drake::multibody::MultibodyPlant<double>& plant_for_lcs_;
//  drake::geometry::SceneGraph<double>& scene_graph_lcs_;
  std::unique_ptr<drake::multibody::MultibodyPlant<drake::AutoDiffXd>> plant_for_lcs_autodiff_;
  std::unique_ptr<drake::systems::Context<double>> plant_franka_context_;
  std::unique_ptr<drake::systems::Context<double>> plant_tray_context_;
//  drake::systems::Context<double>& plant_for_lcs_context_;
  std::unique_ptr<drake::systems::Context<drake::AutoDiffXd>> plant_for_lcs_autodiff_context_;

  const drake::systems::InputPortIndex franka_state_port_ =
      drake::systems::InputPortIndex(0);
  const drake::systems::InputPortIndex tray_state_port_ =
      drake::systems::InputPortIndex(1);
  const drake::systems::InputPortIndex radio_port_ =
      drake::systems::InputPortIndex(2);
  const drake::systems::OutputPortIndex mpc_plan_port_ =
      drake::systems::OutputPortIndex(0);
};

}  // namespace controllers
}  // namespace examples
}  // namespace dairlib
