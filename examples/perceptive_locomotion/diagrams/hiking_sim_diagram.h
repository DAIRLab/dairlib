#pragma once

#include "drake/systems/framework/diagram.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace dairlib::perceptive_locomotion {

class HikingSimDiagram : public drake::systems::Diagram<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(HikingSimDiagram)
  HikingSimDiagram(const std::string& terrain_yaml,
                   const std::string& camera_pose_yaml);

  const drake::systems::InputPort<double>& get_input_port_actuation() const {
    return get_input_port(input_port_control_);
  }
  const drake::systems::InputPort<double>& get_input_port_radio() const {
    return get_input_port(input_port_radio_);
  }
  const drake::systems::OutputPort<double>& get_output_port_state() const {
    return get_output_port(output_port_state_);
  }
  const drake::systems::OutputPort<double>& get_output_port_cassie_out() const {
    return get_output_port(output_port_cassie_out_);
  }
  const drake::systems::OutputPort<double>& get_output_port_lcm_radio() const {
    return get_output_port(output_port_lcm_radio_);
  }

  [[nodiscard]] drake::multibody::MultibodyPlant<double>& get_plant() const {
    return *plant_;
  }

 private:
  drake::multibody::MultibodyPlant<double>* plant_;
  drake::geometry::SceneGraph<double>* scene_graph_;

  drake::systems::InputPortIndex input_port_control_;
  drake::systems::InputPortIndex input_port_radio_;
  drake::systems::OutputPortIndex output_port_state_;
  drake::systems::OutputPortIndex output_port_cassie_out_;
  drake::systems::OutputPortIndex output_port_lcm_radio_;
};

}