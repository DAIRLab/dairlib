#pragma once

// dairlib - perception
#include "systems/perception/elevation_mapping_system.h"

// drake
#include "drake/systems/framework/diagram.h"

namespace dairlib {
namespace perceptive_locomotion {

class PerceptionModuleDiagram : public drake::systems::Diagram<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PerceptionModuleDiagram);

  PerceptionModuleDiagram();

  const drake::systems::InputPort<double>& get_input_port_cassie_out() const {
   return get_input_port(input_port_cassie_out_);
  }
  const drake::systems::InputPort<double>& get_input_port_depth_image() const {
   return get_input_port(input_port_depth_image_);
  }
  const drake::systems::OutputPort<double>& get_output_port_state() const {
   return get_output_port(output_port_state_);
  }
  const drake::systems::OutputPort<double>& get_output_port_robot_output()
  const {
   return get_output_port(output_port_robot_output_);
  }
  const drake::systems::OutputPort<double>& get_output_port_elevation_map()
  const {
   return get_output_port(output_port_elevation_map_);
  }

 private:
  perception::elevation_mapping_params elevation_mapping_params_;

  drake::systems::InputPortIndex input_port_cassie_out_;
  drake::systems::InputPortIndex input_port_depth_image_;

  drake::systems::OutputPortIndex output_port_state_;
  drake::systems::OutputPortIndex output_port_robot_output_;
  drake::systems::OutputPortIndex output_port_elevation_map_;
};

} // namespace perceptive_locomotion
} // namespace dairlib
