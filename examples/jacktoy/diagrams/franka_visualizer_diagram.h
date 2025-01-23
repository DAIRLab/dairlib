#pragma once

#include "drake/geometry/meshcat.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram.h"

namespace dairlib::jacktoy {

class FrankaVisualizerDiagram : public drake::systems::Diagram<double> {
 public:
  FrankaVisualizerDiagram();

 private:
  std::shared_ptr<drake::geometry::Meshcat> meshcat;
  drake::multibody::MultibodyPlant<double> plant{0.0};
  drake::multibody::MultibodyPlant<double> plant_franka{0.0};
  drake::multibody::MultibodyPlant<double> plant_jack{0.0};

  std::unique_ptr<drake::systems::Context<double>> franka_context;
  std::unique_ptr<drake::systems::Context<double>> jack_context;

  drake::systems::InputPortIndex input_port_franka_state_sub_;
  drake::systems::InputPortIndex input_port_object_state_sub_;
  drake::systems::InputPortIndex input_port_is_c3_mode_sub_;
  drake::systems::InputPortIndex input_port_c3_execution_trajectory_sub_actor_;
  drake::systems::InputPortIndex input_port_repos_execution_trajectory_sub_actor_;
  drake::systems::InputPortIndex input_port_trajectory_sub_actor_curr_;
  drake::systems::InputPortIndex input_port_trajectory_sub_object_curr_;
  drake::systems::InputPortIndex input_port_trajectory_sub_force_curr_;
  drake::systems::InputPortIndex input_port_dynamically_feasible_trajectory_sub_object_curr_;
  drake::systems::InputPortIndex input_port_dynamically_feasible_trajectory_sub_actor_curr_;
  drake::systems::InputPortIndex input_port_trajectory_sub_actor_best_;
  drake::systems::InputPortIndex input_port_trajectory_sub_object_best_;
  drake::systems::InputPortIndex input_port_trajectory_sub_force_best_;
  drake::systems::InputPortIndex input_port_dynamically_feasible_trajectory_sub_object_best_;
  drake::systems::InputPortIndex input_port_sample_location_sub_;
  drake::systems::InputPortIndex input_port_sample_buffer_sub_;
  drake::systems::InputPortIndex input_port_sample_costs_sub_;
  drake::systems::InputPortIndex input_port_c3_state_actual_sub_;
  drake::systems::InputPortIndex input_port_c3_state_target_sub_;
  drake::systems::InputPortIndex input_port_c3_final_state_target_sub_;

};

}