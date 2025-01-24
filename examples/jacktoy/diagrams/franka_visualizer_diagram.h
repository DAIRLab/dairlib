#pragma once

#include "drake/geometry/meshcat.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram.h"
#include "examples/jacktoy/parameters/franka_lcm_channels.h"

namespace dairlib::jacktoy {

class FrankaVisualizerDiagram : public drake::systems::Diagram<double> {
 public:

  bool has_input_for_channel(const std::string& channel) const {
    return channel_to_input_port_map_.count(channel) > 0;
  }

  const drake::systems::InputPort<double>& get_input_port_for_channel(const std::string& channel_name) const {
    DRAKE_DEMAND(this->has_input_for_channel(channel_name));
    return get_input_port(channel_to_input_port_map_.at(channel_name));
  }

  std::vector<std::string> get_input_channels() const {
    std::vector<std::string> keys;
    for (const auto& [k, v] : channel_to_input_port_map_) {
      keys.push_back(k);
    }
    return keys;
  }

  std::shared_ptr<drake::geometry::Meshcat> get_meshcat() const {
    return meshcat;
  }

  FrankaVisualizerDiagram();

  std::string get_lcm_type(const std::string& channel_name) const {
    return channel_to_lcmtype_map_.at(channel_name);
  }

 private:

  void MakeChannelToInputPortMap(const FrankaLcmChannels& lcm_channel_params);
  void MakeChannelToLcmTypeMap(const FrankaLcmChannels& lcm_channel_params);

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

  std::unordered_map<std::string, drake::systems::InputPortIndex> channel_to_input_port_map_;
  std::unordered_map<std::string, std::string> channel_to_lcmtype_map_;



};

}