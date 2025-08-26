#pragma once

#include "drake/common/yaml/yaml_read_archive.h"

namespace dairlib {
namespace examples {
namespace plate_balancing {

/**
 * @brief Configuration parameters for the plate balancing example.
 *
 * Holds all relevant configuration options for setting up and running the plate
 * balancing scenario.
 */
struct PlateBalancingConfig {
  int scene_index;
  bool include_end_effector_orientation;
  std::vector<std::string> file_names;
  std::string c3_controller_options_directory;
  std::string c3_scene_config_directory;
  std::string c3_osqp_settings_file;
  std::string c3_actor_trajectory_generator_config;
  std::string c3_object_trajectory_generator_config;
  std::string simulation_scene_config_directory;
  std::string simulation_config_file;
  std::string osc_contoller_config_file;
  std::string osc_osqp_setting_file;
  std::string lcm_hardware_settings_file;
  std::string lcm_simulation_settings_file;
  std::string plate_balancing_target_config_file;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(scene_index));
    a->Visit(DRAKE_NVP(include_end_effector_orientation));
    a->Visit(DRAKE_NVP(file_names));
    a->Visit(DRAKE_NVP(c3_controller_options_directory));
    a->Visit(DRAKE_NVP(c3_scene_config_directory));
    a->Visit(DRAKE_NVP(c3_osqp_settings_file));
    a->Visit(DRAKE_NVP(c3_actor_trajectory_generator_config));
    a->Visit(DRAKE_NVP(c3_object_trajectory_generator_config));
    a->Visit(DRAKE_NVP(simulation_scene_config_directory));
    a->Visit(DRAKE_NVP(simulation_config_file));
    a->Visit(DRAKE_NVP(osc_contoller_config_file));
    a->Visit(DRAKE_NVP(osc_osqp_setting_file));
    a->Visit(DRAKE_NVP(lcm_hardware_settings_file));
    a->Visit(DRAKE_NVP(lcm_simulation_settings_file));
    a->Visit(DRAKE_NVP(plate_balancing_target_config_file));
    DRAKE_ASSERT(scene_index < file_names.size());
  }

  std::string get_c3_controller_option_file() {
    return c3_controller_options_directory + file_names[scene_index];
  }

  std::string get_c3_scene_config_file() {
    return c3_scene_config_directory + file_names[scene_index];
  }

  std::string get_simulation_scene_config_file() {
    return simulation_scene_config_directory + file_names[scene_index];
  }
};
}  // namespace plate_balancing
}  // namespace examples
}  // namespace dairlib