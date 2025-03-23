#pragma once

#include "cassie_mpfc_diagram.h"
#include "hiking_sim_diagram.h"
#include "mpfc_osc_diagram.h"
#include "perception_module_diagram.h"

#include "lcm/lcm_log_sink.h"

namespace dairlib::perceptive_locomotion {

/*!
 * Drake diagram including all non-python components of the perceptive
 * locomotion stack, including MPFC, OSC, Drake simulator, Cassie State
 * estimator, and elevation mapping.
 *
 * Simulations incorporating this diagram must provide the convex polygon
 * footholds via the supplied input port
 */
class PerceptiveFullSimDiagram : public drake::systems::Diagram<double> {
 public:
  explicit PerceptiveFullSimDiagram(
      const std::string& mpc_gains_yaml,
      const std::string& terrain_yaml,
      const std::string& sim_params_yaml,
      const std::string& elevation_mapping_params_yaml="");

  void SetPlantInitialConditions(drake::systems::Diagram<double>* diagram,
                                 drake::systems::Context<double>* context);

  void SaveLcmLog(const std::string& fname);

  const drake::systems::InputPort<double>& get_input_port_footholds() const {
    return get_input_port(input_port_footholds_);
  }
  /*
   * Input port for the processed (segmented) elevation map
   */
  const drake::systems::InputPort<double>& get_input_port_grid_map() const {
    return get_input_port(input_port_grid_map_);
  }

  /*
   * input port for the raw elevation map
   */
  const drake::systems::OutputPort<double>& get_output_port_grid_map() const {
    return get_output_port(output_port_elevation_map_);
  }

  std::shared_ptr<drake::geometry::Meshcat> meshcat() {
    return meshcat_;
  }

  drake::math::RigidTransformd GetCassiePelvisPoseInWorld(
      const drake::systems::Context<double>& root_context) const;

 private:
  lcm::LcmLogSink lcm_log_sink{};

  drake::multibody::MultibodyPlant<double> plant{0.0};
  std::unique_ptr<drake::systems::Context<double>> plant_context;

  drake::systems::InputPortIndex input_port_footholds_;
  drake::systems::InputPortIndex input_port_grid_map_;

  drake::systems::OutputPortIndex output_port_elevation_map_;
  HikingSimDiagram* sim_diagram;
  PerceptionModuleDiagram* perception;

  std::shared_ptr<drake::geometry::Meshcat> meshcat_;
};

}
