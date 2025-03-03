#pragma once

#include "cassie_mpfc_diagram.h"
#include "hiking_sim_diagram.h"
#include "mpfc_osc_diagram.h"

#include "lcm/lcm_log_sink.h"

namespace dairlib::perceptive_locomotion {

/*!
 * Drake diagram with the MPFC, OSC, and a multibody plant sim.
 * Uses ground truth state and terrain information for the controller.
 */
class FullSimDiagram : public drake::systems::Diagram<double> {
 public:
  explicit FullSimDiagram(
      const std::string& mpc_gains_yaml,
      const std::string& terrain_yaml,
      const std::string& sim_params_yaml);

  void SetPlantInitialConditions(drake::systems::Diagram<double>* diagram,
                                 drake::systems::Context<double>* context);

  void SaveLcmLog(const std::string& fname);

  std::shared_ptr<drake::geometry::Meshcat> meshcat() {
    return meshcat_;
  }

  drake::math::RigidTransformd GetCassiePelvisPoseInWorld(
      const drake::systems::Context<double>& root_context) const;

 private:
  lcm::LcmLogSink lcm_log_sink{};

  drake::multibody::MultibodyPlant<double> plant{0.0};
  std::unique_ptr<drake::systems::Context<double>> plant_context;

  HikingSimDiagram* sim_diagram;

  std::shared_ptr<drake::geometry::Meshcat> meshcat_;
};

}
