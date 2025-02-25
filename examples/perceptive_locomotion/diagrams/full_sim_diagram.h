#pragma once

#include "cassie_mpfc_diagram.h"
#include "hiking_sim_diagram.h"
#include "mpfc_osc_diagram.h"
#include "lcm/lcm_log_sink.h"

namespace dairlib::perceptive_locomotion {

class FullSimDiagram : public drake::systems::Diagram<double> {
 public:
  explicit FullSimDiagram(const std::string& terrain_yaml,
                          const std::string& sim_params_yaml);

  void SetPlantInitialConditions(drake::systems::Diagram<double>* diagram,
                                 drake::systems::Context<double>* context);

  void SaveLcmLog(const std::string& fname);

 private:
  lcm::LcmLogSink lcm_log_sink{};

  drake::multibody::MultibodyPlant<double> plant{0.0};
  std::unique_ptr<drake::systems::Context<double>> plant_context;

  drake::systems::InputPortIndex input_port_footholds_;
  drake::systems::OutputPortIndex outpput_port_elevation_map_;
  HikingSimDiagram* sim_diagram;

};

}
