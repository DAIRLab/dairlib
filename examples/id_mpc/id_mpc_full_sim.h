#pragma once

#include "lcm/lcm_log_sink.h"
#include "examples/perceptive_locomotion/diagrams/hiking_sim_diagram.h"
#include "systems/controllers/id_mpc/systems/id_mpc_walking_system.h"
#include "systems/controllers/id_mpc/walking/walking_reference_system.h"

#include "drake/systems/framework/diagram.h"

namespace dairlib::systems::controllers::id_mpc {

class IDMPCFullSim : public drake::systems::Diagram<double> {

 public:
  IDMPCFullSim(const std::string& terrain,
               const std::string& sim_opts,
               const std::string& mpc_gains_yaml,
               const std::string& pd_gains_yaml,
               const std::string& gait_yaml,
               const std::string& solver_options_yaml);

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
  std::unique_ptr<drake::systems::Context<double>> context_for_reference_system;

  perceptive_locomotion::HikingSimDiagram* sim_diagram;
  WalkingReferenceSystem* ref_gen;
  std::shared_ptr<drake::geometry::Meshcat> meshcat_;

};
}