#pragma once

#include "lcm/lcm_log_sink.h"
#include "systems/controllers/footstep_planning/mpfc_system_interface.h"
#include "external/drake/tools/install/libdrake/_virtual_includes/drake_shared_library/drake/systems/primitives/constant_vector_source.h"

namespace dairlib::perceptive_locomotion {

template <mpfc MPC, mpfc_osc_diagram OSC>
class HikingControllerBench {
 public:
  HikingControllerBench(
      const std::string& terrain_yaml,
      const std::string& osc_gains_yaml,
      const std::string& mpc_gains_yaml,
      const std::string& qp_solver_options_yaml,
      const std::string& camera_yaml,
      bool visualize = false);

 private:
  lcm::LcmLogSink lcm_log_sink_{};
  std::unique_ptr<drake::systems::Diagram<double>> diagram_;
  drake::systems::ConstantVectorSource<double>* vdes_source_;

};

}