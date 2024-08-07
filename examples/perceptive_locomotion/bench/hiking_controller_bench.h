#pragma once

#include "lcm/lcm_log_sink.h"
#include "systems/controllers/footstep_planning/mpfc_system_interface.h"

namespace dairlib::perceptive_locomotion {

template <mpfc MPC, mpfc_osc_diagram OSC>
class HikingControllerBench {
 public:
  HikingControllerBench(
      const std::string& terrain_yaml,
      const std::string& osc_gains_yaml,
      const std::string& qp_solver_options_yaml,
      const std::string& mpc_solver_options_yaml);

 private:
  lcm::LcmLogSink lcm_log_sink_{};
  std::unique_ptr<drake::systems::Diagram<double>> diagram_;
};

}