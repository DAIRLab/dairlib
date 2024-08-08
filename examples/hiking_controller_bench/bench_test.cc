#include "examples/hiking_controller_bench/hiking_controller_bench.h"

#include "systems/controllers/footstep_planning/alip_mpfc_s2s_system.h"
#include "systems/controllers/footstep_planning/cf_mpfc_system.h"
#include "examples/perceptive_locomotion/diagrams/mpfc_osc_diagram.h"
#include "examples/cf_mpfc/diagrams/cf_mpfc_osc_diagram.h"

namespace dairlib::perceptive_locomotion {

using Eigen::VectorXd;

using systems::controllers::Alips2sMPFCSystem;
using systems::controllers::CFMPFCSystem;

int DoMain() {
  std::string base = "examples/hiking_controller_bench/";

  VectorXd q = VectorXd::Zero(23);
  q << 1, 0, 0, 0, 0, 0, 0.95, -0.0320918, 0, 0.539399, -1.31373,
      -0.0410844, 1.61932, -0.0301574, -1.67739, 0.0320918, 0, 0.539399,
      -1.31373, -0.0404818, 1.61925, -0.0310551, -1.6785;

  VectorXd v = VectorXd::Zero(22);

  auto alip_bench = HikingControllerBench<Alips2sMPFCSystem, MpfcOscDiagram>(
      base + "terrains/flat.yaml",
      base + "gains/osc_gains_alip_s2s_mpfc.yaml",
      base + "gains/alip_s2s_mpfc_gains.yaml",
      base + "gains/osqp_options_osc.yaml",
      base + "misc_params/camera_calib_sim.yaml",
      true
  );

  auto cf_bench = HikingControllerBench<CFMPFCSystem, CfMpfcOscDiagram>(
      base + "terrains/flat.yaml",
      base + "gains/cf_mpfc_osc_gains.yaml",
      base + "gains/cf_mpfc_gains.yaml",
      base + "gains/osqp_options_osc.yaml",
      base + "misc_params/camera_calib_sim.yaml",
      true
  );

  cf_bench.Simulate(q, v, 1.0, 0.25, "../cf_bench_log");


  return 0;
}

}

int main(int argc, char* arvg[]) {
  return dairlib::perceptive_locomotion::DoMain();
}
