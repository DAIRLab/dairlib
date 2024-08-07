#include "examples/hiking_controller_bench/hiking_controller_bench.h"

#include "systems/controllers/footstep_planning/alip_mpfc_s2s_system.h"
#include "systems/controllers/footstep_planning/cf_mpfc_system.h"
#include "examples/perceptive_locomotion/diagrams/mpfc_osc_diagram.h"

namespace dairlib::perceptive_locomotion {

using systems::controllers::Alips2sMPFCSystem;
using systems::controllers::CFMPFCSystem;

int DoMain() {
  std::string base = "examples/hiking_controller_bench/";
  auto bench = HikingControllerBench<Alips2sMPFCSystem, MpfcOscDiagram>(
      base + "terrains/flat.yaml",
      base + "gains/osc_gains_alip_s2s_mpfc.yaml",
      base + "gains/alip_s2s_mpfc_gains.yaml",
      base + "gains/osqp_options_osc.yaml",
      base + "misc_params/camera_calib_sim.yaml",
      true
  );
  return 0;
}

}

int main(int argc, char* arvg[]) {
  return dairlib::perceptive_locomotion::DoMain();
}
