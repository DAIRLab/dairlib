#include "examples/perceptive_locomotion/diagrams/mpfc_osc_diagram.h"
#include "examples/Cassie/cassie_utils.h"

namespace dairlib::perceptive_locomotion {

int DoMain() {

  const std::string urdf = "examples/Cassie/urdf/cassie_v2.urdf";
  drake::multibody::MultibodyPlant<double> plant(0.0);
  auto instance = AddCassieMultibody(
      &plant, nullptr, true, urdf, true, false);
  plant.Finalize();

  std::string gains_file = "examples/perceptive_locomotion/gains/osc_gains_simulation.yaml";
  std::string gains_mpc_file = "examples/perceptive_locomotion/gains/alip_minlp_gains_simulation.yaml";
  std::string osqp_options = "examples/perceptive_locomotion/gains/osqp_options_osc.yaml";
  auto osc_diagram =
      MpfcOscDiagram(plant, gains_file, gains_mpc_file, osqp_options);
  return 0;
}
}

int main(int argc, char **argv) {
  return dairlib::perceptive_locomotion::DoMain();
}