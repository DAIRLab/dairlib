#include "examples/id_mpc/id_mpc_full_sim.h"
#include "systems/system_utils.h"
#include "drake/systems/analysis/simulator.h"


namespace dairlib::systems::controllers::id_mpc {

using drake::systems::Simulator;

const std::string mpc_gains = "examples/id_mpc/gains/mpc_gains_walking.yaml";
const std::string gait = "examples/id_mpc/gains/gait_params_walking.yaml";
const std::string solver_opts = "examples/id_mpc/gains/ncqp_opts.yaml";
const std::string pd_gains = "examples/id_mpc/gains/pd_gains_standing.yaml";
const std::string terrain =
    "examples/perceptive_locomotion/terrains/flat.yaml";
const std::string sim_opts =
    "examples/perceptive_locomotion/standalone_sim_params.yaml";

void DoMain() {

  IDMPCFullSim diagram(
      terrain, sim_opts, mpc_gains, pd_gains, gait, solver_opts);

  DrawAndSaveDiagramGraph(diagram, "sim_diagram");

  auto context = diagram.CreateDefaultContext();
  diagram.SetPlantInitialConditions(&diagram, context.get());

  Simulator<double> simulator(diagram, std::move(context));
  simulator.set_publish_every_time_step(false);
  simulator.set_publish_at_initialization(false);
  simulator.Initialize();

  simulator.AdvanceTo(10.0);
}

}

int main(int argc, char* argv []) {
  dairlib::systems::controllers::id_mpc::DoMain();
  return 0;
}