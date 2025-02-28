#include <gflags/gflags.h>

#include "examples/perceptive_locomotion/diagrams/sim_diagram.h"
#include "drake/systems/analysis/simulator.h"


namespace dairlib {
namespace perceptive_locomotion {


DEFINE_string(terrain, "examples/perceptive_locomotion/terrains/stones.yaml",
              "yaml file to load terrain from");

DEFINE_string(savefile, "../standalone_sim_log", "lcm log file to save");

int DoMain(int argc, char **argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);


  std::string terrain_yaml = FLAGS_terrain;

  std::string gains_mpc_file =
      "examples/perceptive_locomotion/gains/alip_s2s_mpfc_gains_simulation.yaml";

  std::string sim_params_file =
      "examples/perceptive_locomotion/standalone_sim_params.yaml";

  const auto sim_options =
      drake::yaml::LoadYamlFile<std::map<std::string, std::vector<double>>>(
          sim_params_file);

  drake::systems::DiagramBuilder<double> builder;

  auto sim_diagram = builder.AddSystem<SimDiagram>(gains_mpc_file, terrain_yaml, sim_params_file);

  auto diagram = builder.Build();
  diagram->set_name("mpfc_osc_with_sim");
  DrawAndSaveDiagramGraph(*diagram);

  auto context = diagram->CreateDefaultContext();

  sim_diagram->SetPlantInitialConditions(diagram.get(), context.get());

  drake::systems::Simulator<double> simulator(*diagram, std::move(context));

  simulator.set_publish_every_time_step(false);
  simulator.set_publish_at_initialization(false);
  if (sim_options.at("realtime_rate").front() > 0) {
    simulator.set_target_realtime_rate(sim_options.at("realtime_rate").front());
  }
  double end_time = sim_options.at("time").front();
  simulator.AdvanceTo(end_time);

  sim_diagram->SaveLcmLog(FLAGS_savefile);

  return 0;
}
}
}

int main(int argc, char **argv) {
  return dairlib::perceptive_locomotion::DoMain(argc, argv);
}