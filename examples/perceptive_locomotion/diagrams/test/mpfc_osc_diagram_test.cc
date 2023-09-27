#include "examples/perceptive_locomotion/diagrams/mpfc_osc_diagram.h"
#include "examples/perceptive_locomotion/diagrams/hiking_sim_diagram.h"
#include "examples/Cassie/cassie_utils.h"
#include "examples/Cassie/cassie_fixed_point_solver.h"

#include "drake/systems/analysis/simulator.h"

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
  std::string camera_yaml = "examples/perceptive_locomotion/camera_calib/cassie_hardware.yaml";
  std::string terrain_yaml = "examples/perceptive_locomotion/terrains/stones.yaml";

  auto builder = drake::systems::DiagramBuilder<double>();

  auto osc_diagram = builder.AddSystem<MpfcOscDiagram>(
      plant, gains_file, gains_mpc_file, osqp_options
  );
  auto sim_diagram = builder.AddSystem<HikingSimDiagram>(
      terrain_yaml, camera_yaml
  );
  builder.Connect(
      sim_diagram->get_output_port_state(), osc_diagram->get_input_port_state()
  );
  builder.Connect(
      sim_diagram->get_output_port_lcm_radio(),
      osc_diagram->get_input_port_radio()
  );
  builder.Connect(
      osc_diagram->get_output_port_actuation(),
      sim_diagram->get_input_port_actuation()
  );

  auto diagram = builder.Build();
  DrawAndSaveDiagramGraph(*diagram, "../mpfc_with_sim");

  auto context = diagram->CreateDefaultContext();
  auto [q, v] = GetInitialCassieState(urdf, true, Vector3d::Zero(), 0.3, 0.95);
  auto& sim_plant = sim_diagram->get_plant();
  auto& plant_context = diagram->GetMutableSubsystemContext(
      sim_plant, context.get()
  );
  plant.SetPositions(&plant_context, q);
  plant.SetVelocities(&plant_context, v);

  drake::systems::Simulator<double> simulator(*diagram, std::move(context));

  simulator.set_publish_every_time_step(false);
  simulator.set_publish_at_initialization(false);
  simulator.set_target_realtime_rate(1.0);
  simulator.Initialize();
  simulator.AdvanceTo(1.0);
  return 0;
}
}

int main(int argc, char **argv) {
  return dairlib::perceptive_locomotion::DoMain();
}