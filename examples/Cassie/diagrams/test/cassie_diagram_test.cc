#include "examples/Cassie/diagrams/alip_walking_controller_diagram.h"
#include "examples/Cassie/diagrams/cassie_vision_sim_diagram.h"
#include "examples/Cassie/cassie_utils.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/analysis/simulator.h"

namespace dairlib::examples::controllers {

using drake::multibody::MultibodyPlant;
using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
using Eigen::VectorXd;

int DoMain(int argc, char* argv[]){
  auto builder = DiagramBuilder<double>();
  std::string urdf = "examples/Cassie/urdf/cassie_v2.urdf";

  // Controller
  MultibodyPlant<double> plant_w_spr(0.0);
  addCassieMultibody(&plant_w_spr, nullptr, true, urdf, true, false );
  plant_w_spr.Finalize();
  std::string controller_gains =
      "examples/Cassie/osc/osc_walking_gains_alip.yaml";
  std::string osc_gains =
      "examples/Cassie/osc/solver_settings/osqp_options_walking.yaml";

  auto controller_diagram = builder.AddSystem<AlipWalkingControllerDiagram>(
      plant_w_spr, true, controller_gains, osc_gains);

  auto sim_plant = std::make_unique<MultibodyPlant<double>>(8e-5);
  auto sim_diagram = builder.AddSystem<CassieVisionSimDiagram>(
      std::move(sim_plant));
  auto& plant = sim_diagram->get_plant();

  builder.Connect(sim_diagram->get_state_output_port(),
                  controller_diagram->get_state_input_port());
  builder.Connect(controller_diagram->get_control_output_port(),
                  sim_diagram->get_actuation_input_port());
  auto diagram = builder.Build();
  Simulator<double> sim(*diagram);

  // Set initial state and fix radio port
  VectorXd x_init = VectorXd::Zero(45);
  x_init << 1, 0, 0, 0, 0, 0, 0.95, -0.0358636, 0, 0.67432, -1.688, -0.0458742, 1.90918,
           -0.0381073, -1.82312, 0.0358636, 0, 0.67432, -1.688, -0.0457885, 1.90919, -0.0382424, -1.82321,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  VectorXd radio = VectorXd::Zero(18);
  auto& plant_context =
      diagram->GetMutableSubsystemContext(plant, &sim.get_mutable_context());
  auto& sim_diagram_context =
      diagram->GetMutableSubsystemContext(*sim_diagram, &sim.get_mutable_context());
  auto& controller_diagram_context =
      diagram->GetMutableSubsystemContext(*controller_diagram, &sim.get_mutable_context());

  plant.SetPositionsAndVelocities(&plant_context, x_init);
  sim_diagram->get_radio_input_port().FixValue(&sim_diagram_context, radio);
  controller_diagram->get_radio_input_port().FixValue(&controller_diagram_context, radio);

  std::cout << "Initialize simulation\n";
  sim.Initialize();
  std::cout << "Start Simulation\n";
  sim.AdvanceTo(5.0);
  std::cout << "Simulation Complete\n";
  return 0;
}
}

int main(int argc, char* argv[]) {
  dairlib::examples::controllers::DoMain(argc, argv);
}

