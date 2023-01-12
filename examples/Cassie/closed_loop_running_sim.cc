#include <drake/systems/analysis/simulator.h>
#include <gflags/gflags.h>
#include <iostream>

#include "examples/Cassie/cassie_utils.h"
#include "examples/Cassie/diagrams/cassie_sim_diagram.h"
#include "examples/Cassie/diagrams/osc_running_controller_diagram.h"
#include "examples/Cassie/diagrams/osc_walking_controller_diagram.h"
#include "systems/controllers/pd_config_lcm.h"
#include "systems/system_utils.h"

#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/zero_order_hold.h"

namespace dairlib {
using drake::multibody::MultibodyPlant;
using drake::systems::DiagramBuilder;

namespace examples {

int DoMain(int argc, char* argv[]) {
  DiagramBuilder<double> builder;
  std::string urdf = "examples/Cassie/urdf/cassie_v2_conservative.urdf";
  std::string osc_running_gains = "examples/Cassie/osc_run/osc_running_gains.yaml";
  std::string osc_walking_gains = "examples/Cassie/osc/osc_walking_gains_alip.yaml";
  std::string osqp_settings =
      "examples/Cassie/osc_run/osc_running_qp_settings.yaml";
  std::unique_ptr<MultibodyPlant<double>> plant =
      std::make_unique<MultibodyPlant<double>>(1e-3);
  MultibodyPlant<double> controller_plant =
      MultibodyPlant<double>(1e-3);
  // Built the Cassie MBPs
  AddCassieMultibody(&controller_plant, nullptr, true,
                     "examples/Cassie/urdf/cassie_v2_conservative.urdf",
                     false /*spring model*/, false /*loop closure*/);
  controller_plant.Finalize();

  auto sim_diagram = builder.AddSystem<examples::CassieSimDiagram>(
      std::move(plant), urdf, 0.4);
  MultibodyPlant<double>& new_plant = sim_diagram->get_plant();
  auto controller_diagram =
      builder.AddSystem<examples::controllers::OSCRunningControllerDiagram>(
          controller_plant, osc_running_gains, osqp_settings);
//  auto controller_diagram =
//      builder.AddSystem<examples::controllers::OSCWalkingControllerDiagram>(
//          controller_plant, true, osc_walking_gains, osqp_settings);

  builder.Connect(controller_diagram->get_output_port_robot_input(),
                  sim_diagram->get_input_port_actuation());
  builder.Connect(sim_diagram->get_output_port_state(),
                  controller_diagram->get_input_port_state());
//  builder.Connect(sim_diagram->get_output_port_cassie_out(),
//                  controller_diagram->get_cassie_out_input_port());

  auto diagram = builder.Build();
  diagram->set_name("cassie_running_gym");
  DrawAndSaveDiagramGraph(*diagram);
  std::cout << "built diagram: " << std::endl;
  std::unique_ptr<Context<double>> diagram_context =
      diagram->CreateDefaultContext();

  VectorXd x_init = VectorXd::Zero(45);
  x_init << 1, 0, 0, 0, 0, 0, 0.85, -0.0358636, 0, 0.67432, -1.588, -0.0458742, 1.90918,
      -0.0381073, -1.82312, 0.0358636, 0, 0.67432, -1.588, -0.0457885, 1.90919, -0.0382424, -1.82321,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(new_plant, diagram_context.get());
  drake::systems::Simulator<double> simulator(*diagram,
                                              std::move(diagram_context));
  Context<double>& simulator_context = diagram->GetMutableSubsystemContext(*sim_diagram, &simulator.get_mutable_context());
  Context<double>& controller_context = diagram->GetMutableSubsystemContext(*controller_diagram, &simulator.get_mutable_context());

  sim_diagram->get_input_port_radio().FixValue(&simulator_context, Eigen::VectorXd::Zero(18));
  controller_diagram->get_input_port_radio().FixValue(&controller_context, Eigen::VectorXd::Zero(18));

  new_plant.SetPositionsAndVelocities(&plant_context, x_init);
  simulator.Initialize();
  //  auto sim = drake::systems::Simulator(diagram);
  std::cout << "advancing simulator: " << std::endl;
  simulator.AdvanceTo(5.0);
}
}}

int main(int argc, char* argv[]) {
  return dairlib::examples::DoMain(argc, argv);
}
