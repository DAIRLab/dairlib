#include <drake/common/yaml/yaml_io.h>
#include <drake/systems/analysis/simulator.h>
#include <gflags/gflags.h>

#include "dairlib/lcmt_pd_config.hpp"
#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "examples/Cassie/cassie_utils.h"
#include "examples/Cassie/diagrams/cassie_sim_diagram.h"
#include "examples/Cassie/diagrams/osc_running_controller_diagram.h"
#include "systems/controllers/linear_controller.h"
#include "systems/controllers/pd_config_lcm.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

namespace dairlib {
using drake::multibody::MultibodyPlant;
using drake::systems::DiagramBuilder;

namespace examples {

int DoMain(int argc, char* argv[]) {
  DiagramBuilder<double> builder;
  std::string urdf = "examples/Cassie/urdf/cassie_v2.urdf";
  std::string osc_gains_filename = "examples/Cassie/osc_run/osc_running_gains.yaml";
  std::string osqp_settings =
      "examples/Cassie/osc_run/osc_running_qp_settings.yaml";
  std::unique_ptr<MultibodyPlant<double>> plant =
      std::make_unique<MultibodyPlant<double>>(1e-5);
  MultibodyPlant<double> controller_plant =
      MultibodyPlant<double>(1e-5);
  // Built the Cassie MBPs
  addCassieMultibody(&controller_plant, nullptr, true,
                     "examples/Cassie/urdf/cassie_v2_conservative.urdf",
                     false /*spring model*/, false /*loop closure*/);
  controller_plant.Finalize();
//  auto controller_context = controller_plant.CreateDefaultContext();

  auto sim_diagram = builder.AddSystem<examples::CassieSimDiagram>(
      std::move(plant), urdf, 0.4, 1e4, 1e2);
  MultibodyPlant<double>& new_plant = sim_diagram->get_plant();
  drake::yaml::YamlReadArchive::Options yaml_options;
  yaml_options.allow_yaml_with_no_cpp = true;
  OSCGains osc_gains = drake::yaml::LoadYamlFile<OSCGains>(FindResourceOrThrow(osc_gains_filename), {}, {}, yaml_options);
  OSCRunningGains osc_running_gains = drake::yaml::LoadYamlFile<OSCRunningGains>(FindResourceOrThrow(osc_gains_filename));
  auto controller_diagram =
      builder.AddSystem<examples::controllers::OSCRunningControllerDiagram>(
          controller_plant, osc_gains, osc_running_gains);

  builder.Connect(controller_diagram->get_control_output_port(),
                  sim_diagram->get_actuation_input_port());
  builder.Connect(sim_diagram->get_state_output_port(),
                  controller_diagram->get_state_input_port());
  builder.Connect(sim_diagram->get_cassie_out_output_port_index(),
                  controller_diagram->get_cassie_out_input_port());


  auto diagram = builder.Build();
  diagram->set_name("cassie_running_gym");
  DrawAndSaveDiagramGraph(*diagram);
  std::cout << "built diagram: " << std::endl;
  std::unique_ptr<Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  VectorXd x_init = VectorXd::Zero(45);
  x_init << 1, 0, 0, 0, 0, 0, 1, -0.0304885, 0, 0.466767, -1.15602, -0.037542,
      1.45243, -0.0257992, -1.59913, 0.0304885, 0, 0.466767, -1.15602,
      -0.0374859, 1.45244, -0.0259075, -1.59919, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(new_plant, diagram_context.get());
  drake::systems::Simulator<double> simulator(*diagram,
                                              std::move(diagram_context));
  new_plant.SetPositionsAndVelocities(&plant_context, x_init);
  //  auto sim = drake::systems::Simulator(diagram);
  std::cout << "advancing simulator: " << std::endl;
  simulator.AdvanceTo(5.0);
  return 0;
}
}}

int main(int argc, char* argv[]) {
  return dairlib::examples::DoMain(argc, argv);
}
