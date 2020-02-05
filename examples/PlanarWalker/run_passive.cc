#include <memory>

#include <gflags/gflags.h>
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/manipulation/util/sim_diagram_builder.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

namespace dairlib {
namespace {

// Simple example which simulates the (passive) Kneed walker.  Run drake-visualizer
// to see the animated result.

DEFINE_double(realtime_factor, .5,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

// Simulation parameters.
DEFINE_double(timestep, 1e-5, "The simulator time step (s)");
DEFINE_double(youngs_modulus, 1e8, "The contact model's Young's modulus (Pa)");
DEFINE_double(us, 0.7, "The static coefficient of friction");
DEFINE_double(ud, 0.7, "The dynamic coefficient of friction");
DEFINE_double(v_tol, 0.01,
              "The maximum slipping speed allowed during stiction (m/s)");
DEFINE_double(dissipation, 2, "The contact model's dissipation (s/m)");
DEFINE_double(contact_radius, 2e-4,
              "The characteristic scale of contact patch (m)");
DEFINE_string(simulation_type, "compliant", "The type of simulation to use: "
              "'compliant' or 'timestepping'");
DEFINE_double(dt, 1e-3, "The step size to use for "
              "'simulation_type=timestepping' (ignored for "
              "'simulation_type=compliant'");

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  drake::lcm::DrakeLcm lcm;
  auto tree = std::make_unique<RigidBodyTree<double>>();
  drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      "examples/PlanarWalker/PlanarWalker.urdf",
      drake::multibody::joints::kFixed, tree.get());

  drake::multibody::AddFlatTerrainToWorld(tree.get(), 100., 10.);  

//  manipulation::util::SimDiagramBuilder<double> builder;
  drake::systems::DiagramBuilder<double> builder;  

  //auto plant = biulder.template AddSystem<systems::RigidBodyPlant<double>>(std::move(tree));
  //systems::RigidBodyPlant<double>* plant = builder.AddPlant(std::move(tree));

  if (FLAGS_simulation_type != "timestepping")
    FLAGS_dt = 0.0;
  auto plant = builder.AddSystem<drake::systems::RigidBodyPlant<double>>(std::move(tree), FLAGS_dt);
  //auto plant = builder.AddSystem<systems::RigidBodyPlant<double>>(std::move(tree));
  //systems::RigidBodyPlant<double>* plant = builder.AddPlant(std::move(tree));

    // Note: this sets identical contact parameters across all object pairs:

  drake::systems::CompliantMaterial default_material;
  default_material.set_youngs_modulus(FLAGS_youngs_modulus)
      .set_dissipation(FLAGS_dissipation)
      .set_friction(FLAGS_us, FLAGS_ud);
  plant->set_default_compliant_material(default_material);
  drake::systems::CompliantContactModelParameters model_parameters;
  model_parameters.characteristic_radius = FLAGS_contact_radius;
  model_parameters.v_stiction_tolerance = FLAGS_v_tol;
  plant->set_contact_model_parameters(model_parameters);

  

  // Creates and adds LCM publisher for visualization.
  //builder.AddVisualizer(&lcm);
  auto visualizer = builder.AddSystem<drake::systems::DrakeVisualizer>(plant->get_rigid_body_tree(), &lcm);  
  // Raw state vector to visualizer.
  builder.Connect(plant->state_output_port(), visualizer->get_input_port(0));  

  auto diagram = builder.Build();

  drake::systems::Simulator<double> simulator(*diagram);
  drake::systems::Context<double>& context =
      diagram->GetMutableSubsystemContext(*plant, &simulator.get_mutable_context());

  if (FLAGS_simulation_type != "timestepping") {
    drake::systems::ContinuousState<double>& state = context.get_mutable_continuous_state(); 
    std::cout << "Continuous " << state.size() << std::endl;
    state[1] = 1.5;
    state[3] = 1;
    state[4] = 2;
  } else {
    std::cout << "ngroups "<< context.num_discrete_state_groups() <<  std::endl;
    drake::systems::BasicVector<double>& state = context.get_mutable_discrete_state(0); 
    std::cout << "Discrete " << state.size() << std::endl;
    state[1] = 1;
    state[3] = 1;
    state[4] = 2;    
  }

  int num_u = plant->get_num_actuators();
  auto zero_input = Eigen::MatrixXd::Zero(num_u,1);
  context.FixInputPort(0, zero_input);

  


  simulator.set_target_realtime_rate(FLAGS_realtime_factor);
  simulator.Initialize();
  //simulator.AdvanceTo(10);
  return 0;
}

}  // namespace
}  // namespace dairlib

int main(int argc, char* argv[]) {
  return dairlib::do_main(argc, argv);
}