#include <iostream>
#include <memory>

#include <gflags/gflags.h>

#include "common/find_resource.h"
#include "multibody/multibody_utils.h"
#include "parameters/trifinger_lcm_channels.h"
#include "parameters/trifinger_sim_params.h"
#include "systems/primitives/subvector_pass_through.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"

#include "drake/common/yaml/yaml_io.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"

DEFINE_string(sim_parameters,
              "examples/trifinger/parameters/trifinger_sim_params.yaml",
              "Filepath to simulation configs");

DEFINE_string(lcm_channels,
              "examples/trifinger/parameters/lcm_channels_simulation.yaml",
              "Filepath containing lcm channels");

namespace dairlib {

using drake::geometry::SceneGraph;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::Context;
using drake::systems::DiagramBuilder;
using drake::systems::Simulator;

using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::VectorXd;

int SimulateTrifinger(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  drake::yaml::LoadYamlOptions yaml_options;
  yaml_options.allow_yaml_with_no_cpp = true;

  auto sim_params =
      drake::yaml::LoadYamlFile<TrifingerSimParams>(FLAGS_sim_parameters);

  auto lcm_channels =
      drake::yaml::LoadYamlFile<TrifingerLcmChannels>(FLAGS_lcm_channels);
  DiagramBuilder<double> builder;

  // Creates default plant and scene graph.
  const double time_step = sim_params.dt;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, time_step);
  scene_graph.set_name("scene_graph");

  // Adds the URDF model to the trifinger plant.
  multibody::AddFlatTerrain(&plant, &scene_graph, .8, .8, {0, 0, 1}, false);
  std::string trifinger_urdf = sim_params.trifinger_model;
  std::string cube_urdf = sim_params.cube_model;
  Parser parser(&plant, &scene_graph);
  parser.AddModels(FindResourceOrThrow(trifinger_urdf));
  parser.AddModels(FindResourceOrThrow(cube_urdf));

  // Fixes the trifinger base to the world frame.
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base_link"),
                   drake::math::RigidTransform<double>::Identity());

  // Sets up contact solver.
  if (sim_params.contact_solver == "SAP") {
    plant.set_discrete_contact_solver(
        drake::multibody::DiscreteContactSolver::kSap);
  } else if (sim_params.contact_solver == "TAMSI") {
    plant.set_discrete_contact_solver(
        drake::multibody::DiscreteContactSolver::kTamsi);
  } else {
    std::cerr << "Unknown contact solver setting." << std::endl;
  }

  plant.Finalize();

  // Creates LCM subscriber to controller and also LCM state publisher.
  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>();
  systems::AddActuationRecieverAndStateSenderLcm(
      &builder, plant, lcm, lcm_channels.trifinger_input_channel,
      lcm_channels.trifinger_state_channel, sim_params.trifinger_publish_rate,
      sim_params.publish_efforts, sim_params.actuator_delay);

  // Builds diagram.
  auto diagram = builder.Build();
  diagram->set_name(("trifinger_sim"));
  DrawAndSaveDiagramGraph(*diagram);

  // Create a context for this system:
  std::unique_ptr<Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  diagram_context->EnableCaching();
  diagram->SetDefaultContext(diagram_context.get());
  Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  // Set initial conditions of the simulation
  VectorXd q_init(plant.num_positions());
  q_init << sim_params.q_init_trifinger, sim_params.q_init_cube;
  plant.SetPositions(&plant_context, q_init);
  plant.SetVelocities(&plant_context, VectorXd::Zero(plant.num_velocities()));
  diagram_context->SetTime(0);
  Simulator<double> simulator(*diagram, std::move(diagram_context));

  simulator.set_publish_every_time_step(false);
  simulator.set_publish_at_initialization(false);
  simulator.set_target_realtime_rate(sim_params.realtime_rate);
  simulator.Initialize();
  simulator.AdvanceTo(std::numeric_limits<double>::infinity());

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) {
  return dairlib::SimulateTrifinger(argc, argv);
}