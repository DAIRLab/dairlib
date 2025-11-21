#include <math.h>

#include <iostream>
#include <vector>

#include <drake/common/find_resource.h>
#include <drake/common/yaml/yaml_io.h>
#include <drake/geometry/drake_visualizer.h>
#include <drake/geometry/meshcat_visualizer.h>
#include <drake/lcm/drake_lcm.h>
#include <drake/math/rigid_transform.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/lcm/lcm_interface_system.h>
#include <drake/systems/lcm/lcm_publisher_system.h>
#include <drake/systems/lcm/lcm_subscriber_system.h>
#include <drake/systems/primitives/multiplexer.h>
#include <drake/visualization/visualization_config_functions.h>
#include <gflags/gflags.h>

#include "common/eigen_utils.h"
#include "common/find_resource.h"
#include "examples/deform/deform_utils.h"
#include "examples/deform/parameter_headers/elastoplastic_sim_params.h"
#include "examples/deform/parameter_headers/lcm_channels.h"
#include "examples/deform/parameter_headers/spring_damper_model_params.h"
#include "multibody/multibody_utils.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"

namespace dairlib {

using dairlib::systems::SubvectorPassThrough;
using drake::geometry::SceneGraph;
using drake::math::RigidTransform;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::Context;
using drake::systems::DiagramBuilder;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::trajectories::PiecewisePolynomial;
using systems::AddActuationRecieverAndStateSenderLcm;

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

DEFINE_string(lcm_url, "udpm://239.255.76.67:7667?ttl=0",
              "LCM URL with IP, port, and TTL settings");
DEFINE_string(demo, "1d_rigid", "Demo type");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Load parameters.
  ElastoPlasticSimParams sim_params =
      drake::yaml::LoadYamlFile<ElastoPlasticSimParams>(
          "examples/deform/parameters/demo_" + FLAGS_demo +
          "/elastoplastic_sim_params.yaml");
  DeformLcmChannels lcm_channel_params =
      drake::yaml::LoadYamlFile<DeformLcmChannels>(
          "examples/deform/parameters/lcm_channels_sim.yaml");

  // Build the simulation plant.
  DiagramBuilder<double> builder;
  double sim_dt = sim_params.dt;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, sim_dt);
  std::vector<ModelInstanceIndex> model_indices =
      AddLCSModelsToPlant(&plant, &scene_graph, FLAGS_demo);
  ModelInstanceIndex robot_index = model_indices[0];
  ModelInstanceIndex object_index = model_indices[1];
  plant.Finalize();
  std::vector<std::string> actuator_names = plant.GetActuatorNames();
  std::cout << "Plant actuation input names: " << std::endl;
  for (const auto& name : actuator_names) {
    std::cout << "  " << name << std::endl;
  }

  /* -------------------------------------------------------------------------*/

  drake::lcm::DrakeLcm drake_lcm(FLAGS_lcm_url);
  auto lcm =
      builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>(&drake_lcm);
  AddActuationRecieverAndStateSenderLcm(
      &builder, plant, lcm,
      //"EE_FAKE_INPUT",
      lcm_channel_params.robot_input_channel,
      lcm_channel_params.robot_state_channel, sim_params.robot_publish_rate,
      robot_index, sim_params.publish_efforts, sim_params.actuator_delay);

  systems::ObjectStateSender* object_state_sender =
      builder.AddSystem<systems::ObjectStateSender>(plant, false, object_index);
  LcmPublisherSystem* object_state_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_object_state>(
          lcm_channel_params.object_state_channel, lcm,
          1.0 / sim_params.object_publish_rate));
  builder.Connect(plant.get_state_output_port(object_index),
                  object_state_sender->get_input_port_state());
  builder.Connect(object_state_sender->get_output_port(),
                  object_state_pub->get_input_port());
  int nq = plant.num_positions();
  int nv = plant.num_velocities();

  std::cout << "nq: " << nq << ", nv: " << nv << std::endl;

  if (sim_params.visualize_drake_sim) {
    drake::visualization::AddDefaultVisualization(&builder);
  }

  drake::geometry::DrakeVisualizerParams params;
  drake::geometry::DrakeVisualizerd::AddToBuilder(&builder, scene_graph,
                                                  nullptr, params);

  auto diagram = builder.Build();
  diagram->set_name(("elastoplastic_sim_" + FLAGS_demo));
  DrawAndSaveDiagramGraph(*diagram);

  drake::systems::Simulator<double> simulator(*diagram);

  simulator.set_publish_every_time_step(false);
  simulator.set_publish_at_initialization(false);
  simulator.set_target_realtime_rate(sim_params.realtime_rate);

  auto& plant_context = diagram->GetMutableSubsystemContext(
      plant, &simulator.get_mutable_context());

  plant.SetPositions(&plant_context, robot_index, sim_params.q_init_robot);
  plant.SetPositions(&plant_context, object_index, sim_params.q_init_object);
  VectorXd v = VectorXd::Zero(nv);
  plant.SetVelocities(&plant_context, v);

  std::vector<std::string> position_names = plant.GetPositionNames();
  Eigen::VectorXd q = plant.GetPositions(plant_context);

  simulator.Initialize();
  simulator.AdvanceTo(std::numeric_limits<double>::infinity());

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { dairlib::DoMain(argc, argv); }
