#include <math.h>
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
#include "examples/sampling_c3/sampling_c3_utils.h"
#include "examples/sampling_c3/parameter_headers/sampling_c3_controller_params.h"
#include "examples/sampling_c3/parameter_headers/lcm_channels.h"
#include "examples/sampling_c3/parameter_headers/franka_sim_params.h"
#include "multibody/multibody_utils.h"
#include "systems/robot_lcm_systems.h"

namespace dairlib {

using dairlib::systems::SubvectorPassThrough;
using drake::geometry::GeometrySet;
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
using multibody::MakeNameToPositionsMap;
using multibody::MakeNameToVelocitiesMap;
using systems::AddActuationRecieverAndStateSenderLcm;

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;


DEFINE_string(lcm_url, "udpm://239.255.76.67:7667?ttl=0",
              "LCM URL with IP, port, and TTL settings");
DEFINE_string(demo_name, "jacktoy",
              "Name for the demo, used when building filepaths for output.");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Load parameters.
  std::string controller_params_path = "examples/sampling_c3/" +
    FLAGS_demo_name + "/parameters/sampling_c3_controller_params.yaml";
  SamplingC3ControllerParams controller_params =
      drake::yaml::LoadYamlFile<SamplingC3ControllerParams>(
          controller_params_path);
  std::string lcm_channels_file =
      controller_params.lcm_channels_simulation_file;
  SamplingC3LcmChannels lcm_channel_params =
      drake::yaml::LoadYamlFile<SamplingC3LcmChannels>(lcm_channels_file);
  FrankaSimParams sim_params = drake::yaml::LoadYamlFile<FrankaSimParams>(
      controller_params.sim_params_file);

  // Build the simulation plant.
  DiagramBuilder<double> builder;
  double sim_dt = sim_params.dt;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, sim_dt);
  ModelInstanceIndex franka_index = AddFrankaToPlant(&plant, &scene_graph);
  ModelInstanceIndex object_index = AddObjectToPlant(&plant, &scene_graph,
                                                     sim_params.object_model);

  std::vector<ModelInstanceIndex> balls;
  int num_balls = 1;
  for (int i = 0; i < num_balls; i++) {
    ModelInstanceIndex object_index1 = AddObjectToPlant(&plant, &scene_graph,
                                                     "examples/sampling_c3/urdf/ball.urdf");
    balls.push_back(object_index1);
  }                                                                          
  plant.Finalize();
  /* -------------------------------------------------------------------------------------------*/

  drake::lcm::DrakeLcm drake_lcm(FLAGS_lcm_url);
  auto lcm =
      builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>(&drake_lcm);
  AddActuationRecieverAndStateSenderLcm(
      &builder, plant, lcm, lcm_channel_params.franka_input_channel,
      lcm_channel_params.franka_state_channel, sim_params.franka_publish_rate,
      franka_index, sim_params.publish_efforts, sim_params.actuator_delay);
  auto object_state_sender =
      builder.AddSystem<systems::ObjectStateSender>(plant, false, object_index);
  auto object_state_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_object_state>(
          lcm_channel_params.object_state_channel, lcm,
          1.0 / sim_params.object_publish_rate));

  builder.Connect(plant.get_state_output_port(object_index),
                  object_state_sender->get_input_port_state());
  builder.Connect(object_state_sender->get_output_port(),
                  object_state_pub->get_input_port());

  int nq = plant.num_positions();
  int nv = plant.num_velocities();

  if (sim_params.visualize_drake_sim) {
    drake::visualization::AddDefaultVisualization(&builder);
  }

  auto diagram = builder.Build();

  drake::systems::Simulator<double> simulator(*diagram);

  simulator.set_publish_every_time_step(false);
  simulator.set_publish_at_initialization(false);
  simulator.set_target_realtime_rate(sim_params.realtime_rate);

  auto& plant_context = diagram->GetMutableSubsystemContext(
      plant, &simulator.get_mutable_context());

  VectorXd q = VectorXd::Zero(nq);

  q.head(plant.num_positions(franka_index)) = sim_params.q_init_franka;
  for (int i = 0; i < num_balls; i++) {
      q.segment(7 * (i+1), 7) = sim_params.q_init_object;
  }
  q.tail(7) = sim_params.q_init_object;

  plant.SetPositions(&plant_context, q);

  VectorXd v = VectorXd::Zero(nv);
  plant.SetVelocities(&plant_context, v);

  simulator.Initialize();
  simulator.AdvanceTo(std::numeric_limits<double>::infinity());

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { dairlib::DoMain(argc, argv); }
