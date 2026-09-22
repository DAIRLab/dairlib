#include <math.h>

#include <iostream>
#include <limits>
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
#include "examples/sampling_c3/object_state_error_injector.h"
#include "examples/sampling_c3/parameter_headers/lcm_channels.h"
#include "examples/sampling_c3/parameter_headers/robot_sim_params.h"
#include "examples/sampling_c3/parameter_headers/sampling_c3_controller_params.h"
#include "examples/sampling_c3/parameter_headers/sampling_c3_options.h"
#include "examples/sampling_c3/sampling_c3_utils.h"
#include "multibody/multibody_utils.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"

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
using drake::systems::Diagram;
using drake::systems::DiagramBuilder;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::trajectories::PiecewisePolynomial;
using multibody::MakeNameToPositionsMap;
using multibody::MakeNameToVelocitiesMap;
using systems::Add3dPrinterStateReceiverAndStateSenderLcm;
using systems::AddActuationRecieverAndStateSenderLcm;

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

DEFINE_string(lcm_url, "udpm://239.255.76.67:7667?ttl=0",
              "LCM URL with IP, port, and TTL settings");
DEFINE_string(demo_name, "cone",
              "Name for the sampling_c3/three_d_printer demo, used when "
              "building filepaths for output.");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  if (FLAGS_demo_name != "cone") {
    throw std::runtime_error("Unknown --demo_name value: " + FLAGS_demo_name);
  }

  // --------------------------------------------------------------------------
  // Load parameters
  // --------------------------------------------------------------------------

  std::string controller_params_path =
      "examples/sampling_c3/three_d_printer/" + FLAGS_demo_name +
      "/parameters/sampling_c3_controller_params.yaml";

  SamplingC3ControllerParams controller_params =
      drake::yaml::LoadYamlFile<SamplingC3ControllerParams>(
          controller_params_path);

  std::string lcm_channels_file =
      controller_params.lcm_channels_simulation_file;

  SamplingC3LcmChannels lcm_channel_params =
      drake::yaml::LoadYamlFile<SamplingC3LcmChannels>(lcm_channels_file);

  RobotSimParams sim_params = drake::yaml::LoadYamlFile<RobotSimParams>(
      controller_params.sim_params_file);

  SamplingC3Options sampling_c3_options =
      drake::yaml::LoadYamlFile<SamplingC3Options>(
          controller_params.sampling_c3_options_file);

  // --------------------------------------------------------------------------
  // Build plant
  // --------------------------------------------------------------------------

  DiagramBuilder<double> builder;

  double sim_dt = sim_params.dt;

  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, sim_dt);

  ModelInstanceIndex robot_index =
      Add3DPrinterToPlant(&plant, &scene_graph, true);

  int num_objects = sim_params.object_models.size();

  std::vector<ModelInstanceIndex> object_indices =
      AddObjectsToPlant(&plant, &scene_graph, sim_params.object_models);

  plant.Finalize();

  // --------------------------------------------------------------------------
  // LCM
  // --------------------------------------------------------------------------

  drake::lcm::DrakeLcm drake_lcm(FLAGS_lcm_url);

  auto lcm =
      builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>(&drake_lcm);

  const dairlib::systems::TransportLag* command_lag =
      dairlib::systems::Add3dPrinterStateReceiverAndStateSenderLcm(
          &builder, plant, lcm, lcm_channel_params.robot_input_channel,
          lcm_channel_params.robot_state_channel,
          sim_params.robot_publish_rate, robot_index,
          sim_params.publish_efforts, sim_params.q_init_robot,
          dairlib::systems::k3dPrinterMaxHorizontalVelocity,
          dairlib::systems::k3dPrinterMaxVerticalVelocity,
          sim_params.actuator_delay,
          sim_params.command_time_constant.value_or(0.0), sim_dt);

  // --------------------------------------------------------------------------
  // Object publishers
  // --------------------------------------------------------------------------

  // When injecting object state estimation errors, the clean object state goes
  // out on clean_object_state_channels and a corrupted copy takes its place on
  // object_state_channels, so the controller needs no change to see
  // hardware-like pose estimates.
  if (sim_params.inject_object_state_errors) {
    if (!lcm_channel_params.clean_object_state_channels.has_value()) {
      throw std::runtime_error(
          "inject_object_state_errors is true but the lcm channels file " +
          lcm_channels_file + " does not set clean_object_state_channels.");
    }
    if (static_cast<int>(
            lcm_channel_params.clean_object_state_channels->size()) !=
        num_objects) {
      throw std::runtime_error(
          "clean_object_state_channels has " +
          std::to_string(lcm_channel_params.clean_object_state_channels->size()) +
          " entries but there are " + std::to_string(num_objects) +
          " objects.");
    }
  }

  std::vector<systems::ObjectStateSender*> object_state_senders;
  std::vector<LcmPublisherSystem*> object_state_pubs;

  for (int i = 0; i < num_objects; i++) {
    object_state_senders.push_back(
        builder.AddSystem<systems::ObjectStateSender>(plant, false,
                                                      object_indices.at(i)));

    object_state_pubs.push_back(
        builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_object_state>(
            lcm_channel_params.object_state_channels.at(i), lcm,
            1.0 / sim_params.object_publish_rate)));
  }

  for (int i = 0; i < num_objects; i++) {
    if (sim_params.inject_object_state_errors) {
      // The clean state goes out on its own sender and publisher, untouched.
      auto clean_object_state_sender =
          builder.AddSystem<systems::ObjectStateSender>(plant, false,
                                                        object_indices.at(i));
      auto clean_object_state_pub =
          builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_object_state>(
              lcm_channel_params.clean_object_state_channels->at(i), lcm,
              1.0 / sim_params.object_publish_rate));

      builder.Connect(plant.get_state_output_port(object_indices[i]),
                      clean_object_state_sender->get_input_port_state());
      builder.Connect(clean_object_state_sender->get_output_port(),
                      clean_object_state_pub->get_input_port());

      // The error injector sits in front of the sender that feeds the channel
      // the controller listens to.  Redraw the error once per publish.  Offset
      // the seed per object so that objects do not share an error stream while
      // a run still reproduces exactly for a given seed.
      ObjectStateErrorParams error_params =
          *sim_params.object_state_error_params;
      if (error_params.seed.has_value()) {
        error_params.seed = *error_params.seed + i;
      }
      auto error_injector =
          builder.AddSystem<systems::ObjectStateErrorInjector>(
              plant.num_positions(object_indices.at(i)),
              plant.num_velocities(object_indices.at(i)), error_params,
              1.0 / sim_params.object_publish_rate);

      builder.Connect(plant.get_state_output_port(object_indices[i]),
                      error_injector->get_input_port_state());
      builder.Connect(error_injector->get_output_port_noisy_state(),
                      object_state_senders.at(i)->get_input_port_state());
    } else {
      builder.Connect(plant.get_state_output_port(object_indices[i]),
                      object_state_senders.at(i)->get_input_port_state());
    }

    builder.Connect(object_state_senders.at(i)->get_output_port(),
                    object_state_pubs.at(i)->get_input_port());
  }

  int nq = plant.num_positions();
  int nv = plant.num_velocities();

  if (sim_params.visualize_drake_sim) {
    drake::visualization::AddDefaultVisualization(&builder);
  }

  // --------------------------------------------------------------------------
  // Build diagram
  // --------------------------------------------------------------------------

  auto diagram = builder.Build();

  diagram->set_name(("three_d_printer_sim"));
  DrawAndSaveDiagramGraph(*diagram);

  drake::systems::Simulator<double> simulator(*diagram);

  simulator.set_publish_every_time_step(false);
  simulator.set_publish_at_initialization(false);
  simulator.set_target_realtime_rate(sim_params.realtime_rate);

  auto& plant_context = diagram->GetMutableSubsystemContext(
      plant, &simulator.get_mutable_context());

  // --------------------------------------------------------------------------
  // Initialize state
  // --------------------------------------------------------------------------

  VectorXd q = VectorXd::Zero(nq);

  q.head(plant.num_positions(robot_index)) = sim_params.q_init_robot;

  for (int i = 0; i < num_objects; i++) {
    q.segment(3 + 7 * (i), 7) = sim_params.q_init_objects.at(i);
  }

  q.tail(7) = sim_params.q_init_objects.at(num_objects - 1);

  plant.SetPositions(&plant_context, q);

  VectorXd v = VectorXd::Zero(nv);

  plant.SetVelocities(&plant_context, v);

  // Hold the printer's starting pose in the command lag as well, so the
  // end effector does not get dragged toward the origin while the delay
  // buffer fills.
  if (command_lag != nullptr) {
    VectorXd x_init = VectorXd::Zero(command_lag->size());
    x_init.head(sim_params.q_init_robot.size()) = sim_params.q_init_robot;
    command_lag->SetInitialValue(
        &diagram->GetMutableSubsystemContext(*command_lag,
                                             &simulator.get_mutable_context()),
        x_init);
  }

  // --------------------------------------------------------------------------
  // Run
  // --------------------------------------------------------------------------

  simulator.Initialize();

  simulator.AdvanceTo(std::numeric_limits<double>::infinity());

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::DoMain(argc, argv); }