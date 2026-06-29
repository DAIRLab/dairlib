#include <math.h>
#include <vector>
#include <iostream>
#include <limits>

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

#include <drake/systems/framework/diagram_builder.h>
#include "common/eigen_utils.h"
#include "common/find_resource.h"
#include "examples/sampling_c3/sampling_c3_utils.h"
#include "examples/sampling_c3/parameter_headers/sampling_c3_controller_params.h"
#include "examples/sampling_c3/parameter_headers/sampling_c3_options.h"
#include "examples/sampling_c3/parameter_headers/lcm_channels.h"
#include "examples/sampling_c3/parameter_headers/franka_sim_params.h"
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
using systems::AddActuationRecieverAndStateSenderLcm;
using systems::Add3dPrinterStateReceiverAndStateSenderLcm;

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

DEFINE_string(lcm_url, "udpm://239.255.76.67:7667?ttl=0",
              "LCM URL with IP, port, and TTL settings");
DEFINE_string(demo_name, "jacktoy",
              "Name for the demo, used when building filepaths for output.");

int DoMain(int argc, char* argv[]) {
  std::cout << "\n========== STARTING SIM ==========\n" << std::endl;

  gflags::ParseCommandLineFlags(&argc, &argv, true);

  std::cout << "[DEBUG] demo_name = " << FLAGS_demo_name << std::endl;
  std::cout << "[DEBUG] lcm_url   = " << FLAGS_lcm_url << std::endl;

  // --------------------------------------------------------------------------
  // Load parameters
  // --------------------------------------------------------------------------

  std::string controller_params_path =
      "examples/sampling_c3/" + FLAGS_demo_name +
      "/parameters/sampling_c3_controller_params.yaml";

  std::cout << "\n[DEBUG] Loading controller params from:\n  "
            << controller_params_path << std::endl;

  SamplingC3ControllerParams controller_params =
      drake::yaml::LoadYamlFile<SamplingC3ControllerParams>(
          controller_params_path);

  std::cout << "[DEBUG] Controller params loaded." << std::endl;

  std::string lcm_channels_file =
      controller_params.lcm_channels_simulation_file;

  std::cout << "[DEBUG] Loading LCM channel file:\n  "
            << lcm_channels_file << std::endl;

  SamplingC3LcmChannels lcm_channel_params =
      drake::yaml::LoadYamlFile<SamplingC3LcmChannels>(
          lcm_channels_file);

  std::cout << "[DEBUG] LCM channel params loaded." << std::endl;

  std::cout << "[DEBUG] Loading sim params:\n  "
            << controller_params.sim_params_file << std::endl;

  FrankaSimParams sim_params =
      drake::yaml::LoadYamlFile<FrankaSimParams>(
          controller_params.sim_params_file);

  std::cout << "[DEBUG] Sim params loaded." << std::endl;

  std::cout << "[DEBUG] Loading sampling C3 options:\n  "
            << controller_params.sampling_c3_options_file << std::endl;

  SamplingC3Options sampling_c3_options =
      drake::yaml::LoadYamlFile<SamplingC3Options>(
          controller_params.sampling_c3_options_file);

  std::cout << "[DEBUG] Sampling C3 options loaded." << std::endl;

  // --------------------------------------------------------------------------
  // Build plant
  // --------------------------------------------------------------------------

  std::cout << "\n========== BUILDING PLANT ==========" << std::endl;

  DiagramBuilder<double> builder;

  double sim_dt = sim_params.dt;
  std::cout << "[DEBUG] sim_dt = " << sim_dt << std::endl;

  auto [plant, scene_graph] =
      AddMultibodyPlantSceneGraph(&builder, sim_dt);

  std::cout << "[DEBUG] Plant + SceneGraph created." << std::endl;

  ModelInstanceIndex franka_index =
      Add3DPrinterToPlant(
          &plant,
          &scene_graph,
          true,
          true,
          sampling_c3_options.include_walls);

  std::cout << "[DEBUG] Added 3D printer." << std::endl;
  std::cout << "[DEBUG] franka_index = "
            << franka_index << std::endl;

  int num_objects = sim_params.object_models.size();

  std::cout << "[DEBUG] Number of objects = "
            << num_objects << std::endl;

  for (int i = 0; i < num_objects; i++) {
    std::cout << "[DEBUG] object_model[" << i << "] = "
              << sim_params.object_models[i]
              << std::endl;
  }

  std::vector<ModelInstanceIndex> object_indices =
      AddObjectsToPlant(
          &plant,
          &scene_graph,
          sim_params.object_models);

  std::cout << "[DEBUG] Objects added to plant." << std::endl;

  for (int i = 0; i < object_indices.size(); i++) {
    std::cout << "[DEBUG] object_indices[" << i
              << "] = " << object_indices[i]
              << std::endl;
  }

  std::cout << "[DEBUG] Finalizing plant..." << std::endl;

  plant.Finalize();

  std::cout << "[DEBUG] Plant finalized." << std::endl;

  std::cout << "[DEBUG] num_positions = "
            << plant.num_positions() << std::endl;

  std::cout << "[DEBUG] num_velocities = "
            << plant.num_velocities() << std::endl;

  // --------------------------------------------------------------------------
  // LCM
  // --------------------------------------------------------------------------
std::cout << "\n========== SETTING UP LCM =========="
          << std::endl;

drake::lcm::DrakeLcm drake_lcm(FLAGS_lcm_url);

auto lcm =
    builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>(
        &drake_lcm);

std::cout << "[DEBUG] LCM interface system created."
          << std::endl;

dairlib::systems::Add3dPrinterStateReceiverAndStateSenderLcm(
    &builder,
    plant,
    lcm,
    lcm_channel_params.three_d_printer_input_channel,
    lcm_channel_params.three_d_printer_state_channel,
    sim_params.franka_publish_rate,
    franka_index,
    sim_params.publish_efforts);

std::cout << "[DEBUG] 3D printer LCM systems connected."
          << std::endl;

  // --------------------------------------------------------------------------
  // Object publishers
  // --------------------------------------------------------------------------

  std::vector<systems::ObjectStateSender*> object_state_senders;
  std::vector<LcmPublisherSystem*> object_state_pubs;

  for (int i = 0; i < num_objects; i++) {
    std::cout << "\n[DEBUG] Creating publisher for object "
              << i << std::endl;

    std::cout << "[DEBUG] object index = "
              << object_indices.at(i) << std::endl;

    std::cout << "[DEBUG] channel = "
              << lcm_channel_params.object_state_channels.at(i)
              << std::endl;

    object_state_senders.push_back(
        builder.AddSystem<systems::ObjectStateSender>(
            plant,
            false,
            object_indices.at(i)));

    object_state_pubs.push_back(
        builder.AddSystem(
            LcmPublisherSystem::Make<dairlib::lcmt_object_state>(
                lcm_channel_params.object_state_channels.at(i),
                lcm,
                1.0 / sim_params.object_publish_rate)));
  }

  std::cout << "\n[DEBUG] Connecting object publishers..."
            << std::endl;

  for (int i = 0; i < num_objects; i++) {
    builder.Connect(
        plant.get_state_output_port(object_indices[i]),
        object_state_senders.at(i)->get_input_port_state());

    builder.Connect(
        object_state_senders.at(i)->get_output_port(),
        object_state_pubs.at(i)->get_input_port());

    std::cout << "[DEBUG] Connected object "
              << i << std::endl;
  }

  int nq = plant.num_positions();
  int nv = plant.num_velocities();

  std::cout << "\n[DEBUG] nq = " << nq << std::endl;
  std::cout << "[DEBUG] nv = " << nv << std::endl;

  if (sim_params.visualize_drake_sim) {
    std::cout << "[DEBUG] Visualization ENABLED"
              << std::endl;

    drake::visualization::AddDefaultVisualization(
        &builder);
  } else {
    std::cout << "[DEBUG] Visualization DISABLED"
              << std::endl;
  }

  // --------------------------------------------------------------------------
  // Build diagram
  // --------------------------------------------------------------------------

  std::cout << "\n========== BUILDING DIAGRAM =========="
            << std::endl;

  auto diagram = builder.Build();

  std::cout << "[DEBUG] Diagram built."
            << std::endl;

  diagram->set_name(("three_d_printer_sim"));
  DrawAndSaveDiagramGraph(*diagram);

  drake::systems::Simulator<double> simulator(*diagram);

  simulator.set_publish_every_time_step(false);
  simulator.set_publish_at_initialization(false);
  simulator.set_target_realtime_rate(
      sim_params.realtime_rate);

  std::cout << "[DEBUG] Simulator configured."
            << std::endl;

  auto& plant_context =
      diagram->GetMutableSubsystemContext(
          plant,
          &simulator.get_mutable_context());

  std::cout << "[DEBUG] Plant context acquired."
            << std::endl;

  // --------------------------------------------------------------------------
  // Initialize state
  // --------------------------------------------------------------------------

  std::cout << "\n========== INITIALIZING STATE =========="
            << std::endl;

  VectorXd q = VectorXd::Zero(nq);

  std::cout << "[DEBUG] q size = "
            << q.size() << std::endl;

  std::cout << "[DEBUG] franka q_init size = "
            << sim_params.q_init_franka.size()
            << std::endl;

  q.head(plant.num_positions(franka_index)) =
      sim_params.q_init_franka;

  for (int i = 0; i < num_objects; i++) {
    std::cout << "[DEBUG] Setting object "
              << i
              << " position = "
              << sim_params.q_init_objects.at(i).transpose()
              << std::endl;

    q.segment(3 * (i + 1), 3) =
        sim_params.q_init_objects.at(i);
  }

  std::cout << "[DEBUG] Writing final tail object position."
            << std::endl;

  q.tail(3) =
      sim_params.q_init_objects.at(num_objects - 1);

  std::cout << "[DEBUG] Final q:\n"
            << q.transpose()
            << std::endl;

  plant.SetPositions(&plant_context, q);

  std::cout << "[DEBUG] Positions set."
            << std::endl;

  VectorXd v = VectorXd::Zero(nv);

  std::cout << "[DEBUG] Initial velocity vector:\n"
            << v.transpose()
            << std::endl;

  plant.SetVelocities(&plant_context, v);

  std::cout << "[DEBUG] Velocities set."
            << std::endl;

  // --------------------------------------------------------------------------
  // Run
  // --------------------------------------------------------------------------

  std::cout << "\n========== INITIALIZING SIMULATOR =========="
            << std::endl;

  simulator.Initialize();

  std::cout << "[DEBUG] Simulator initialized."
            << std::endl;

  std::cout << "\n========== STARTING ADVANCE =========="
            << std::endl;

  simulator.AdvanceTo(
      std::numeric_limits<double>::infinity());

  std::cout << "[DEBUG] Simulator exited AdvanceTo()."
            << std::endl;

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) {
  return dairlib::DoMain(argc, argv);
}