#include <dairlib/lcmt_object_state.hpp>
#include <gflags/gflags.h>

#include "examples/sampling_c3/merged_object_state_sender.h"
#include "examples/sampling_c3/parameter_headers/lcm_channels.h"
#include "examples/sampling_c3/parameter_headers/sampling_c3_controller_params.h"
#include "examples/sampling_c3/sampling_c3_utils.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"

#include "drake/common/yaml/yaml_io.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

namespace dairlib {

using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::Diagram;
using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;

using systems::MergedObjectStateSender;
using systems::ObjectStateReceiver;

DEFINE_bool(is_simulation, true, "True for simulation, false for hardware");
DEFINE_string(demo_name, "anything",
              "Name for the demo, used when building filepaths for output.");
DEFINE_string(lcm_url, "udpm://239.255.76.67:7667?ttl=0",
              "LCM URL with IP, port, and TTL settings");
DEFINE_double(publish_rate, 200.0, "Publish rate for merged object states");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Load parameters
  std::string controller_params_path =
      "examples/sampling_c3/" + FLAGS_demo_name +
      "/parameters/sampling_c3_controller_params.yaml";
  SamplingC3ControllerParams controller_params =
      drake::yaml::LoadYamlFile<SamplingC3ControllerParams>(
          controller_params_path);

  std::string lcm_channels_file =
      FLAGS_is_simulation ? controller_params.lcm_channels_simulation_file
                          : controller_params.lcm_channels_hardware_file;
  SamplingC3LcmChannels lcm_channel_params =
      drake::yaml::LoadYamlFile<SamplingC3LcmChannels>(lcm_channels_file);

  DiagramBuilder<double> builder;

  // Create the LCM interface
  drake::lcm::DrakeLcm lcm(FLAGS_lcm_url);

  // Build a plant just for ObjectStateReceiver (to get object dimensions)
  MultibodyPlant<double> object_plant(0.0);
  std::vector<ModelInstanceIndex> object_indices = AddObjectsToPlant(
      &object_plant, nullptr, controller_params.object_models);
  object_plant.Finalize();

  // Get merging configuration from controller params
  std::vector<std::string> merged_object_names =
      controller_params.merged_object_names;
  std::vector<std::vector<std::string>> object_groups_for_merging =
      controller_params.object_groups_for_merging;

  // Extract object names from all groups
  std::vector<std::string> object_names;
  for (const auto& group : object_groups_for_merging) {
    for (const auto& name : group) {
      object_names.push_back(name);
    }
  }
  int num_objects = static_cast<int>(object_names.size());

  if (num_objects == 0) {
    std::cerr << "No objects configured. Exiting." << std::endl;
    return 1;
  }

  // Create LCM subscribers for each object state
  std::vector<LcmSubscriberSystem*> object_state_subs;
  for (int i = 0; i < num_objects; i++) {
    object_state_subs.push_back(
        builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_object_state>(
            lcm_channel_params.object_state_channels.at(i), &lcm)));
  }

  // Create ObjectStateReceivers for each object (for potential internal use)
  std::vector<ObjectStateReceiver*> object_state_receivers;
  for (int i = 0;
       i < num_objects && i < static_cast<int>(object_indices.size()); i++) {
    object_state_receivers.push_back(builder.AddSystem<ObjectStateReceiver>(
        object_plant, object_indices[i]));
  }

  // Connect subscribers to ObjectStateReceivers
  for (size_t i = 0; i < object_state_receivers.size(); i++) {
    builder.Connect(object_state_subs.at(i)->get_output_port(),
                    object_state_receivers.at(i)->get_input_port(0));
  }

  // Create MergedObjectStateSender
  auto merged_object_state_sender = builder.AddSystem<MergedObjectStateSender>(
      object_names, merged_object_names, object_groups_for_merging);

  // Connect LCM subscribers directly to MergedObjectStateSender
  // (subscribers output lcmt_object_state which is what MergedObjectStateSender
  // expects)
  for (int i = 0; i < num_objects; i++) {
    builder.Connect(object_state_subs.at(i)->get_output_port(),
                    merged_object_state_sender->get_input_port_object_state(
                        object_names[i]));
  }

  // Create LcmPublisherSystem for each merged object state
  int num_merged_objects = static_cast<int>(merged_object_names.size());
  std::vector<LcmPublisherSystem*> merged_state_pubs;
  for (int i = 0; i < num_merged_objects; i++) {
    std::string channel_name = "MERGED_OBJECT_STATE_" + merged_object_names[i];
    merged_state_pubs.push_back(
        builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_object_state>(
            channel_name, &lcm, 1.0 / FLAGS_publish_rate)));
  }

  // Connect MergedObjectStateSender individual outputs to publishers
  for (int i = 0; i < num_merged_objects; i++) {
    builder.Connect(merged_object_state_sender->get_output_port_merged_state(i),
                    merged_state_pubs.at(i)->get_input_port());
  }

  // Build the diagram and launch lcm-driven loop
  auto owned_diagram = builder.Build();
  owned_diagram->set_name(("sampling_c3_controller_" + FLAGS_demo_name));
  DrawAndSaveDiagramGraph(*owned_diagram);

  // Run lcm-driven simulation.  The buffer size argument is needed to ensure
  // the latest messages are used in the control loop.  See
  // https://github.com/DAIRLab/dairlib/pull/366 for more details.
  int lcm_buffer_size = 200;
  std::shared_ptr<Diagram<double>> shared_diagram = std::move(owned_diagram);
  systems::LcmDrivenLoop<dairlib::lcmt_object_state> loop(
      &lcm, shared_diagram, object_state_receivers[0],
      lcm_channel_params.object_state_channels[0], true, lcm_buffer_size);

  LcmHandleSubscriptionsUntil(&lcm, [&]() {
    for (const auto& sub : object_state_subs) {
      if (sub->GetInternalMessageCount() <= 1) {
        return false;
      }
    }
    return true;
  });

  // Print debugging messages
  std::cout << "=== Merged Object State Sender ===" << std::endl;
  std::cout << "Listening on " << num_objects
            << " object state channel(s):" << std::endl;
  for (int i = 0; i < num_objects; i++) {
    std::cout << "  [" << i << "] "
              << lcm_channel_params.object_state_channels.at(i)
              << " -> object name: " << object_names[i] << std::endl;
  }
  std::cout << "\nMerging configuration:" << std::endl;
  for (size_t g = 0; g < object_groups_for_merging.size(); g++) {
    std::cout << "  Group '" << merged_object_names[g] << "': {";
    for (size_t j = 0; j < object_groups_for_merging[g].size(); j++) {
      if (j > 0) std::cout << ", ";
      std::cout << object_groups_for_merging[g][j];
    }
    std::cout << "}" << std::endl;
  }
  std::cout << "\nPublishing merged states to channel(s):" << std::endl;
  for (int i = 0; i < num_merged_objects; i++) {
    std::cout << "  [" << i << "] MERGED_OBJECT_STATE_"
              << merged_object_names[i] << std::endl;
  }
  std::cout << "\nPublish rate: " << FLAGS_publish_rate << " Hz" << std::endl;
  std::cout << "Running..." << std::endl;

  loop.Simulate();

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::DoMain(argc, argv); }
