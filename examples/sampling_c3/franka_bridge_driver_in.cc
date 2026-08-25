#include <dairlib/lcmt_robot_input.hpp>
#include <drake/common/yaml/yaml_io.h>
#include <drake/lcm/drake_lcm.h>
#include <drake/lcmt_panda_command.hpp>
#include <drake/lcmt_panda_status.hpp>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/lcm/lcm_interface_system.h>
#include <drake/systems/lcm/lcm_publisher_system.h>
#include <drake/systems/lcm/lcm_subscriber_system.h>
#include <gflags/gflags.h>

#include "common/parameters/franka_drake_lcm_driver_channels.h"
#include "examples/sampling_c3/parameter_headers/lcm_channels.h"
#include "examples/sampling_c3/parameter_headers/sampling_c3_controller_params.h"
#include "examples/sampling_c3/sampling_c3_utils.h"
#include "multibody/multibody_utils.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/franka_state_translator.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"

#include "drake/common/find_resource.h"

using drake::math::RigidTransform;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::Diagram;
using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;

using dairlib::systems::RobotInputReceiver;
using dairlib::systems::RobotOutputSender;
using dairlib::systems::SubvectorPassThrough;
using dairlib::systems::TimestampedVector;

DEFINE_string(lcm_url, "udpm://239.255.76.67:7667?ttl=0",
              "LCM URL with IP, port, and TTL settings");
DEFINE_string(demo_name, "jacktoy",
              "Demo within sampling_c3; used to find controller params file");

namespace dairlib {

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  std::string controller_params_path =
      "examples/sampling_c3/" + FLAGS_demo_name +
      "/parameters/sampling_c3_controller_params.yaml";
  SamplingC3ControllerParams controller_params =
      drake::yaml::LoadYamlFile<SamplingC3ControllerParams>(
          controller_params_path);
  std::string lcm_channels_file = controller_params.lcm_channels_hardware_file;
  SamplingC3LcmChannels lcm_channel_params =
      drake::yaml::LoadYamlFile<SamplingC3LcmChannels>(lcm_channels_file);
  FrankaDrakeLcmDriverChannels franka_driver_channel_params =
      drake::yaml::LoadYamlFile<FrankaDrakeLcmDriverChannels>(
          controller_params.franka_driver_channels_file);

  DiagramBuilder<double> builder;

  MultibodyPlant<double> plant(0.0);
  AddFrankaToPlant(&plant, nullptr, false, false);
  plant.Finalize();

  auto pos_map = multibody::MakeNameToPositionsMap(plant);
  auto vel_map = multibody::MakeNameToVelocitiesMap(plant);
  auto act_map = multibody::MakeNameToActuatorsMap(plant);

  auto pos_names = multibody::ExtractOrderedNamesFromMap(pos_map);
  auto vel_names = multibody::ExtractOrderedNamesFromMap(vel_map);
  auto act_names = multibody::ExtractOrderedNamesFromMap(act_map);

  /* -------------------------------------------------------------------------------------------*/
  drake::lcm::DrakeLcm lcm(FLAGS_lcm_url);

  auto franka_command_pub =
      builder.AddSystem(LcmPublisherSystem::Make<drake::lcmt_panda_command>(
          franka_driver_channel_params.franka_command_channel, &lcm,
          1.0 / 1000.0));
  auto franka_command_translator =
      builder.AddSystem<systems::FrankaEffortsInTranslator>();

  builder.Connect(*franka_command_translator, *franka_command_pub);

  auto owned_diagram = builder.Build();
  owned_diagram->set_name(("sampling_c3_franka_bridge_driver_in"));
  DrawAndSaveDiagramGraph(*owned_diagram);

  std::shared_ptr<Diagram<double>> shared_diagram = std::move(owned_diagram);
  systems::LcmDrivenLoop<dairlib::lcmt_robot_input> loop(
      &lcm, shared_diagram, franka_command_translator,
      lcm_channel_params.robot_input_channel, true);
  loop.Simulate();

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { dairlib::DoMain(argc, argv); }
