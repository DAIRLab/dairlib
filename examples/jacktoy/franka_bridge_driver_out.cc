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
#include "drake/common/find_resource.h"

#include "examples/jacktoy/parameters/franka_drake_lcm_driver_channels.h"
#include "examples/jacktoy/parameters/franka_lcm_channels.h"
#include "examples/jacktoy/parameters/franka_sim_params.h"
#include "examples/jacktoy/systems/franka_state_translator.h"
#include "multibody/multibody_utils.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"

using drake::math::RigidTransform;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;

using dairlib::systems::RobotInputReceiver;
using dairlib::systems::RobotOutputSender;
using dairlib::systems::SubvectorPassThrough;
using dairlib::systems::TimestampedVector;

DEFINE_string(lcm_channels,
              "examples/jacktoy/parameters/lcm_channels_hardware.yaml",
              "Filepath containing lcm channels");
DEFINE_string(franka_driver_channels, "examples/jacktoy/parameters/franka_drake_lcm_driver_channels.yaml",
              "Filepath containing drake franka driver channels");

namespace dairlib {

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  FrankaLcmChannels lcm_channel_params =
      drake::yaml::LoadYamlFile<FrankaLcmChannels>(FLAGS_lcm_channels);
  FrankaDrakeLcmDriverChannels franka_driver_channel_params =
      drake::yaml::LoadYamlFile<FrankaDrakeLcmDriverChannels>(FLAGS_franka_driver_channels);
  FrankaSimParams sim_params = drake::yaml::LoadYamlFile<FrankaSimParams>(
      "examples/jacktoy/parameters/franka_sim_params.yaml");

  DiagramBuilder<double> builder;

  MultibodyPlant<double> plant(0.0);

  Parser parser(&plant);
      parser.AddModels(drake::FindResourceOrThrow(sim_params.franka_model));

//   parser.AddModelsFromUrl(sim_params.franka_model);
  Eigen::Vector3d franka_origin = Eigen::VectorXd::Zero(3);
  RigidTransform<double> R_X_W = RigidTransform<double>(
      drake::math::RotationMatrix<double>(), franka_origin);
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_link0"),
                   R_X_W);
  plant.Finalize();

  auto pos_map = multibody::MakeNameToPositionsMap(plant);
  auto vel_map = multibody::MakeNameToVelocitiesMap(plant);
  auto act_map = multibody::MakeNameToActuatorsMap(plant);

  auto pos_names = multibody::ExtractOrderedNamesFromMap(pos_map);
  auto vel_names = multibody::ExtractOrderedNamesFromMap(vel_map);
  auto act_names = multibody::ExtractOrderedNamesFromMap(act_map);

  /* -------------------------------------------------------------------------------------------*/
  drake::lcm::DrakeLcm lcm("udpm://239.255.76.67:7667?ttl=0");


  auto franka_state_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_output>(
          lcm_channel_params.franka_state_channel, &lcm,
          1.0 / 1000.0));
  auto franka_state_translator = builder.AddSystem<systems::FrankaStateOutTranslator>(
      pos_names, vel_names, act_names);

  builder.Connect(*franka_state_translator, *franka_state_pub);

  auto owned_diagram = builder.Build();
  owned_diagram->set_name(("franka_bridge_driver_out"));

  systems::LcmDrivenLoop<drake::lcmt_panda_status> loop(
      &lcm, std::move(owned_diagram), franka_state_translator,
      franka_driver_channel_params.franka_status_channel, true);
  DrawAndSaveDiagramGraph(*loop.get_diagram());
  loop.Simulate();

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { dairlib::DoMain(argc, argv); }