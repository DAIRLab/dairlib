#include <drake/systems/analysis/simulator.h>

#include "dairlib/lcmt_round_belt_state.hpp"
#include "examples/magna/parameter_headers/round_belt_controller_params.h"
#include "examples/magna/systems/state_estimation/round_belt_state_receiver.h"
#include "gflags/gflags.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"

#include "drake/common/yaml/yaml_io.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

namespace dairlib {
namespace examples {
namespace magna {

using dairlib::examples::magna::systems::state_estimation::
    RoundBeltStateReceiver;
using drake::math::RigidTransform;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::Diagram;
using drake::systems::DiagramBuilder;
using drake::systems::TriggerType;
using drake::systems::TriggerTypeSet;

DEFINE_string(lcm_url, "udpm://239.255.76.67:7667?ttl=0",
              "LCM URL with IP, port, and TTL settings");
DEFINE_bool(is_simulation, true, "True for simulation, false for hardware");
int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::systems::DiagramBuilder<double> builder;
  drake::lcm::DrakeLcm drake_lcm(FLAGS_lcm_url);

  // --------------------- Load parameters ---------------------- //
  RoundBeltControllerParams round_belt_controller_params =
      drake::yaml::LoadYamlFile<RoundBeltControllerParams>(
          FLAGS_is_simulation ? "examples/magna/parameters/"
                                "round_belt_controller_params_sim.yaml"
                              : "examples/magna/parameters/"
                                "round_belt_controller_params_hw.yaml");
  // ------------------------------------------------------------- //

  auto lcm =
      builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>(&drake_lcm);
  auto franka_state_sub =
      builder.AddSystem(drake::systems::lcm::LcmSubscriberSystem::Make<
                        dairlib::lcmt_robot_output>(
          round_belt_controller_params.lcm_channels.franka_state_channel, lcm));
  auto round_belt_state_wrt_taskboard_sub = builder.AddSystem(
      drake::systems::lcm::LcmSubscriberSystem::Make<
          dairlib::lcmt_round_belt_state>("ROUND_BELT_STATE", lcm));

  auto round_belt_state_wrt_robot_pub =
      builder.AddSystem(drake::systems::lcm::LcmPublisherSystem::Make<
                        dairlib::lcmt_round_belt_state>(
          "ROUND_BELT_STATE_WRT_ROBOT", lcm,
          TriggerTypeSet({TriggerType::kForced})));

  auto task_relevant_keypoints_pub =
      builder.AddSystem(drake::systems::lcm::LcmPublisherSystem::Make<
                        dairlib::lcmt_round_belt_state>(
          "TASK_RELEVANT_KEYPOINTS", lcm,
          TriggerTypeSet({TriggerType::kForced})));

  // Create Franka plant and context
  MultibodyPlant<double> plant_franka(0.0);
  Parser parser_franka(&plant_franka);
  [[maybe_unused]] ModelInstanceIndex franka_index =
      parser_franka.AddModelsFromUrl(
          round_belt_controller_params.franka_arm_hand_model)[0];
  plant_franka.WeldFrames(plant_franka.world_frame(),
                          plant_franka.GetFrameByName("panda_link0"),
                          RigidTransform<double>::Identity());
  plant_franka.Finalize();
  auto franka_context = plant_franka.CreateDefaultContext();
  auto round_belt_state_receiver = builder.AddSystem<RoundBeltStateReceiver>(
      plant_franka, franka_context.get(), "finger_tip",
      round_belt_controller_params.task_board_position,
      round_belt_controller_params.task_board_orientation);
  auto robot_state_receiver =
      builder.AddSystem<dairlib::systems::RobotOutputReceiver>(plant_franka);

  // Wire up connections
  builder.Connect(franka_state_sub->get_output_port(),
                  robot_state_receiver->get_input_port(0));
  builder.Connect(robot_state_receiver->get_output_port(0),
                  round_belt_state_receiver->get_input_port_franka_state());
  builder.Connect(round_belt_state_wrt_taskboard_sub->get_output_port(),
                  round_belt_state_receiver->get_input_port_round_belt_state());
  builder.Connect(round_belt_state_receiver->get_output_port_keypoint_state(),
                  round_belt_state_wrt_robot_pub->get_input_port());
  builder.Connect(
      round_belt_state_receiver->get_output_port_interested_keypoints(),
      task_relevant_keypoints_pub->get_input_port());
  // Build diagram
  auto diagram = builder.Build();
  std::shared_ptr<Diagram<double>> shared_diagram = std::move(diagram);
  shared_diagram->set_name("round_belt_state_estimator");
  DrawAndSaveDiagramGraph(*shared_diagram);

  // Run lcm-driven simulation
  dairlib::systems::LcmDrivenLoop<dairlib::lcmt_round_belt_state> loop(
      &drake_lcm, shared_diagram, round_belt_state_receiver, "ROUND_BELT_STATE",
      true);
  drake::log()->info("Round belt state estimation started");
  loop.Simulate(std::numeric_limits<double>::infinity());
  return 0;
}
}  // namespace magna
}  // namespace examples
}  // namespace dairlib

int main(int argc, char* argv[]) {
  return dairlib::examples::magna::DoMain(argc, argv);
}