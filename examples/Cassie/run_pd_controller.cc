#include <gflags/gflags.h>

#include "systems/primitives/output_subvector_pass_through.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

#include "dairlib/lcmt_pd_config.hpp"
#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "examples/Cassie/cassie_utils.h"
#include "systems/controllers/linear_controller.h"
#include "systems/controllers/pd_config_lcm.h"
#include "systems/robot_lcm_systems.h"

namespace dairlib {

using drake::geometry::SceneGraph;
using drake::multibody::MultibodyPlant;
using drake::systems::DiagramBuilder;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;

// lcm channels
DEFINE_string(channel_x, "CASSIE_STATE_DISPATCHER",
              "LCM channel for receiving state. "
              "Use CASSIE_STATE_SIMULATION to get state from simulator, and "
              "use CASSIE_STATE_DISPATCHER to get state from state estimator");
DEFINE_string(channel_u, "CASSIE_INPUT",
              "The name of the channel which publishes command");

// Cassie model parameter
DEFINE_bool(floating_base, true, "Fixed or floating base model");

int doMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  DiagramBuilder<double> builder;
  DiagramBuilder<double> builder_null;

  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>(
      "udpm://239.255.76.67:7667?ttl=0");

  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  SceneGraph<double>& scene_graph_null = *builder_null.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  MultibodyPlant<double>& plant_fixed_base =
      *builder_null.AddSystem<MultibodyPlant>(1.0);
  MultibodyPlant<double>& plant = *builder_null.AddSystem<MultibodyPlant>(1.0);

  addCassieMultibody(&plant_fixed_base, &scene_graph_null, false);
  if (FLAGS_floating_base) {
    addCassieMultibody(&plant, &scene_graph_null, true);
  } else {
    addCassieMultibody(&plant, &scene_graph_null, false);
  }
  plant_fixed_base.Finalize();
  plant.Finalize();

  int nq_fixed_base = plant_fixed_base.num_positions();
  int nv_fixed_base = plant_fixed_base.num_velocities();
  int nu_fixed_base = plant_fixed_base.num_actuators();
  int nq = plant.num_positions();
  int nv = plant.num_velocities();
  int nu = plant.num_actuators();

  std::cout << "nq_fixed_base" << nq_fixed_base << std::endl;
  std::cout << "nv_fixed_base" << nv_fixed_base << std::endl;
  std::cout << "nu_fixed_base" << nu_fixed_base << std::endl;

  const std::string channel_x = FLAGS_channel_x;
  const std::string channel_u = FLAGS_channel_u;
  const std::string channel_config = "PD_CONFIG";

  // Create state receiver.
  auto state_sub = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_robot_output>(channel_x, lcm));
  auto state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(plant);
  builder.Connect(state_sub->get_output_port(),
                  state_receiver->get_input_port(0));

  // Create config receiver.
  auto config_sub = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_pd_config>(channel_config, lcm));
  auto config_receiver =
      builder.AddSystem<systems::PDConfigReceiver>(plant_fixed_base);
  builder.Connect(config_sub->get_output_port(),
                  config_receiver->get_input_port(0));

  // Create command sender.
  auto command_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          channel_u, lcm, 1.0 / 1000.0));
  auto command_sender =
      builder.AddSystem<systems::RobotCommandSender>(plant_fixed_base);

  builder.Connect(command_sender->get_output_port(0),
                  command_pub->get_input_port());

  auto controller = builder.AddSystem<systems::LinearController>(
      plant_fixed_base.num_positions(), plant_fixed_base.num_velocities(),
      plant_fixed_base.num_actuators());

  auto floating_to_fixed_base =
      builder.AddSystem<systems::OutputSubvectorPassThrough<double>>(
          nq, nv, nu, nq - nq_fixed_base, nq_fixed_base,
          nq + nv - nv_fixed_base, nv_fixed_base, nq + nv + nu - nu_fixed_base,
          nu_fixed_base);

  builder.Connect(state_receiver->get_output_port(0),
                  floating_to_fixed_base->get_input_port());

  builder.Connect(floating_to_fixed_base->get_output_port(),
                  controller->get_input_port_output());

  builder.Connect(config_receiver->get_output_port(0),
                  controller->get_input_port_config());

  std::cout << controller->get_output_port(0).size() << std::endl;
  std::cout << command_sender->get_input_port(0).size() << std::endl;
  builder.Connect(controller->get_output_port(0),
                  command_sender->get_input_port(0));

  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  /// Use the simulator to drive at a fixed rate
  /// If set_publish_every_time_step is true, this publishes twice
  /// Set realtime rate. Otherwise, runs as fast as possible
  auto stepper = std::make_unique<drake::systems::Simulator<double>>(
      *diagram, std::move(context));
  stepper->set_publish_every_time_step(false);
  stepper->set_publish_at_initialization(false);
  stepper->set_target_realtime_rate(1.0);
  stepper->Initialize();

  drake::log()->info("controller started");

  stepper->AdvanceTo(std::numeric_limits<double>::infinity());

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::doMain(argc, argv); }
