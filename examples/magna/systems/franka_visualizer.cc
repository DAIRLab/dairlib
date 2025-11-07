#include <drake/systems/primitives/vector_log_sink.h>
#include <gflags/gflags.h>

#include "common/find_resource.h"
#include "dairlib/lcmt_robot_output.hpp"
#include "examples/magna/systems/franka_common.h"
#include "systems/primitives/subvector_pass_through.h"
#include "systems/robot_lcm_systems.h"

#include "drake/common/yaml/yaml_io.h"
#include "drake/geometry/meshcat_visualizer.h"
#include "drake/geometry/meshcat_visualizer_params.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/rendering/multibody_position_to_geometry_pose.h"

using drake::geometry::Meshcat;
using drake::geometry::MeshcatVisualizerParams;
using drake::geometry::SceneGraph;
using drake::multibody::MultibodyPlant;
using drake::systems::Simulator;
using drake::systems::TriggerType;
using drake::systems::TriggerTypeSet;
using drake::systems::VectorLogSink;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::rendering::MultibodyPositionToGeometryPose;

DEFINE_string(franka_state_channel, "FRANKA_WITH_HAND_STATE",
              "LCM channel for receiving Franka state");
DEFINE_uint32(franka_visualizer_publish_rate, 30,
              "Publish rate for the Franka visualizer in Hz");
DEFINE_string(lcm_url, "udpm://239.255.76.67:7667?ttl=0",
              "LCM URL with IP, port, and TTL settings");

namespace dairlib {

using systems::ObjectStateReceiver;
using systems::RobotOutputReceiver;
using systems::SubvectorPassThrough;

namespace examples {
namespace magna {
namespace systems {

int RunFrankaVisualizer() {
  // setup lcm

  drake::systems::DiagramBuilder<double> builder;
  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  MultibodyPlant<double> plant(0.0);
  drake::multibody::ModelInstanceIndex franka_index = AddFrankaToPlant(
      &plant, &scene_graph, std::nullopt, true /* with hand */);
  plant.Finalize();

  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>();

  // Create state receiver.
  auto franka_state_subscriber =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_robot_output>(
          FLAGS_franka_state_channel, lcm));
  auto franka_state_debug =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_output>(
          FLAGS_franka_state_channel + "_debug", lcm,
          TriggerTypeSet({TriggerType::kPeriodic}), 0.1));
  builder.Connect(franka_state_subscriber->get_output_port(),
                  franka_state_debug->get_input_port());
  auto franka_state_receiver =
      builder.AddSystem<RobotOutputReceiver>(plant, franka_index);
  builder.Connect(franka_state_subscriber->get_output_port(),
                  franka_state_receiver->get_input_port());

  auto franka_position_passthrough = builder.AddSystem<SubvectorPassThrough>(
      franka_state_receiver->get_output_port(0).size(), 0,
      plant.num_positions(franka_index));
  builder.Connect(franka_state_receiver->get_output_port(),
                  franka_position_passthrough->get_input_port());

  auto franka_time_passthrough = builder.AddSystem<SubvectorPassThrough>(
      franka_state_receiver->get_output_port(0).size(),
      franka_state_receiver->get_output_port(0).size() - 1, 1);
  builder.Connect(franka_state_receiver->get_output_port(),
                  franka_time_passthrough->get_input_port());

  auto to_pose =
      builder.AddSystem<MultibodyPositionToGeometryPose<double>>(plant);
  builder.Connect(franka_position_passthrough->get_output_port(),
                  to_pose->get_input_port());
  builder.Connect(
      to_pose->get_output_port(),
      scene_graph.get_source_pose_port(plant.get_source_id().value()));

  drake::geometry::MeshcatVisualizerParams params;
  params.publish_period = 1.0 / FLAGS_franka_visualizer_publish_rate;
  auto meshcat = std::make_shared<drake::geometry::Meshcat>();
  auto visualizer = &drake::geometry::MeshcatVisualizer<double>::AddToBuilder(
      &builder, scene_graph, meshcat, std::move(params));

  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  auto& franka_state_subscriber_context = diagram->GetMutableSubsystemContext(
      *franka_state_subscriber, context.get());
  franka_state_receiver->InitializeSubscriberPositions(
      plant, franka_state_subscriber_context);

  /// Use the simulator to drive at a fixed rate
  /// If set_publish_every_time_step is true, this publishes twice
  /// Set realtime rate. Otherwise, runs as fast as possible
  auto simulator =
      std::make_unique<Simulator<double>>(*diagram, std::move(context));
  simulator->set_publish_every_time_step(false);
  simulator->set_publish_at_initialization(false);
  simulator->set_target_realtime_rate(
      1.0);  // may need to change this to param.real_time_rate?
  simulator->Initialize();
  drake::log()->info("Starting Franka visualizer...");

  simulator->AdvanceTo(std::numeric_limits<double>::infinity());
  return 0;
}

}  // namespace systems
}  // namespace magna
}  // namespace examples
}  // namespace dairlib

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return dairlib::examples::magna::systems::RunFrankaVisualizer();
}
