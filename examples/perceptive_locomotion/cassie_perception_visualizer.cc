#include <gflags/gflags.h>
#include <iostream>

#include "geometry/convex_polygon_lcm_systems.h"
#include "geometry/convex_polygon_visualizer.h"
#include "systems/robot_lcm_systems.h"
#include "systems/plant_visualizer.h"
#include "systems/perception/grid_map_lcm_systems.h"
#include "systems/perception/grid_map_visualizer.h"

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

namespace dairlib {

DEFINE_string(channel_x, "NETWORK_CASSIE_STATE_DISPATCHER", "State channel");
DEFINE_string(channel_elevation_map, "CASSIE_ELEVATION_MAP",
              "Elevation mapping grid map channel");
DEFINE_string(channel_convex_polygons, "FOOTHOLDS_PROCESSED",
              "lcm channel with convex terrain decomposition");
DEFINE_double(fps, 20.0, "visualizer update rate");

using systems::RobotOutputReceiver;
using systems::SubvectorPassThrough;
using systems::PlantVisualizer;
using geometry::ConvexPolygonVisualizer;
using geometry::ConvexPolygonReceiver;
using perception::GridMapReceiver;
using perception::GridMapVisualizer;
using drake::multibody::MultibodyPlant;
using drake::systems::Simulator;
using drake::systems::lcm::LcmSubscriberSystem;

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  drake::systems::DiagramBuilder<double> builder;

  const std::string urdf{"examples/Cassie/urdf/cassie_v2_shells.urdf"};
  auto plant_visualizer = builder.AddSystem<PlantVisualizer>(urdf);

  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>(
      "udpm://239.255.76.67:7667?ttl=0");

  std::map<std::string, drake::systems::LeafSystem<double>*> subscribers;
  std::map<std::string, drake::systems::LeafSystem<double>*> receivers;
  std::map<std::string, drake::systems::LeafSystem<double>*> visualizers;

  // state receiver
  subscribers["state"] = builder.AddSystem(
      LcmSubscriberSystem::Make<lcmt_robot_output>(FLAGS_channel_x, lcm)
  );
  receivers["state"] = builder.AddSystem<RobotOutputReceiver>(
      plant_visualizer->get_plant()
  );
  visualizers["state"] =
      reinterpret_cast<drake::systems::LeafSystem<double>*>(plant_visualizer);

  // grid map receiver
  subscribers["grid_map"] = builder.AddSystem(
      LcmSubscriberSystem::Make<lcmt_grid_map>(FLAGS_channel_elevation_map, lcm)
  );
  receivers["grid_map"] = builder.AddSystem<GridMapReceiver>();
  visualizers["grid_map"] = builder.AddSystem<GridMapVisualizer>(
      plant_visualizer->get_meshcat(), 1.0 / FLAGS_fps,
      std::vector<std::string>{"elevation"}
  );

  // polygon receiver
  subscribers["polygons"] = builder.AddSystem(
      LcmSubscriberSystem::Make<lcmt_foothold_set>(
          FLAGS_channel_convex_polygons, lcm
      )
  );
  receivers["polygons"] = builder.AddSystem<ConvexPolygonReceiver>();
  visualizers["polygons"] = builder.AddSystem<ConvexPolygonVisualizer>(
      plant_visualizer->get_meshcat(), 1.0 / FLAGS_fps
  );

  for (const auto key : {"state", "grid_map", "polygons"}) {
    builder.Connect(*subscribers.at(key), *receivers.at(key));
    builder.Connect(*receivers.at(key), *visualizers.at(key));
  }


  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  /// Initialize the lcm subscriber to avoid triggering runtime errors
  /// during initialization due to internal checks
  /// (unit quaternion check in MultibodyPositionToGeometryPose
  /// internal calculations)
  auto& state_sub_context = diagram->GetMutableSubsystemContext(
      *subscribers.at("state"), context.get()
  );
  dynamic_cast<RobotOutputReceiver*>(receivers.at("state"))->InitializeSubscriberPositions(
      plant_visualizer->get_plant(), state_sub_context
  );

  /// Use the simulator to drive at a fixed rate
  /// If set_publish_every_time_step is true, this publishes twice
  /// Set realtime rate. Otherwise, runs as fast as possible
  auto stepper =
  std::make_unique<Simulator<double>>(*diagram, std::move(context));
  stepper->set_publish_every_time_step(false);
  stepper->set_publish_at_initialization(false);
  stepper->set_target_realtime_rate(1.0);
  stepper->Initialize();

  drake::log()->info("visualizer started");

  stepper->AdvanceTo(std::numeric_limits<double>::infinity());

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::do_main(argc, argv); }