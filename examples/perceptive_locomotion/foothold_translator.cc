#include <iostream>
#include <csignal>
#include <gflags/gflags.h>

#include "dairlib/lcmt_foothold_set.hpp"
#include "geometry/convex_polygon_lcm_systems.h"
#include "geometry/convex_polygon_set.h"

#include "geometry/convex_polygon_lcm_systems.h"
#include "geometry/convex_polygon_ros_receiver.h"
#include "systems/ros/ros_subscriber_system.h"
#include "systems/ros/ros_publisher_system.h"

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/lcm/drake_lcm.h"

void SigintHandler(int sig) {
  std::cout << "Received shutdown command!\n";
  ros::shutdown();
  exit(0);
}

namespace dairlib {


using geometry::ConvexPolygon;
using geometry::ConvexPolygonSet;
using drake::systems::Simulator;
using drake::systems::TriggerType;
using drake::systems::TriggerTypeSet;
using drake::systems::lcm::LcmPublisherSystem;



DEFINE_string(foothold_topic, "", "ros topic containing the incoming footholds");
DEFINE_string(foothold_channel, "FOOTHOLDS_PROCESSED",
              "lcm channel for the outgoing processed footholds");
DEFINE_string(debug_channel, "FOOTHOLD_PROCESS_DEBUG", "dbg lcm channel");

DEFINE_double(conv_thresh, 0.15, "Convexity threshold for ACD");


int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  ros::init(argc, argv, "planar_terrain_to_convex_polygon_lcm_bridge");
  ros::NodeHandle node_handle;
  signal(SIGINT, SigintHandler);
  ros::AsyncSpinner spinner(1);

  drake::systems::DiagramBuilder<double> builder;
  drake::lcm::DrakeLcm lcm_local("udpm://239.255.76.67:7667?ttl=0");

  auto plane_subscriber = builder.AddSystem(
      systems::RosSubscriberSystem<
          convex_plane_decomposition_msgs::PlanarTerrain>::Make(
                  FLAGS_foothold_topic, &node_handle));

  auto plane_receiver =
      builder.AddSystem<geometry::ConvexPolygonRosReceiver>(FLAGS_conv_thresh);
  auto foothold_sender = builder.AddSystem<geometry::ConvexPolygonSender>();
  auto foothold_publisher = builder.AddSystem(
      LcmPublisherSystem::Make<lcmt_foothold_set>(
          FLAGS_foothold_channel, &lcm_local,
          TriggerTypeSet{TriggerType::kForced})
      );
  auto debug_publisher = builder.AddSystem(
      LcmPublisherSystem::Make<lcmt_convex_decomposition_debug>(
          FLAGS_debug_channel, &lcm_local,
          TriggerTypeSet{TriggerType::kForced})
      );
  builder.Connect(*plane_subscriber, *plane_receiver);
  builder.Connect(plane_receiver->get_output_port_footholds(),
                  foothold_sender->get_input_port());
  builder.Connect(plane_receiver->get_output_port_debug(),
                  debug_publisher->get_input_port());
  builder.Connect(*foothold_sender, *foothold_publisher);

  // Create the diagram
  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();
  auto simulator = std::make_unique<Simulator<double>>(*diagram, std::move(context));
  auto& sim_context = simulator->get_mutable_context();

  ros::Rate r(100.0); // 100 Hz

  spinner.start();
  plane_subscriber->WaitForMessage(0);
  spinner.stop();

  double t_begin = ros::Time::now().toSec();

  std::cout << "First message received, starting foothold translator\n";
  // Poor man's LcmDrivenLoop with synchronous ros spinning
  while (true) {
    ros::spinOnce();
    double elapsed = ros::Time::now().toSec() - t_begin;
    simulator->AdvanceTo(elapsed);
    diagram->CalcForcedUnrestrictedUpdate(
        sim_context, &sim_context.get_mutable_state());
    diagram->ForcedPublish(sim_context);
    r.sleep();
  }

}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::DoMain(argc, argv); }
