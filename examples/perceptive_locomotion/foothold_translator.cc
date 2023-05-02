#include <iostream>
#include <signal.h>
#include <gflags/gflags.h>

#include "dairlib/lcmt_foothold_set.hpp"
#include "geometry/convex_foothold_lcm_systems.h"
#include "geometry/convex_foothold_set.h"

#include "geometry/convex_foothold_lcm_systems.h"
#include "geometry/convex_foothold_ros_receiver.h"
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


using geometry::ConvexFoothold;
using geometry::ConvexFootholdSet;
using drake::systems::Simulator;
using drake::systems::TriggerType;
using drake::systems::TriggerTypeSet;
using drake::systems::lcm::LcmPublisherSystem;



DEFINE_string(foothold_topic, "", "ros topic containing the incoming footholds");
DEFINE_string(foothold_channel, "FOOTHOLDS_PROCESSED",
              "lcm channel for the outgoing processed footholds");

DEFINE_double(conv_thresh, 0.15, "Convexity threshold for ACD");


int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  ros::init(argc, argv, "planar_terrain_to_convex_foothold_lcm_bridge");
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
      builder.AddSystem<geometry::ConvexFootholdRosReceiver>(FLAGS_conv_thresh);
  auto foothold_sender = builder.AddSystem<geometry::ConvexFootholdSender>();
  auto foothold_publisher = builder.AddSystem(
      LcmPublisherSystem::Make<lcmt_foothold_set>(
          FLAGS_foothold_channel, &lcm_local,
          TriggerTypeSet{TriggerType::kPerStep})
      );
  builder.Connect(*plane_subscriber, *plane_receiver);
  builder.Connect(*plane_receiver, *foothold_sender);
  builder.Connect(*foothold_sender, *foothold_publisher);

  // Create the diagram
  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  // Run lcm-driven simulation
  auto simulator = std::make_unique<Simulator<double>>(*diagram, std::move(context));
  ros::Rate r(100.0); // 50 Hz

  spinner.start();
  plane_subscriber->WaitForMessage(0);
  spinner.stop();

  double t_begin = ros::Time::now().toSec();

  std::cout << "First message received, starting foothold translator\n";
  while (true) {
    ros::spinOnce();
    double elapsed = ros::Time::now().toSec() - t_begin;
    simulator->AdvanceTo(elapsed);
    r.sleep();
  }

}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::DoMain(argc, argv); }
