#include <iostream>
#include <signal.h>
#include <gflags/gflags.h>

#include "dairlib/lcmt_foothold_set.hpp"
#include "geometry/convex_foothold_lcm_systems.h"
#include "geometry/convex_foothold_set.h"

#include "geometry/convex_foothold_lcm_systems.h"
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


int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  ros::init(argc, argv, "terrain_receiver");
  ros::NodeHandle node_handle;
  signal(SIGINT, SigintHandler);
  ros::AsyncSpinner spinner(1);

  drake::systems::DiagramBuilder<double> builder;
  drake::lcm::DrakeLcm lcm_local("udpm://239.255.76.67:7667?ttl=0");

  auto plane_subscriber = builder.AddSystem(
      systems::RosSubscriberSystem<
          convex_plane_decomposition_msgs::PlanarTerrain>::Make(
                  FLAGS_foothold_topic, &node_handle));

  auto plane_receiver = builder.AddSystem<geometry::ConvexFootholdRosReceiver>();
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
  auto stepper = std::make_unique<Simulator<double>>(*diagram, std::move(context));
  stepper->reset_integrator<drake::systems::RungeKutta2Integrator<double>>(.05);
  stepper->set_publish_every_time_step(true);
  stepper->set_publish_at_initialization(false);
  stepper->get_mutable_integrator().set_maximum_step_size(0.05);
  stepper->set_target_realtime_rate(1.0);
  stepper->Initialize();

  spinner.start();
  std::cout << "starting simulation" << std::endl;
  double t = 0;
  while (t < std::numeric_limits<double>::infinity()) {
    t += .000001;
    stepper->AdvanceTo(t);
  }
  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::DoMain(argc, argv); }
