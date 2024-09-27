#include "systems/perception/realsense/single_rs_interface.h"
#include "systems/perception/realsense/realsense_point_cloud_subscriber.h"
#include "elevation_mapping/PointXYZRGBConfidenceRatio.hpp"


#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/analysis/simulator.h"


namespace rs2_systems {

using drake::systems::DiagramBuilder;
using dairlib::perception::RealsensePointCloudSubscriber;

int DoMain(int argc, char**argv) {
  auto interface = SingleRSInterface();

  auto builder = DiagramBuilder<double>();

  auto sub = builder.AddSystem<
      RealsensePointCloudSubscriber<pcl::PointXYZRGBConfidenceRatio>>(
          &interface);

  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  drake::systems::Simulator<double> simulator(*diagram, std::move(context));

  interface.Start();

  sub->WaitForFrame();

  double dt = 0.001;
  double t = 0;
  while (1) {
    simulator.AdvanceTo(t + dt);
    t += dt;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }


  interface.Stop();

  return 0;
}

}

int main(int argc, char**argv) {
  return rs2_systems::DoMain(argc, argv);
}