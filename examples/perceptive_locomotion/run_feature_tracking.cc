#include <gflags/gflags.h>
#include <signal.h>

#include "systems/perception/feature_tracking/feature_tracker.h"

void SigintHandler(int sig) {
  ros::shutdown();
  exit(0);
}

namespace dairlib {

DEFINE_string(params,
"examples/perceptive_locomotion/feature_tracking_node_params.yaml",
"yaml with tracking params");

using perception::FeatureTrackingNode;
using perception::feature_tracking_node_params;

int DoMain(int argc, char** argv) {
  ros::init(argc, argv, "feature_publisher");
  ros::NodeHandle node_handle;
  signal(SIGINT, SigintHandler);

  auto params = drake::yaml::LoadYamlFile<feature_tracking_node_params>(
      FLAGS_params);

  FeatureTrackingNode node(node_handle, params);

  // TODO (@Brian-Acosta) set mask
  ros::spin();
  return 0;
}

}


int main(int argc, char** argv) {
  return dairlib::DoMain(argc, argv);
}