#include "ros_interface_system.h"



namespace dairlib {
namespace systems {

using ros::NodeHandle;

RosInterfaceSystem::RosInterfaceSystem(const std::string& node_name) {

  int argc = 0;
  char** argv = nullptr;

  ros::init(argc, argv, node_name, ros::init_options::NoSigintHandler);
  node_handle_ = std::make_unique<NodeHandle>();

}

RosInterfaceSystem::~RosInterfaceSystem() {
  ros::shutdown();
}

void RosInterfaceSystem::DoCalcNextUpdateTime(
    const drake::systems::Context<double>& context,
    drake::systems::CompositeEventCollection<double>* events, double* t) const {
  ros::spinOnce();

  *t = std::numeric_limits<double>::infinity();
}

}
}