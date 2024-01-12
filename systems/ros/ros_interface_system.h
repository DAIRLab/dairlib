#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "ros/ros.h"

namespace dairlib {
namespace systems {

/** Leaf system that creates, holds, and manages a ros node */
class RosInterfaceSystem : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RosInterfaceSystem);

  RosInterfaceSystem(const std::string& node_name);
  ~RosInterfaceSystem();

  ros::NodeHandle* node_handle() { return node_handle_.get(); }

 protected:
  void DoCalcNextUpdateTime(const drake::systems::Context<double>&,
                            drake::systems::CompositeEventCollection<double>*,
                            double*) const;

 private:
  std::unique_ptr<ros::NodeHandle> node_handle_;

};

}
}
