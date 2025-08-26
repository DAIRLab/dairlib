#pragma once

#include <map>
#include <string>
#include <vector>

#include "dairlib/lcmt_robot_input.hpp"
#include "systems/framework/timestamped_vector.h"

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace systems {
namespace lcmt_systems {
/**
 * Receives the output of an LcmSubscriberSystem that subscribes to the
 * robot input channel with LCM type lcmt_robot_input and outputs the
 * robot inputs as a TimestampedVector.
 */
class RobotInputConsumer : public drake::systems::LeafSystem<double> {
 public:
  explicit RobotInputConsumer(
      const drake::multibody::MultibodyPlant<double>& plant);

 private:
  void CopyInputOut(const drake::systems::Context<double>& context,
                    TimestampedVector<double>* output) const;

  int num_actuators_;
  std::map<std::string, int> actuator_index_map_;
};

/**
 * Receives the output of a controller, and outputs it as an LCM
 * message with type lcm_robot_input. Its output port is usually connected to
 * an LcmPublisherSystem to publish the messages it generates.
 */
class RobotInputGenerator : public drake::systems::LeafSystem<double> {
 public:
  explicit RobotInputGenerator(
      const drake::multibody::MultibodyPlant<double>& plant);

 private:
  void OutputCommand(const drake::systems::Context<double>& context,
                     dairlib::lcmt_robot_input* output) const;

  int num_actuators_;
  std::vector<std::string> ordered_actuator_names_;
  std::map<std::string, int> actuator_index_map_;
};
}  // namespace lcmt_systems
}  // namespace systems
}  // namespace dairlib