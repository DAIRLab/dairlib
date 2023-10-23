#pragma once


#include <map>
#include <string>
#include <vector>

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_interface_system.h"

namespace dairlib {
namespace systems {

class RobotInputToROS : public drake::systems::LeafSystem<double> {
 public:
  explicit RobotInputToROS(drake::multibody::MultibodyPlant<double>& plant);

 private:
  void ConvertToROS(const drake::systems::Context<double>& context,
                    std_msgs::Float64MultiArray* output) const;
  int num_elements_;
};

}
}