#pragma once

#include <map>
#include <string>
#include <vector>

#include "systems/framework/output_vector.h"
#include "systems/framework/timestamped_vector.h"

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_interface_system.h"

#include "std_msgs/Float64MultiArray.h"

namespace dairlib {
namespace systems {

class TimestampedVectorToROS : public drake::systems::LeafSystem<double> {
 public:
  static std::unique_ptr<TimestampedVectorToROS> Make(int num_elements) {
    return std::make_unique<TimestampedVectorToROS>(num_elements);
  }

  explicit TimestampedVectorToROS(int num_elements);

 private:
  void ConvertToROS(const drake::systems::Context<double>& context,
                  std_msgs::Float64MultiArray* output) const;
  int num_elements_;
};


}  // namespace systems
}  // namespace dairlib
