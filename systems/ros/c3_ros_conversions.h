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
#include "dairlib/lcmt_c3.hpp"
#include "dairlib/lcmt_robot_output.hpp"

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

class ROSToRobotOutputLCM : public drake::systems::LeafSystem<double> {
 public:
  static std::unique_ptr<ROSToRobotOutputLCM> Make(int num_positions, int num_velocities, int num_efforts) {
    return std::make_unique<ROSToRobotOutputLCM>(num_positions, num_velocities, num_efforts);
  }

  explicit ROSToRobotOutputLCM(int num_positions, int num_velocities, int num_efforts);

 private:
  void ConvertToLCM(const drake::systems::Context<double>& context,
                  dairlib::lcmt_robot_output* output) const;
  int num_positions_;
  int num_velocities_;
  int num_efforts_;
};

class ROSToC3LCM : public drake::systems::LeafSystem<double> {
 public:
  static std::unique_ptr<ROSToC3LCM> Make(int num_positions, int num_velocities,
    int lambda_size, int misc_size) {

    return std::make_unique<ROSToC3LCM>(num_positions, num_velocities,
      lambda_size, misc_size);
  }

  explicit ROSToC3LCM(int num_positions, int num_velocities,
    int lambda_size, int misc_size);

 private:
  void ConvertToLCM(const drake::systems::Context<double>& context,
                  dairlib::lcmt_c3* output) const;
  int num_positions_;
  int num_velocities_;
  int lambda_size_;
  int misc_size_;
  int data_size_;
};



}  // namespace systems
}  // namespace dairlib
