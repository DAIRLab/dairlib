#pragma once

#include <map>
#include <string>
#include <vector>

#include "dairlib/lcmt_object_state.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_fingertips_delta_position.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "systems/framework/output_vector.h"
#include "systems/framework/timestamped_vector.h"
#include "trifinger_msgs/msg/trifinger_action.hpp"
#include "trifinger_msgs/msg/trifinger_state.hpp"
#include "trifinger_msgs/msg/fingertip_delta_position.hpp"

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_interface_system.h"

namespace dairlib {
namespace systems {

class LcmToRosTimestampedVector : public drake::systems::LeafSystem<double> {
 public:
  static std::unique_ptr<LcmToRosTimestampedVector> Make(int vector_size) {
    return std::make_unique<LcmToRosTimestampedVector>(vector_size);
  }

  explicit LcmToRosTimestampedVector(int vector_size);

 private:
  void ConvertToROS(const drake::systems::Context<double>& context,
                    std_msgs::msg::Float64MultiArray* output) const;
  int vector_size_;
};

class RosToLcmRobotState : public drake::systems::LeafSystem<double> {
 public:
  static std::unique_ptr<RosToLcmRobotState> Make(int num_positions,
                                                  int num_velocities,
                                                  int num_efforts) {
    return std::make_unique<RosToLcmRobotState>(num_positions, num_velocities,
                                                num_efforts);
  }

  explicit RosToLcmRobotState(int num_positions, int num_velocities,
                              int num_efforts);

 private:
  void ConvertToLCM(const drake::systems::Context<double>& context,
                    dairlib::lcmt_robot_output* output) const;
  int num_positions_;
  int num_velocities_;
  int num_efforts_;

  const std::vector<std::string> position_names_{
      "finger_base_to_upper_joint_0",     "finger_upper_to_middle_joint_0",
      "finger_middle_to_lower_joint_0",   "finger_base_to_upper_joint_120",
      "finger_upper_to_middle_joint_120", "finger_middle_to_lower_joint_120",
      "finger_base_to_upper_joint_240",   "finger_upper_to_middle_joint_240",
      "finger_middle_to_lower_joint_240",
  };

  const std::vector<std::string> velocity_names_{
      "finger_base_to_upper_joint_0dot",
      "finger_upper_to_middle_joint_0dot",
      "finger_middle_to_lower_joint_0dot",
      "finger_base_to_upper_joint_120dot",
      "finger_upper_to_middle_joint_120dot",
      "finger_middle_to_lower_joint_120dot",
      "finger_base_to_upper_joint_240dot",
      "finger_upper_to_middle_joint_240dot",
      "finger_middle_to_lower_joint_240dot",
  };

  const std::vector<std::string> effort_names_{
      "finger_base_to_upper_joint_0_torque",
      "finger_upper_to_middle_joint_0_torque",
      "finger_middle_to_lower_joint_0_torque",
      "finger_base_to_upper_joint_120_torque",
      "finger_upper_to_middle_joint_120_torque",
      "finger_middle_to_lower_joint_120_torque",
      "finger_base_to_upper_joint_240_torque",
      "finger_upper_to_middle_joint_240_torque",
      "finger_middle_to_lower_joint_240_torque",
  };
};

class RosToLcmObjectState : public drake::systems::LeafSystem<double> {
 public:
  static std::unique_ptr<RosToLcmObjectState> Make(
      const drake::multibody::MultibodyPlant<double>& plant,
      drake::multibody::ModelInstanceIndex model_instance,
      const std::string& object_name) {
    return std::make_unique<RosToLcmObjectState>(plant, model_instance,
                                                 object_name);
  }

  explicit RosToLcmObjectState(
      const drake::multibody::MultibodyPlant<double>& plant,
      drake::multibody::ModelInstanceIndex model_instance,
      const std::string& object_name);

 private:
  void ConvertToLCM(const drake::systems::Context<double>& context,
                    dairlib::lcmt_object_state* object_state) const;
};

class RosToLcmFingertipsDeltaPosition : public
    drake::systems::LeafSystem<double> {
 public:
  static std::unique_ptr<RosToLcmFingertipsDeltaPosition> Make() {
    return std::make_unique<RosToLcmFingertipsDeltaPosition>();
  }
  explicit RosToLcmFingertipsDeltaPosition();
 private:
  void ConvertToLCM(const drake::systems::Context<double>& context,
                    dairlib::lcmt_fingertips_delta_position* output)
                    const;

  const int n_delta_pos_ = 9;
  mutable int prev_msg_utime_ = -1;
};
}  // namespace systems
}  // namespace dairlib