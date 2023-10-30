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
#include "sensor_msgs/JointState.h"
//#include "dairlib/lcmt_c3.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_franka_state.hpp"
#include "dairlib/lcmt_estimated_object_state.hpp"

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
                    std_msgs::Float64MultiArray* output) const;
  int vector_size_;
};

// NOTE: this class appends 7 zeros to the position and 6 zeros
// to the velocity fields.  This was done since this class was hard
// coded for the C3 Franka experiments.
class RosToLcmRobotState : public drake::systems::LeafSystem<double> {
 public:
  static std::unique_ptr<RosToLcmRobotState> Make(int num_positions, int num_velocities, int num_efforts) {
    return std::make_unique<RosToLcmRobotState>(num_positions, num_velocities, num_efforts);
  }

  explicit RosToLcmRobotState(int num_positions, int num_velocities, int num_efforts);

 private:
  void ConvertToLCM(const drake::systems::Context<double>& context,
                    dairlib::lcmt_robot_output* output) const;
  int num_positions_;
  int num_velocities_;
  int num_efforts_;
  const int num_franka_joints_{7};

  const std::vector<double> default_ball_position_ {1, 0, 0, 0, 0.5, 0, 0.0315-0.0301};

  const std::vector<std::string> position_names_ {
      "panda_joint1",
      "panda_joint2",
      "panda_joint3",
      "panda_joint4",
      "panda_joint5",
      "panda_joint6",
      "panda_joint7",
      "base_qw",
      "base_qx",
      "base_qy",
      "base_qz",
      "base_x",
      "base_y",
      "base_z"};

  const std::vector<std::string> velocity_names_ {
      "panda_joint1dot",
      "panda_joint2dot",
      "panda_joint3dot",
      "panda_joint4dot",
      "panda_joint5dot",
      "panda_joint6dot",
      "panda_joint7dot",
      "base_wx",
      "base_wy",
      "base_wz",
      "base_vx",
      "base_vy",
      "base_vz"};

  const std::vector<std::string> effort_names_ {
      "panda_motor1",
      "panda_motor2",
      "panda_motor3",
      "panda_motor4",
      "panda_motor5",
      "panda_motor6",
      "panda_motor7",};

};

//class ROSToC3LCM : public drake::systems::LeafSystem<double> {
// public:
//  static std::unique_ptr<ROSToC3LCM> Make(int num_positions, int num_velocities,
//                                          int lambda_size, int misc_size) {
//
//    return std::make_unique<ROSToC3LCM>(num_positions, num_velocities,
//                                        lambda_size, misc_size);
//  }
//
//  explicit ROSToC3LCM(int num_positions, int num_velocities,
//                      int lambda_size, int misc_size);
//
// private:
//  void ConvertToLCM(const drake::systems::Context<double>& context,
//                    dairlib::lcmt_c3* output) const;
//  int num_positions_;
//  int num_velocities_;
//  int lambda_size_;
//  int misc_size_;
//  int data_size_;
//};

/// More specific translator from Franka state to LCM. Similar to lcmt_cassie_out.lcm
class RosToLcmFrankaState : public drake::systems::LeafSystem<double> {
 public:
  static std::unique_ptr<RosToLcmFrankaState> Make() {

    return std::make_unique<RosToLcmFrankaState>();
  }

  explicit RosToLcmFrankaState();

 private:
  void ConvertToLCM(const drake::systems::Context<double>& context,
                    dairlib::lcmt_franka_state* franka_state) const;
};

class RosToLcmObjectState : public drake::systems::LeafSystem<double> {
 public:
  static std::unique_ptr<RosToLcmObjectState> Make() {

    return std::make_unique<RosToLcmObjectState>();
  }

  explicit RosToLcmObjectState();

 private:
  void ConvertToLCM(const drake::systems::Context<double>& context,
                    dairlib::lcmt_object_state* franka_state) const;

  std::map<double, std::string> enum_map_ =
      {{-3.0, "Invalid"},         // invalid ball measurement obtained (i.e. incorrectly identified ball)
       {-2.0, "Outlier"},         // outlier ball measurement obtained (i.e. too diff than other measurements)
       {-1.0, "Ball not found"},  // no ball measurement obtained
       { 0.0, "N/A"},             // See below
       { 1.0, "Valid"}};          // valid ball measurement obtained

};

}  // namespace systems
}  // namespace dairlib
