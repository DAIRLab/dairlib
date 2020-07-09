#pragma once

#include <string>
#include <map>
#include <vector>

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"
#include "systems/framework/output_vector.h"
#include "systems/framework/timestamped_vector.h"

#include "attic/multibody/rigidbody_utils.h"
#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_robot_input.hpp"

namespace dairlib {
namespace systems {

/// @file This file contains classes dealing with sending/receiving
/// LCM messages related to a robot. The classes in this file are based on
/// acrobot_lcm.h

/// Receives the output of an LcmSubsriberSystem that subsribes to the
/// Robot output channel with LCM type lcmt_robot_output, and outputs the
/// robot states as a OutputVector.
class RobotOutputReceiver : public drake::systems::LeafSystem<double> {
 public:
  explicit RobotOutputReceiver(const RigidBodyTree<double>& tree);

  explicit RobotOutputReceiver(
    const drake::multibody::MultibodyPlant<double>& plant);


 private:
  void CopyOutput(const drake::systems::Context<double>& context,
                    OutputVector<double>* output) const;
  int num_positions_;
  int num_velocities_;
  int num_efforts_;
  std::map<std::string, int> positionIndexMap_;
  std::map<std::string, int> velocityIndexMap_;
  std::map<std::string, int> effortIndexMap_;
};


/// Converts a OutputVector object to LCM type lcmt_robot_output
class RobotOutputSender : public drake::systems::LeafSystem<double> {
 public:
  explicit RobotOutputSender(const RigidBodyTree<double>& tree,
      const bool publish_efforts=false);

  explicit RobotOutputSender(
    const drake::multibody::MultibodyPlant<double>& plant,
    const bool publish_efforts=false);;

  const drake::systems::InputPort<double>& get_input_port_state()
      const {
    return this->get_input_port(state_input_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_effort()
      const {
    return this->get_input_port(effort_input_port_);
  }

 private:
  void Output(const drake::systems::Context<double>& context,
                   dairlib::lcmt_robot_output* output) const;

  int num_positions_;
  int num_velocities_;
  int num_efforts_;
  std::vector<std::string> ordered_position_names_;
  std::vector<std::string> ordered_velocity_names_;
  std::vector<std::string> ordered_effort_names_;
  std::map<std::string, int> positionIndexMap_;
  std::map<std::string, int> velocityIndexMap_;
  std::map<std::string, int> effortIndexMap_;
  int state_input_port_;
  int effort_input_port_;
  bool publish_efforts_;
};

/// Receives the output of an LcmSubsriberSystem that subsribes to the
/// robot input channel with LCM type lcmt_robot_input and outputs the
/// robot inputs as a TimestampedVector.
class RobotInputReceiver : public drake::systems::LeafSystem<double> {
 public:
  explicit RobotInputReceiver(const RigidBodyTree<double>& tree);

  explicit RobotInputReceiver(
      const drake::multibody::MultibodyPlant<double>& plant);


 private:
  void CopyInputOut(const drake::systems::Context<double>& context,
                    TimestampedVector<double>* output) const;

  int num_actuators_;
  std::map<std::string, int> actuatorIndexMap_;
};


/// Receives the output of a controller, and outputs it as an LCM
/// message with type lcm_robot_u. Its output port is usually connected to
/// an LcmPublisherSystem to publish the messages it generates.
class RobotCommandSender : public drake::systems::LeafSystem<double> {
 public:
  explicit RobotCommandSender(const RigidBodyTree<double>& tree);

  explicit RobotCommandSender(
      const drake::multibody::MultibodyPlant<double>& plant);

 private:
  void OutputCommand(const drake::systems::Context<double>& context,
                     dairlib::lcmt_robot_input* output) const;

  int num_actuators_;
  std::vector<std::string> ordered_actuator_names_;
  std::map<std::string, int> actuatorIndexMap_;
};

}  // namespace systems
}  // namespace dairlib
