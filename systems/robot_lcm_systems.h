#pragma once

#include <map>
#include <string>
#include <vector>

#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "systems/framework/output_vector.h"
#include "systems/framework/timestamped_vector.h"
#include "systems/primitives/subvector_pass_through.h"

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_interface_system.h"

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
  explicit RobotOutputSender(
      const drake::multibody::MultibodyPlant<double>& plant,
      const bool publish_efforts = false, const bool publish_imu = false);

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_input_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_effort() const {
    if(!publish_efforts_)
      std::cerr << "RobotOutputSender not configured to publish efforts." << std::endl;
    return this->get_input_port(effort_input_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_imu() const {
    return this->get_input_port(imu_input_port_);
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
  int state_input_port_ = -1;
  int effort_input_port_ = -1;
  int imu_input_port_ = -1;
  bool publish_efforts_;
  bool publish_imu_;
};

/// Receives the output of an LcmSubsriberSystem that subsribes to the
/// robot input channel with LCM type lcmt_robot_input and outputs the
/// robot inputs as a TimestampedVector.
class RobotInputReceiver : public drake::systems::LeafSystem<double> {
 public:
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
  explicit RobotCommandSender(
      const drake::multibody::MultibodyPlant<double>& plant);

 private:
  void OutputCommand(const drake::systems::Context<double>& context,
                     dairlib::lcmt_robot_input* output) const;

  int num_actuators_;
  std::vector<std::string> ordered_actuator_names_;
  std::map<std::string, int> actuatorIndexMap_;
};


/// Adds LCM 
///
///
SubvectorPassThrough<double>* AddActuatorAndStateLcm(
    drake::systems::DiagramBuilder<double>* builder,
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::lcm::LcmInterfaceSystem* lcm,
    std::string actuator_channel,
    std::string state_channel,
    double publish_rate,
    bool publish_efforts = true,
    double actuator_delay = 0);

}  // namespace systems
}  // namespace dairlib
