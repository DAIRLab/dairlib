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

  /// Convenience function to initialize an lcmt_robot_output subscriber with
  /// positions and velocities which are all zero except for the quaternion
  /// positions, which are all 1, 0, 0, 0
  /// @param context The context of a
  ///                drake::LcmSubScriberSystem<lcmt_robot_output>
  void InitializeSubscriberPositions(
      const drake::multibody::MultibodyPlant<double>& plant,
      drake::systems::Context<double>& context) const;

 private:
  void CopyOutput(const drake::systems::Context<double>& context,
                  OutputVector<double>* output) const;
  int num_positions_;
  int num_velocities_;
  int num_efforts_;
  std::map<std::string, int> position_index_map_;
  std::map<std::string, int> velocity_index_map_;
  std::map<std::string, int> effort_index_map_;
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
    DRAKE_DEMAND(publish_efforts_);
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
  std::map<std::string, int> position_index_map_;
  std::map<std::string, int> velocity_index_map_;
  std::map<std::string, int> effort_index_map_;
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
  std::map<std::string, int> actuator_index_map_;
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
  std::map<std::string, int> actuator_index_map_;
};


///
/// Convenience method to add and connect leaf systems for controlling
/// a MultibodyPlant via LCM. Makes two primary connections:
///  (1) connects the state output from the plant to a RobotOutputSender and
///      LCM publisher, publishing lcmt_robot_output
///  (2) connects the actuation input port of the plant to a RobotInputReceiver
///      and LCM receiver, receiving lcmt_robot_input
///
/// If an actuator delay is specified, also adds a DiscreteTimeDelay block
/// between the input receiver and plant. This delay will also be applied to the
/// efforts sent to the RobotOutputSender (if publish_efforts = true).
///
/// @param builder
/// @param lcm The LcmInterfaceSystem to publish and receive on. Typically
///        from `builder.AddSystem(LcmInterfaceSystem(...))`;
/// @param actuator_channel The LCM channel name for the lcmt_robot_input
/// @param state_channel The LCM channel name for the lcmt_robot_output
/// @param publish_rate The frequency, in Hz, to publish at
/// @param publish_efforts If true, the RobotOutputSender will also publish
///        actuator efforts.
/// @param actuator_delay The delay, in seconds, will be discretized according
///        to publish_rate
SubvectorPassThrough<double>* AddActuationRecieverAndStateSenderLcm(
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
