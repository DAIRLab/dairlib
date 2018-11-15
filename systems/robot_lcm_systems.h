#pragma once

#include <string>
#include <map>

#include "drake/systems/framework/leaf_system.h"
#include "systems/framework/output_vector.h"
#include "systems/framework/timestamped_vector.h"
#include "multibody/rbt_utils.h"

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
  RobotOutputReceiver(const RigidBodyTree<double>& tree);


 private:
  void CopyOutput(const drake::systems::Context<double>& context,
                    OutputVector<double>* output) const;
  const RigidBodyTree<double>* tree_;
  std::map<std::string, int> positionIndexMap_;
  std::map<std::string, int> velocityIndexMap_;
};


/// Converts a OutputVector object to LCM type lcmt_robot_output
class RobotOutputSender : public drake::systems::LeafSystem<double> {
 public:
  RobotOutputSender(const RigidBodyTree<double>& tree);

  const drake::systems::InputPort<double>& get_input_port_state()
      const {
    return this->get_input_port(state_input_port_);
  }

 private:
  void Output(const drake::systems::Context<double>& context,
                   dairlib::lcmt_robot_output* output) const;
  const RigidBodyTree<double>* tree_;
  std::map<std::string, int> positionIndexMap_;
  std::map<std::string, int> velocityIndexMap_;
  int state_input_port_;
};

/// Receives the output of an LcmSubsriberSystem that subsribes to the
/// robot input channel with LCM type lcmt_robot_input and outputs the
/// robot inputs as a TimestampedVector.
class RobotInputReceiver : public drake::systems::LeafSystem<double> {
 public:
  RobotInputReceiver(const RigidBodyTree<double>& tree);


 private:
  void CopyInputOut(const drake::systems::Context<double>& context,
                    TimestampedVector<double>* output) const;

  const RigidBodyTree<double>* tree_;
  std::map<std::string, int> actuatorIndexMap_;
};


/// Receives the output of a controller, and outputs it as an LCM
/// message with type lcm_robot_u. Its output port is usually connected to
/// an LcmPublisherSystem to publish the messages it generates.
class RobotCommandSender : public drake::systems::LeafSystem<double> {
 public:
  RobotCommandSender(const RigidBodyTree<double>& tree);

 private:
  void OutputCommand(const drake::systems::Context<double>& context,
                     dairlib::lcmt_robot_input* output) const;

  const RigidBodyTree<double>* tree_;
  std::map<std::string, int> actuatorIndexMap_;
};

}  // namespace systems
}  // namespace dairlib
