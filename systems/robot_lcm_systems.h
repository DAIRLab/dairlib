#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "systems/framework/state_vector.h"
#include "systems/framework/timestamped_vector.h"
#include "multibody/rbt_utils.h"

#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_robot_input.hpp"

namespace dairlib{
namespace systems{

using systems::StateVector;
using systems::TimestampedVector;
using Eigen::VectorXd;
using drake::systems::LeafSystem;
using drake::systems::Context;

/// @file This file contains classes dealing with sending/receiving
/// LCM messages related to a robot. The classes in this file are based on
/// acrobot_lcm.h

/// Receives the output of an LcmSubsriberSystem that subsribes to the
/// Robot output channel with LCM type lcmt_robot_output, and outputs the
/// robot states as a OutputVector.
class RobotOutputReceiver : public LeafSystem<double> {
 public:
  RobotOutputReceiver(RigidBodyTree<double>& tree);


 private:
  void CopyOutput(const Context<double>& context,
                    StateVector<double>* output) const;
  const RigidBodyTree<double>* tree_;
  map<string, int> positionIndexMap_;
  map<string, int> velocityIndexMap_;
};


/// Converts a StateVector object to LCM type lcmt_robot_output
class RobotOutputSender : public LeafSystem<double> {
 public:
  RobotOutputSender(RigidBodyTree<double>& tree);


 private:
  void Output(const Context<double>& context,
                   dairlib::lcmt_robot_output* output) const;
  const RigidBodyTree<double>* tree_;
  map<string, int> positionIndexMap_;
  map<string, int> velocityIndexMap_;
};

/// Receives the output of an LcmSubsriberSystem that subsribes to the
/// robot input channel with LCM type lcmt_robot_input and outputs the
/// robot inputs as a TimestampedVector.
class RobotInputReceiver : public LeafSystem<double> {
 public:
  RobotInputReceiver(RigidBodyTree<double>& tree);


 private:
  void CopyInputOut(const Context<double>& context,
                    TimestampedVector<double>* output) const;

  const RigidBodyTree<double>* tree_;
  map<string, int> actuatorIndexMap_;
};


/// Receives the output of a controller, and outputs it as an LCM
/// message with type lcm_robot_u. Its output port is usually connected to
/// an LcmPublisherSystem to publish the messages it generates.
class RobotCommandSender : LeafSystem<double> {
 public:
  RobotCommandSender(RigidBodyTree<double>& tree);

 private:
  void OutputCommand(const Context<double>& context,
                     dairlib::lcmt_robot_input* output) const;

  const RigidBodyTree<double>* tree_;
  map<string, int> actuatorIndexMap_;
};

} //namespace systems
} //namespace dairlib