#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "systems/framework/state_vector.h"
#include "systems/framework/timestamped_vector.h"
#include "multibody/rbt_utils.h"

#include "dairlib/lcmt_cassie_state.hpp"
#include "dairlib/lcmt_cassie_input.hpp"
#include "dairlib/lcmt_cassie_pd_config.hpp"
#include "cassie_controller_lcm.h"

namespace dairlib{

using systems::StateVector;
using systems::TimestampedVector;
using Eigen::VectorXd;
using drake::systems::LeafSystem;
using drake::systems::Context;

/// @file This file contains classes dealing with sending/receiving
/// LCM messages related to Cassie. The classes in this file are based on
/// acrobot_lcm.h

/// Receives the output of an LcmSubsriberSystem that subsribes to the
/// Cassie state channel with LCM type lcmt_cassie_state, and outputs the
/// Cassie states as a StateVector.
class CassieStateReceiver : public LeafSystem<double> {
 public:
  CassieStateReceiver(RigidBodyTree<double>& tree);


 private:
  void CopyStateOut(const Context<double>& context,
                    StateVector<double>* output) const;
  const RigidBodyTree<double>* tree_;
  map<string, int> positionIndexMap_;
  map<string, int> velocityIndexMap_;
};


/// Converts a StateVector object to LCM type lcmt_cassie_state
class CassieStateSender : public LeafSystem<double> {
 public:
  CassieStateSender(RigidBodyTree<double>& tree);


 private:
  void OutputState(const Context<double>& context,
                   dairlib::lcmt_cassie_state* output) const;
  const RigidBodyTree<double>* tree_;
  map<string, int> positionIndexMap_;
  map<string, int> velocityIndexMap_;
};

/// Receives the output of an LcmSubsriberSystem that subsribes to the
/// Cassie input channel with LCM type lcmt_input_state, and outputs the
/// Cassie states as a TimestampedVector.
class CassieInputReceiver : public LeafSystem<double> {
 public:
  CassieInputReceiver(RigidBodyTree<double>& tree);


 private:
  void CopyInputOut(const Context<double>& context,
                    TimestampedVector<double>* output) const;

  const RigidBodyTree<double>* tree_;
  map<string, int> actuatorIndexMap_;
};


/// Receives the output of a controller, and outputs it as an LCM
/// message with type lcm_cassie_u. Its output port is usually connected to
/// an LcmPublisherSystem to publish the messages it generates.
class CassieCommandSender : LeafSystem<double> {
 public:
  CassieCommandSender(RigidBodyTree<double>& tree);

 private:
  void OutputCommand(const Context<double>& context,
                     dairlib::lcmt_cassie_input* output) const;

  const RigidBodyTree<double>* tree_;
  map<string, int> actuatorIndexMap_;
};
}
