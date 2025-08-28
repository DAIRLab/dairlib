#pragma once
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_system.h"
using drake::systems::BasicVector;

namespace dairlib {
namespace systems {
/// Receives the output of an LcmSubscriberSystem that subscribes to the
/// franka_target_joint_position with LCM type lcmt_franka_target_joint_position
/// and outputs the joint positions as a BasicVector of size 7.
class FrankaTargetJointPositionReceiver
    : public drake::systems::LeafSystem<double> {
 public:
  explicit FrankaTargetJointPositionReceiver();

 private:
  void CopyInputOut(const drake::systems::Context<double>& context,
                    BasicVector<double>* output) const;
};
}  // namespace systems
}  // namespace dairlib
