#pragma once
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_system.h"
using drake::systems::BasicVector;

namespace dairlib {
namespace systems {
/// Receives the output of an LcmSubscriberSystem that subscribes to the
/// robot input channel with LCM type lcmt_robot_input and outputs the
/// robot inputs as a TimestampedVector.
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
