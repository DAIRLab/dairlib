#pragma once
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
using drake::systems::BasicVector;

namespace dairlib {
namespace systems {
/// Receives the output of an LcmSubscriberSystem that subscribes to the
/// franka_hand_target_position with LCM type lcmt_franka_hand_target_position
/// and outputs a constant trajectory of the hand position.
class FrankaHandTargetPositionReceiver
    : public drake::systems::LeafSystem<double> {
 public:
  explicit FrankaHandTargetPositionReceiver();

 private:
  void CalcTraj(const drake::systems::Context<double>& context,
                drake::AbstractValue* traj) const;
};
}  // namespace systems
}  // namespace dairlib
