#pragma once
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_system.h"
using drake::systems::BasicVector;

namespace dairlib {
namespace examples {
namespace magna {
namespace systems {
/// Receives the output of an LcmSubscriberSystem that subscribes to the
/// franka_target_cartesian_pose with LCM type lcmt_franka_cartesian_pose
/// and outputs the cartesian pose as a BasicVector of size 6.
class FrankaTargetCartesianPoseReceiver
    : public drake::systems::LeafSystem<double> {
 public:
  explicit FrankaTargetCartesianPoseReceiver(double expire_time = 1e5);

 private:
  void CopyInputOut(const drake::systems::Context<double>& context,
                    BasicVector<double>* output) const;
  double expire_time_;
};
}  // namespace systems
}  // namespace magna
}  // namespace examples
}  // namespace dairlib
