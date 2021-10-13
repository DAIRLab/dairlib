#pragma once
#include "multibody/kinematic/kinematic_evaluator.h"

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/context.h"

namespace dairlib {
namespace multibody {

/// Simple evaluator for a one dimensional fixed joint
template <typename T>
class FixedJointEvaluator : public KinematicEvaluator<T> {
 public:
  /// Constructor for FixedJointEvaluator
  /// @param plant
  /// @param pos_idx index in the generalized positions
  /// @param vel_idx index in the generalized velocities

  FixedJointEvaluator(const drake::multibody::MultibodyPlant<T>& plant,
                      int pos_idx, int vel_idx, double pos_value);

  drake::VectorX<T> EvalFull(
      const drake::systems::Context<T>& context) const override;

  void EvalFullJacobian(const drake::systems::Context<T>& context,
                        drake::EigenPtr<drake::MatrixX<T>> J) const override;

  drake::VectorX<T> EvalFullJacobianDotTimesV(
      const drake::systems::Context<T>& context) const override;

  using KinematicEvaluator<T>::EvalFullJacobian;
  using KinematicEvaluator<T>::plant;

 private:
  const int pos_idx_;
  const double pos_value_;
  drake::MatrixX<T> J_;
};

}  // namespace multibody
}  // namespace dairlib
