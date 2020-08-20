#pragma once
#include "multibody/kinematic/kinematic_evaluator.h"

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/context.h"

namespace dairlib {
namespace multibody {

/// Basic contact evaluator for the distance between point A in frame A
/// and point B in frame B
///   phi(q) = ||pt_A - pt_B|| - distance
/// Calculations are performed in B, for efficiency.
/// The one exception is Jdotv, since Drake does not currently support
/// MultibodyPlant.CalcBiasSpatialAcceleration with non-world frames,
template <typename T>
class DistanceEvaluator : public KinematicEvaluator<T> {
 public:
  /// Constructor for DistanceEvaluator
  /// @param plant
  /// @param pt_A
  /// @param frame_A
  /// @param pt_B
  /// @param frame_B

  DistanceEvaluator(const drake::multibody::MultibodyPlant<T>& plant,
                    const Eigen::Vector3d pt_A,
                    const drake::multibody::Frame<T>& frame_A,
                    const Eigen::Vector3d pt_B,
                    const drake::multibody::Frame<T>& frame_B, double distance);

  drake::VectorX<T> EvalFull(
      const drake::systems::Context<T>& context) const override;

  void EvalFullJacobian(const drake::systems::Context<T>& context,
                        drake::EigenPtr<drake::MatrixX<T>> J) const override;

  drake::VectorX<T> EvalFullJacobianDotTimesV(
      const drake::systems::Context<T>& context) const override;

  using KinematicEvaluator<T>::EvalFullJacobian;
  using KinematicEvaluator<T>::plant;

 private:
  const Eigen::Vector3d pt_A_;
  const drake::multibody::Frame<T>& frame_A_;
  const Eigen::Vector3d pt_B_;
  const drake::multibody::Frame<T>& frame_B_;
  const double distance_;
};

}  // namespace multibody
}  // namespace dairlib
