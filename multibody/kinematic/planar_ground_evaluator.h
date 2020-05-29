#pragma once
#include "multibody/kinematic/kinematic_evaluator.h"

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/context.h"

namespace dairlib {
namespace multibody {

/// Basic contact evaluator with a flat ground
template <typename T>
class PlanarGroundEvaluator :  public KinematicEvaluator<T> {
 public:
  /// Constructor for PlanarGroundEvaluator
  /// @param plant
  /// @param pt_A the contact point on the body
  /// @param frame_A the frame of reference for pt_A
  /// @param normal The normal vector for the contact (default [0;0;1])
  /// @param offset The nominal world location of pt_A (default [0;0;0])
  /// @param xy_active If true, then the tangential directions are active
  ///    If false, they be inactive, and thus still appear in
  ///    the "Full" terms, but not the "Active" values

  PlanarGroundEvaluator(
      const drake::multibody::MultibodyPlant<T>& plant,
      const Eigen::Vector3d pt_A, const drake::multibody::Frame<T>& frame_A,
      const Eigen::Vector3d normal = Eigen::Vector3d({0, 0, 1}), //(Eigen::Vector3d() << 0, 0, 1).finished(),
      const Eigen::Vector3d offset = Eigen::Vector3d::Zero(),
      bool tangent_active = false);

  drake::VectorX<T> EvalFull(
      const drake::systems::Context<T>& context) const;

  drake::MatrixX<T> EvalFullJacobian(
      const drake::systems::Context<T>& context) const;

  drake::VectorX<T> EvalFullJacobianDotTimesV(
      const drake::systems::Context<T>& context) const;

  using KinematicEvaluator<T>::plant;

 private:
  const Eigen::Vector3d pt_A_;
  const drake::multibody::Frame<T>& frame_A_;
  const Eigen::Vector3d offset_;
  const Eigen::Matrix3d rotation_;
};

}  // namespace multibody
}  // namespace dairlib
