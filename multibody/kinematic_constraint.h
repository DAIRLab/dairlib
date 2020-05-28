#pragma once

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/context.h"

namespace dairlib {
namespace multibody {

/// Virtual class to represent arbitrary ik
template <typename T>
class KinematicConstraint {
 public:
  // Disabling copy construction and assignment
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(KinematicConstraint)

  explicit KinematicConstraint(int num_constraints);  

  /// Evaluates the constraint, phi(q), limited only to active rows
  drake::VectorX<T> CalcActiveConstraint(
      const drake::systems::Context<T>& context) const;

  /// Evaluates the time-derivative of the constraint, d/dt phi(q),
  ///  limited only to active rows
  drake::VectorX<T> CalcActiveConstraintDot(
      const drake::systems::Context<T>& context) const;

  /// Evaluates the constraint Jacobian w.r.t. velocity v (not qdot)
  ///  limited only to active rows
  /// TODO (posa): could add an option to compute w.r.t. q
  drake::MatrixX<T> CalcActiveJacobian(
      const drake::systems::Context<T>& context) const;

  /// Evaluates Jdot * v, useful for computing constraint second derivative,
  ///  which would be d^2 phi/dt^2 = J * vdot + Jdot * v
  ///  limited only to active rows
  drake::VectorX<T> CalcActiveJacobianDotTimesV(
      const drake::systems::Context<T>& context) const;

  /// Evaluates the constraint, phi(q), including inactive rows
  virtual drake::VectorX<T> CalcFullConstraint(
      const drake::systems::Context<T>& context) const;

  /// Evaluates the time-derivative of the constraint, d/dt phi(q)
  virtual drake::VectorX<T> CalcFullConstraintDot(
      const drake::systems::Context<T>& context) const;

  /// Evaluates the constraint Jacobian w.r.t. velocity v (not qdot)
  /// TODO (posa): could add an option to compute w.r.t. q
  virtual drake::MatrixX<T> CalcFullJacobian(
      const drake::systems::Context<T>& context) const;

  /// Evaluates Jdot * v, useful for computing constraint second derivative,
  ///  which would be d^2 phi/dt^2 = J * vdot + Jdot * v
  virtual drake::VectorX<T> CalcFullJacobianDotTimesV(
      const drake::systems::Context<T>& context) const;

  void set_active_inds(std::vector<int> active_inds);

  int num_constraints() {
    return num_constraints_;
  }

 private:
  int num_constraints_;
  std::vector<int> active_inds_;
};


/// Basic contact constraint with a flat ground
template <typename T>
class PlanarGroundContactConstraint :  public KinematicConstraint<T> {
 public:
  // Disabling copy construction and assignment
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PlanarGroundContactConstraint)

  PlanarGroundContactConstraint(
      const drake::multibody::MultibodyPlant<T>& plant,
      const Eigen::Vector3d pt_A, const drake::multibody::Frame<T>* frame_A,
      const Eigen::Vector3d normal = (Eigen::Vector3d() << 0, 0, 1).finished(),
      const Eigen::Vector3d offset = Eigen::Vector3d::Zero(),
      bool xy_active = "false");

  drake::VectorX<T> CalcFullConstraint(
      const drake::systems::Context<T>& context) const;

  drake::VectorX<T> CalcFullConstraintDot(
      const drake::systems::Context<T>& context) const;

  drake::MatrixX<T> CalcFullJacobian(
      const drake::systems::Context<T>& context) const;

  drake::VectorX<T> CalcFullJacobianDotTimesV(
      const drake::systems::Context<T>& context) const;

 private:
  const drake::multibody::MultibodyPlant<T>& plant_;
  const Eigen::Vector3d pt_A_;
  const drake::multibody::Frame<T>* frame_A_;
  const Eigen::Vector3d offset_;
  const Eigen::Matrix3d rotation_;
};

}  // namespace multibody
}  // namespace dairlib
