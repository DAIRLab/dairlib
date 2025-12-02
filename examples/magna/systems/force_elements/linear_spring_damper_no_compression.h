#pragma once

#include <memory>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/multibody/tree/force_element.h"

namespace dairlib {
namespace examples {
namespace magna {
namespace systems {
namespace force_elements {

using drake::AutoDiffXd;
using drake::Vector3;
using drake::multibody::ForceElement;
using drake::multibody::MultibodyForces;
using drake::multibody::RigidBody;
using drake::multibody::internal::MultibodyTree;
using drake::multibody::internal::PositionKinematicsCache;
using drake::multibody::internal::VelocityKinematicsCache;
using drake::symbolic::Expression;
using drake::systems::Context;

template <typename T>
class LinearSpringDamperNoCompression final : public ForceElement<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LinearSpringDamperNoCompression);

  /// Constructor for a spring-damper between a point P on `bodyA` and a
  /// point Q on `bodyB`. Point P is defined by its position `p_AP` as
  /// measured and expressed in the body frame A and similarly, point Q is
  /// defined by its position p_BQ as measured and expressed in body frame B.
  /// The remaining parameters define:
  /// @param[in] free_length
  ///   The free length of the spring ℓ₀, in meters, at which the spring
  ///   applies no forces. Since this force element is meant to model finite
  ///   length springs, ℓ₀ must be strictly positive.
  /// @param[in] stiffness
  ///   The stiffness k of the spring in N/m. It must be non-negative.
  /// @param[in] damping
  ///   The damping c of the damper in N⋅s/m. It must be non-negative.
  /// Refer to this class's documentation for further details.
  /// @throws std::exception if `free_length` is negative or zero.
  /// @throws std::exception if `stiffness` is negative.
  /// @throws std::exception if `damping` is negative.
  LinearSpringDamperNoCompression(const RigidBody<T>& bodyA,
                                  const Vector3<double>& p_AP,
                                  const RigidBody<T>& bodyB,
                                  const Vector3<double>& p_BQ,
                                  double free_length, double stiffness,
                                  double damping);

  ~LinearSpringDamperNoCompression() override;

  const RigidBody<T>& bodyA() const { return bodyA_; }

  const RigidBody<T>& bodyB() const { return bodyB_; }

  /// The position p_AP of point P on body A as measured and expressed in body
  /// frame A.
  const Vector3<double> p_AP() const { return p_AP_; }

  /// The position p_BQ of point Q on body B as measured and expressed in body
  /// frame B.
  const Vector3<double> p_BQ() const { return p_BQ_; }

  double free_length() const { return free_length_; }

  double stiffness() const { return stiffness_; }

  double damping() const { return damping_; }

  T CalcPotentialEnergy(const Context<T>& context,
                        const PositionKinematicsCache<T>& pc) const override;

  T CalcConservativePower(const Context<T>& context,
                          const PositionKinematicsCache<T>& pc,
                          const VelocityKinematicsCache<T>& vc) const override;

  T CalcNonConservativePower(
      const Context<T>& context, const PositionKinematicsCache<T>& pc,
      const VelocityKinematicsCache<T>& vc) const override;

 protected:
  void DoCalcAndAddForceContribution(const Context<T>& context,
                                     const PositionKinematicsCache<T>& pc,
                                     const VelocityKinematicsCache<T>& vc,
                                     MultibodyForces<T>* forces) const override;

  std::unique_ptr<ForceElement<double>> DoCloneToScalar(
      const MultibodyTree<double>& tree_clone) const override;

  std::unique_ptr<ForceElement<AutoDiffXd>> DoCloneToScalar(
      const MultibodyTree<AutoDiffXd>& tree_clone) const override;

  std::unique_ptr<ForceElement<Expression>> DoCloneToScalar(
      const MultibodyTree<Expression>&) const override;

  std::unique_ptr<ForceElement<T>> DoShallowClone() const override;

 private:
  // Helper method to make a clone templated on ToScalar.
  template <typename ToScalar>
  std::unique_ptr<ForceElement<ToScalar>> TemplatedDoCloneToScalar(
      const MultibodyTree<ToScalar>& tree_clone) const;

  // To avoid a division by zero when computing a normalized vector from point P
  // on body A to point Q on body B as length of the spring approaches zero,
  // we use a "soft norm" defined by:
  //   ‖x‖ₛ = sqrt(xᵀ⋅x + δ²)
  // where δ = ε⋅ℓ₀ with ε a small dimensionless positive value so that the
  // effect of δ is negligible for non-zero x.
  // This spring model does not allow the length of the spring to approach zero
  // since that would incur in a non-physical situation. Therefore this "safe"
  // norm will throw a std::exception when ‖x‖ < δ.
  T SafeSoftNorm(const Vector3<T>& x) const;

  // Helper method to compute the rate of change of the separation length
  // between the two endpoints for this spring-damper.
  T CalcLengthTimeDerivative(const PositionKinematicsCache<T>& pc,
                             const VelocityKinematicsCache<T>& vc) const;

  const RigidBody<T>& bodyA_;
  const Vector3<double> p_AP_;
  const RigidBody<T>& bodyB_;
  const Vector3<double> p_BQ_;
  double free_length_;
  double stiffness_;
  double damping_;
};

}  // namespace force_elements
}  // namespace systems
}  // namespace magna
}  // namespace examples
}  // namespace dairlib

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::dairlib::examples::magna::systems::force_elements::
        LinearSpringDamperNoCompression);
