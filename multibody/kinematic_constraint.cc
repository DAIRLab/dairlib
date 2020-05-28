#include "multibody/kinematic_constraint.h"

#include "drake/math/orthonormal_basis.h"

using drake::multibody::Frame;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using drake::MatrixX;
using drake::VectorX;
using Eigen::Vector3d;

namespace dairlib {
namespace multibody {

///
/// Basic implementations for KinematicConstraint
///
template <typename T>
KinematicConstraint<T>::KinematicConstraint(int num_constraints) : 
    num_constraints_(num_constraints) {
  std::vector<int> all_inds;
  for (int i = 0; i < num_constraints; i++) {
    all_inds.push_back(i);
  }
  set_active_inds(all_inds);
}

template <typename T>
VectorX<T> KinematicConstraint<T>::CalcActiveConstraint(
    const Context<T>& context) const {
  // TODO: With Eigen 3.4, can slice by (active_inds_);

  auto constraint_full = CalcFullConstraint(context);

  VectorX<T> constraint(num_active_);
  for (int i = 0; i < num_active_; i++) {
    constraint(i) = constraint_full(active_inds_.at(i));
  }
  return constraint;
}

template <typename T>
VectorX<T> KinematicConstraint<T>::CalcActiveConstraintDot(
    const Context<T>& context) const {
  // TODO: With Eigen 3.4, can slice by (active_inds_);
  auto constraint_dot_full = CalcFullConstraintDot(context);
  VectorX<T> constraint_dot(num_active_);
  for (int i = 0; i < num_active_; i++) {
    constraint_dot(i) = constraint_dot_full(active_inds_.at(i));
  }
  return constraint_dot;
}

template <typename T>
MatrixX<T> KinematicConstraint<T>::CalcActiveJacobian(
      const Context<T>& context) const {
  // TODO: With Eigen 3.4, can slice by (active_inds_, all);
  auto J_full = CalcFullJacobian(context);

  MatrixX<T> J_active(num_active_, J_full.cols());
  for (int i = 0; i < num_active_; i++) {
    J_active.row(i) = J_full.row(active_inds_.at(i));
  }

  // Extract active rows only
  return J_active;
}

template <typename T>
VectorX<T> KinematicConstraint<T>::CalcActiveJacobianDotTimesV(
      const Context<T>& context) const {
  // TODO: With Eigen 3.4, can slice by (active_inds_);
  auto Jdot_v_full = CalcFullJacobianDotTimesV(context);
  VectorX<T> Jdot_v(num_active_);
  for (int i = 0; i < num_active_; i++) {
    Jdot_v(i) = Jdot_v_full(active_inds_.at(i));
  }
  return Jdot_v;
}

template <typename T>
void KinematicConstraint<T>::set_active_inds(std::vector<int> active_inds) {
  active_inds_ = active_inds;
  num_active_ = active_inds_.size();
}

/// 
/// Specific implementations for PlanarGroundContactConstraint
///
template <typename T>
PlanarGroundContactConstraint<T>::PlanarGroundContactConstraint(
    const MultibodyPlant<T>& plant, const Vector3d pt_A,
    const Frame<T>& frame_A, const Vector3d normal, const Vector3d offset,
    bool tangent_active)
    : KinematicConstraint<T>(3),
      plant_(plant),
      pt_A_(pt_A),
      frame_A_(frame_A),
      offset_(offset),
      rotation_(drake::math::ComputeBasisFromAxis(2, normal)) {
  if (!tangent_active) {
    this->set_active_inds({2});  // only z is active
  }
}

template <typename T>
VectorX<T> PlanarGroundContactConstraint<T>::CalcFullConstraint(
    const Context<T>& context) const {
  VectorX<T> pt_world(3);
  const drake::multibody::Frame<T>& world = plant_.world_frame();

  plant_.CalcPointsPositions(context, frame_A_,
      pt_A_.template cast<T>(), world, &pt_world);   

  return rotation_ * (pt_world - offset_);
}

template <typename T>
VectorX<T> PlanarGroundContactConstraint<T>::CalcFullConstraintDot(
    const Context<T>& context) const {
  return CalcFullJacobian(context) * plant_.GetVelocities(context);
}

template <typename T>
MatrixX<T> PlanarGroundContactConstraint<T>::CalcFullJacobian(
    const Context<T>& context) const {
  MatrixX<T> J(3, plant_.num_velocities());

  const drake::multibody::Frame<T>& world = plant_.world_frame();

  // .template cast<T> converts pt_A_, as a double, into type T
  plant_.CalcJacobianTranslationalVelocity(
    context, drake::multibody::JacobianWrtVariable::kV,
    frame_A_, pt_A_.template cast<T>(), world, world, &J);

  return rotation_ * J;
}

template <typename T>
VectorX<T> PlanarGroundContactConstraint<T>::CalcFullJacobianDotTimesV(
    const Context<T>& context) const {
  const drake::multibody::Frame<T>& world = plant_.world_frame();

  MatrixX<T> Jdot_times_V = plant_.CalcBiasSpatialAcceleration(
      context, drake::multibody::JacobianWrtVariable::kV, frame_A_,
      pt_A_.template cast<T>(), world, world).translational();

  return rotation_ * Jdot_times_V;
}

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::multibody::KinematicConstraint)
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::multibody::PlanarGroundContactConstraint)

}  // namespace multibody
}  // namespace dairlib