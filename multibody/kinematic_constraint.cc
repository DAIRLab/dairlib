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
  return CalcFullConstraint(context)(active_inds_);
}

template <typename T>
VectorX<T> KinematicConstraint<T>::CalcActiveConstraintDot(
    const Context<T>& context) const {
  return CalcFullConstraintDot(context)(active_inds_);
}

template <typename T>
MatrixX<T> KinematicConstraint<T>::CalcActiveJacobian(
      const Context<T>& context) const {
  auto J_full = CalcFullJacobian(context);

  // Build a vector 0:J.cols()
  std::vector<int> all_cols(J_full.cols());
  std::iota(std::begin(all_cols), std::end(all_cols), 0);

  // Extract active rows only
  return J_full(active_inds_, all_cols);
}

template <typename T>
VectorX<T> KinematicConstraint<T>::CalcActiveJacobianDotTimesV(
      const Context<T>& context) const {
  return CalcFullJacobianDotTimesV(context)(active_inds_);
}

template <typename T>
void KinematicConstraint<T>::set_active_inds(std::vector<int> active_inds) {
  active_inds_ = active_inds;
}

/// 
/// Specific implementations for PlanarGroundContactConstraint
///
template <typename T>
PlanarGroundContactConstraint<T>::PlanarGroundContactConstraint(
    const MultibodyPlant<T>& plant, const Vector3d pt_A,
    const Frame<T>* frame_A, const Vector3d offset, const Vector3d normal,
    bool xy_active)
    : KinematicConstraint<T>(3),
      plant_(plant),
      pt_A_(pt_A),
      frame_A_(frame_A),
      offset_(offset),
      rotation_(drake::math::ComputeBasisFromAxis(2, normal)) {
  if (!xy_active) {
    this->set_active_inds({2});  // only z is active
  }
}

template <typename T>
VectorX<T> PlanarGroundContactConstraint<T>::CalcFullConstraint(
    const Context<T>& context) const {
  VectorX<T> pt_world(3);
  const drake::multibody::Frame<T>& world = plant_.world_frame();

  plant_.CalcPointsPositions(context, *frame_A_,
      pt_A_.template cast<T>(), world, &pt_world);   

  return rotation_ * (pt_world - offset_);
}

template <typename T>
VectorX<T> PlanarGroundContactConstraint<T>::CalcFullConstraintDot(
    const Context<T>& context) const {
  auto J = CalcFullJacobian(context);
  return J * plant_.GetVelocities(context);
}

template <typename T>
MatrixX<T> PlanarGroundContactConstraint<T>::CalcFullJacobian(
    const Context<T>& context) const {
  MatrixX<T> J(3, plant_.num_velocities());

  const drake::multibody::Frame<T>& world = plant_.world_frame();

  // .template cast<T> converts pt_A_, as a double, into type T
  plant_.CalcJacobianTranslationalVelocity(
    context, drake::multibody::JacobianWrtVariable::kV,
    *frame_A_, pt_A_.template cast<T>(), world, world, &J);

  return rotation_ * J;
}

template <typename T>
VectorX<T> PlanarGroundContactConstraint<T>::CalcFullJacobianDotTimesV(
    const Context<T>& context) const {
  const drake::multibody::Frame<T>& world = plant_.world_frame();

  MatrixX<T> Jdot_times_V = plant_.CalcBiasSpatialAcceleration(
      context, drake::multibody::JacobianWrtVariable::kV, *frame_A_,
      pt_A_.template cast<T>(), world, world).translational();

  return rotation_ * Jdot_times_V;
}

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::dairlib::multibody::PlanarGroundContactConstraint)

}  // namespace multibody
}  // namespace dairlib
