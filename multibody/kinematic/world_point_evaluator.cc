#include "multibody/kinematic/world_point_evaluator.h"

#include "drake/math/orthonormal_basis.h"

using drake::multibody::Frame;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using drake::MatrixX;
using drake::VectorX;
using Eigen::Vector3d;
using Eigen::Matrix3d;

namespace dairlib {
namespace multibody {

template <typename T>
WorldPointEvaluator<T>::WorldPointEvaluator(
    const MultibodyPlant<T>& plant, Vector3d pt_A,
    const Frame<T>& frame_A,
    const Matrix3d rotation,
    const Vector3d offset,
    std::vector<int> active_directions_)
    : KinematicEvaluator<T>(plant, 3),
      pt_A_(pt_A),
      frame_A_(frame_A),
      offset_(offset),
      rotation_(rotation) {
    this->set_active_inds(active_directions_);
}

template <typename T>
WorldPointEvaluator<T>::WorldPointEvaluator(
    const MultibodyPlant<T>& plant, const Vector3d pt_A,
    const Frame<T>& frame_A, const Vector3d normal, const Vector3d offset,
    bool tangent_active)
    : KinematicEvaluator<T>(plant, 3),
      pt_A_(pt_A),
      frame_A_(frame_A),
      offset_(offset),
      rotation_(drake::math::ComputeBasisFromAxis(2, normal)) {
  if (!tangent_active) {
    this->set_active_inds({2});  // only z is active
  }
}

template <typename T>
VectorX<T> WorldPointEvaluator<T>::EvalFull(
    const Context<T>& context) const {
  VectorX<T> pt_world(3);
  const drake::multibody::Frame<T>& world = plant().world_frame();

  plant().CalcPointsPositions(context, frame_A_,
      pt_A_.template cast<T>(), world, &pt_world);   

  return rotation_ * (pt_world - offset_);
}

template <typename T>
MatrixX<T> WorldPointEvaluator<T>::EvalFullJacobian(
    const Context<T>& context) const {
  MatrixX<T> J(3, plant().num_velocities());

  const drake::multibody::Frame<T>& world = plant().world_frame();

  // .template cast<T> converts pt_A_, as a double, into type T
  plant().CalcJacobianTranslationalVelocity(
    context, drake::multibody::JacobianWrtVariable::kV,
    frame_A_, pt_A_.template cast<T>(), world, world, &J);

  return rotation_ * J;
}

template <typename T>
VectorX<T> WorldPointEvaluator<T>::EvalFullJacobianDotTimesV(
    const Context<T>& context) const {
  const drake::multibody::Frame<T>& world = plant().world_frame();

  MatrixX<T> Jdot_times_V = plant().CalcBiasSpatialAcceleration(
      context, drake::multibody::JacobianWrtVariable::kV, frame_A_,
      pt_A_.template cast<T>(), world, world).translational();

  return rotation_ * Jdot_times_V;
}

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::multibody::WorldPointEvaluator)

}  // namespace multibody
}  // namespace dairlib