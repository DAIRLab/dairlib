#include "multibody/kinematic/distance_evaluator.h"

using drake::MatrixX;
using drake::Matrix3X;
using drake::VectorX;
using drake::Vector3;
using drake::multibody::Frame;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using Eigen::Vector3d;

namespace dairlib {
namespace multibody {

template <typename T>
DistanceEvaluator<T>::DistanceEvaluator(const MultibodyPlant<T>& plant,
                                        const Vector3d pt_A,
                                        const Frame<T>& frame_A,
                                        const Vector3d pt_B,
                                        const Frame<T>& frame_B,
                                        double distance)
    : KinematicEvaluator<T>(plant, 1),
      pt_A_(pt_A),
      frame_A_(frame_A),
      pt_B_(pt_B),
      frame_B_(frame_B),
      distance_(distance) {}

template <typename T>
VectorX<T> DistanceEvaluator<T>::EvalFull(const Context<T>& context) const {
  // Transform points A and B to world frame
  const drake::multibody::Frame<T>& world = plant().world_frame();
  static Vector3<T> pt_A_W;
  static Vector3<T> pt_B_W;

  plant().CalcPointsPositions(context, frame_A_, pt_A_.template cast<T>(),
                              world, &pt_A_W);
  plant().CalcPointsPositions(context, frame_B_, pt_B_.template cast<T>(),
                              world, &pt_B_W);
  auto rel_pos = pt_A_W - pt_B_W;
  VectorX<T> difference(1);
  difference << rel_pos.norm() - distance_;
  return difference;
}

template <typename T>
void DistanceEvaluator<T>::EvalFullJacobian(
    const Context<T>& context, drake::EigenPtr<MatrixX<T>> J) const {
  /// Jacobian of ||pt_A - pt_B||, evaluated all in world frame, is
  ///   (pt_A - pt_B)^T * (J_A - J_B) / ||pt_A - pt_B||

  // Create static Jacobians and point positions for re-use. Warning: not thread
  // safe
  static Matrix3X<T> J_A(3, plant().num_velocities());
  static Matrix3X<T> J_B(3, plant().num_velocities());
  static Vector3<T> pt_A_W;;
  static Vector3<T> pt_B_W;

  const drake::multibody::Frame<T>& world = plant().world_frame();

  plant().CalcPointsPositions(context, frame_A_, pt_A_.template cast<T>(),
                              world, &pt_A_W);
  plant().CalcPointsPositions(context, frame_B_, pt_B_.template cast<T>(),
                              world, &pt_B_W);
  auto rel_pos = pt_A_W - pt_B_W;

  // .template cast<T> converts pt_A_, as a double, into type T
  plant().CalcJacobianTranslationalVelocity(
      context, drake::multibody::JacobianWrtVariable::kV, frame_A_,
      pt_A_.template cast<T>(), world, world, &J_A);
  plant().CalcJacobianTranslationalVelocity(
      context, drake::multibody::JacobianWrtVariable::kV, frame_B_,
      pt_B_.template cast<T>(), world, world, &J_B);
  *J = (rel_pos.transpose() * (J_A - J_B)) / rel_pos.norm();
}

template <typename T>
VectorX<T> DistanceEvaluator<T>::EvalFullJacobianDotTimesV(
    const Context<T>& context) const {
  // From applying the chain rule to Jacobian, Jdot * v is
  //
  // ||(J_A - J_B) * v||^2/phi ...
  //   + (pt_A - pt_B)^T * (J_A_dot * v  -J_B_dot * v) / phi ...
  //   - phidot * (pt_A - pt_B)^T (J_A - J_B) *v / phi^2
  const drake::multibody::Frame<T>& world = plant().world_frame();

  static MatrixX<T> J_A(3, plant().num_velocities());
  static MatrixX<T> J_B(3, plant().num_velocities());
  static VectorX<T> pt_A_world(3);
  static VectorX<T> pt_B_world(3);

  auto pt_A_cast = pt_A_.template cast<T>();
  auto pt_B_cast = pt_B_.template cast<T>();

  // Perform all kinematic calculations, finding A, B in world frame,
  // Jacobians J_A and J_B, and Jdotv for both A and B
  plant().CalcPointsPositions(context, frame_A_, pt_A_cast, world, &pt_A_world);
  plant().CalcPointsPositions(context, frame_B_, pt_B_cast, world, &pt_B_world);
  auto rel_pos = pt_A_world - pt_B_world;

  plant().CalcJacobianTranslationalVelocity(
      context, drake::multibody::JacobianWrtVariable::kV, frame_A_, pt_A_cast,
      world, world, &J_A);
  plant().CalcJacobianTranslationalVelocity(
      context, drake::multibody::JacobianWrtVariable::kV, frame_B_, pt_B_cast,
      world, world, &J_B);
  auto J_rel = J_A - J_B;

  VectorX<T> J_A_dot_times_v = plant().CalcBiasTranslationalAcceleration(
      context, drake::multibody::JacobianWrtVariable::kV, frame_A_, pt_A_cast,
      world, world);
  VectorX<T> J_B_dot_times_v = plant().CalcBiasTranslationalAcceleration(
      context, drake::multibody::JacobianWrtVariable::kV, frame_B_, pt_B_cast,
      world, world);

  auto J_rel_dot_times_v = J_A_dot_times_v - J_B_dot_times_v;
  T phi = rel_pos.norm();

  auto J = (rel_pos.transpose() * J_rel) / phi;

  T phidot = J.dot(plant().GetVelocities(context));

  // Compute (J_A - J_B) * v, as this is used multiple times
  VectorX<T> J_rel_v(3);
  J_rel_v << J_rel * plant().GetVelocities(context);

  // Compute all terms as scalars using dot products
  VectorX<T> J_dot_times_v(1);
  J_dot_times_v << (J_rel_v).squaredNorm() / phi +
                       rel_pos.dot(J_rel_dot_times_v) / phi -
                       phidot * rel_pos.dot(J_rel_v) / (phi * phi);
  return J_dot_times_v;
}

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::multibody::DistanceEvaluator)

}  // namespace multibody
}  // namespace dairlib
