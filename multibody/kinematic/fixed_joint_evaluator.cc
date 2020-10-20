#include "multibody/kinematic/fixed_joint_evaluator.h"

#include "drake/math/orthonormal_basis.h"

using drake::Matrix3X;
using drake::MatrixX;
using drake::Vector3;
using drake::VectorX;
using drake::multibody::Frame;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using Eigen::Vector3d;

namespace dairlib {
namespace multibody {

template <typename T>
FixedJointEvaluator<T>::FixedJointEvaluator(const MultibodyPlant<T>& plant,
                                            int pos_idx, int vel_idx,
                                            double pos_value)
    : KinematicEvaluator<T>(plant, 1),
      pos_idx_(pos_idx),
      pos_value_(pos_value) {
  J_ = MatrixX<T>::Zero(1, plant.num_velocities());
  J_(0, vel_idx) = 1;
}

template <typename T>
VectorX<T> FixedJointEvaluator<T>::EvalFull(const Context<T>& context) const {
  // Transform points A and B to world frame
  VectorX<T> difference(1);
  difference << plant().GetPositions(context)(pos_idx_) - pos_value_;
  return difference;
}

template <typename T>
void FixedJointEvaluator<T>::EvalFullJacobian(
    const Context<T>& context, drake::EigenPtr<MatrixX<T>> J) const {
  *J = J_;
}

template <typename T>
VectorX<T> FixedJointEvaluator<T>::EvalFullJacobianDotTimesV(
    const Context<T>& context) const {
  return VectorX<T>::Zero(1);
}

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::multibody::FixedJointEvaluator)

}  // namespace multibody
}  // namespace dairlib
