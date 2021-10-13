#include "multibody/kinematic/kinematic_evaluator.h"

using drake::MatrixX;
using drake::VectorX;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;

namespace dairlib {
namespace multibody {

template <typename T>
KinematicEvaluator<T>::KinematicEvaluator(const MultibodyPlant<T>& plant,
                                          int length)
    : plant_(plant), length_(length) {
  std::vector<int> all_inds;
  all_active_default_order_ = true;
  for (int i = 0; i < length; i++) {
    all_inds.push_back(i);
  }
  set_active_inds(all_inds);
}

template <typename T>
MatrixX<T> KinematicEvaluator<T>::EvalFullJacobian(
    const drake::systems::Context<T>& context) const {
  drake::MatrixX<T> J(length_, plant_.num_velocities());
  EvalFullJacobian(context, &J);
  return J;
}

template <typename T>
VectorX<T> KinematicEvaluator<T>::EvalActive(const Context<T>& context) const {
  // TODO: With Eigen 3.4, can slice by (active_inds_);

  auto phi_full = EvalFull(context);
  if (all_active_default_order_) {
    return phi_full;
  }

  VectorX<T> phi(num_active_);
  for (int i = 0; i < num_active_; i++) {
    phi(i) = phi_full(active_inds_.at(i));
  }
  return phi;
}

template <typename T>
VectorX<T> KinematicEvaluator<T>::EvalActiveTimeDerivative(
    const Context<T>& context) const {
  // TODO: With Eigen 3.4, can slice by (active_inds_);
  auto phidot_full = EvalFullTimeDerivative(context);
  if (all_active_default_order_) {
    return phidot_full;
  }

  VectorX<T> phidot(num_active_);
  for (int i = 0; i < num_active_; i++) {
    phidot(i) = phidot_full(active_inds_.at(i));
  }
  return phidot;
}

template <typename T>
MatrixX<T> KinematicEvaluator<T>::EvalActiveJacobian(
    const Context<T>& context) const {
  // TODO: With Eigen 3.4, can slice by (active_inds_, all);
  auto J_full = EvalFullJacobian(context);
  if (all_active_default_order_) {
    return J_full;
  }

  MatrixX<T> J_active(num_active_, J_full.cols());
  for (int i = 0; i < num_active_; i++) {
    J_active.row(i) = J_full.row(active_inds_.at(i));
  }

  // Extract active rows only
  return J_active;
}

template <typename T>
VectorX<T> KinematicEvaluator<T>::EvalActiveJacobianDotTimesV(
    const Context<T>& context) const {
  // TODO: With Eigen 3.4, can slice by (active_inds_);
  auto Jdot_v_full = EvalFullJacobianDotTimesV(context);
  if (all_active_default_order_) {
    return Jdot_v_full;
  }

  VectorX<T> Jdot_v(num_active_);
  for (int i = 0; i < num_active_; i++) {
    Jdot_v(i) = Jdot_v_full(active_inds_.at(i));
  }
  return Jdot_v;
}

template <typename T>
VectorX<T> KinematicEvaluator<T>::EvalFullTimeDerivative(
    const Context<T>& context) const {
  return EvalFullJacobian(context) * plant_.GetVelocities(context);
}

template <typename T>
void KinematicEvaluator<T>::set_active_inds(std::vector<int> active_inds) {
  // Check active_inds [0, length()] bounds
  for (auto& i : active_inds) {
    DRAKE_DEMAND(i >= 0);
    DRAKE_DEMAND(i < num_full());
  }
  // Create a set to check for uniqueness
  std::set<int> s(active_inds.begin(), active_inds.end());
  DRAKE_DEMAND(s.size() == active_inds.size());

  active_inds_ = active_inds;
  num_active_ = active_inds_.size();

  // Cannot just check length, because active_inds could be out of order
  all_active_default_order_ = true;
  if (num_active_ == length_) {
    for (int i = 0; i < num_active_; i++) {
      if (active_inds_.at(i) != i) {
        all_active_default_order_ = false;
      }
    }
  } else {
    all_active_default_order_ = false;
  }
}

template <typename T>
bool KinematicEvaluator<T>::is_active(int index) const {
  return std::find(active_inds_.begin(), active_inds_.end(), index) !=
         active_inds_.end();
}

template <typename T>
int KinematicEvaluator<T>::full_index_to_active_index(int full_index) const {
  for (size_t i = 0; i < active_inds_.size(); i++) {
    if (active_inds_.at(i) == full_index) {
      return i;
    }
  }
  return -1;
}

template <typename T>
const std::vector<int>& KinematicEvaluator<T>::active_inds() const {
  return active_inds_;
};

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::multibody::KinematicEvaluator)

}  // namespace multibody
}  // namespace dairlib
