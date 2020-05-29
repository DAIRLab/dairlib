#include "multibody/kinematic/kinematic_evaluator_set.h"

namespace dairlib {
namespace multibody {

using drake::MatrixX;
using drake::VectorX;
using drake::systems::Context;

template <typename T>
KinematicEvaluatorSet<T>::KinematicEvaluatorSet() {}

template <typename T>
VectorX<T> KinematicEvaluatorSet<T>::EvalActive(
  const Context<T>& context) const {
  VectorX<T> phi(count_active());
  int ind = 0;
  for (const auto& e : evaluators_) {
    phi.segment(ind, e->num_active()) = e->EvalActive(context);
    ind += e->num_active();
  }
  return phi;
}

template <typename T>
VectorX<T> KinematicEvaluatorSet<T>::EvalActiveTimeDerivative(
  const Context<T>& context) const {
  VectorX<T> phidot(count_active());
  int ind = 0;
  for (const auto& e : evaluators_) {
    phidot.segment(ind, e->num_active()) = e->EvalActiveTimeDerivative(context);
    ind += e->num_active();
  }
  return phidot;
}

template <typename T>
MatrixX<T> KinematicEvaluatorSet<T>::EvalActiveJacobian(
  const Context<T>& context) const {
  const int num_velocities = evaluators_.at(0)->plant().num_velocities();
  MatrixX<T> J(count_active(), num_velocities);
  int ind = 0;
  for (const auto& e : evaluators_) {
    J.block(ind, 0, e->num_active(), num_velocities) =
        e->EvalActiveJacobian(context);
    ind += e->num_active();
  }
  return J;
}

template <typename T>
VectorX<T> KinematicEvaluatorSet<T>::EvalActiveJacobianDotTimesV(
  const Context<T>& context) const {
  VectorX<T> Jdotv(count_active());
  int ind = 0;
  for (const auto& e : evaluators_) {
    Jdotv.segment(ind, e->num_active()) = e->EvalActiveJacobianDotTimesV(context);
    ind += e->num_active();
  }
  return Jdotv;
}

template <typename T>
VectorX<T> KinematicEvaluatorSet<T>::EvalFull(
  const Context<T>& context) const {
  VectorX<T> phi(count_active());
  int ind = 0;
  for (const auto& e : evaluators_) {
    phi.segment(ind, e->num_full()) = e->EvalFull(context);
    ind += e->num_full();
  }
  return phi;
}

template <typename T>
VectorX<T> KinematicEvaluatorSet<T>::EvalFullTimeDerivative(
  const Context<T>& context) const {
  VectorX<T> phidot(count_active());
  int ind = 0;
  for (const auto& e : evaluators_) {
    phidot.segment(ind, e->num_full()) = e->EvalFullTimeDerivative(context);
    ind += e->num_full();
  }
  return phidot;
}

template <typename T>
MatrixX<T> KinematicEvaluatorSet<T>::EvalFullJacobian(
  const Context<T>& context) const {
  const int num_velocities = evaluators_.at(0)->plant().num_velocities();
  MatrixX<T> J(count_active(), num_velocities);
  int ind = 0;
  for (const auto& e : evaluators_) {
    J.block(ind, 0, e->num_full(), num_velocities) =
        e->EvalFullJacobian(context);
    ind += e->num_full();
  }
  return J;
}

template <typename T>
VectorX<T> KinematicEvaluatorSet<T>::EvalFullJacobianDotTimesV(
  const Context<T>& context) const {
  VectorX<T> Jdotv(count_active());
  int ind = 0;
  for (const auto& e : evaluators_) {
    Jdotv.segment(ind, e->num_full()) = e->EvalFullJacobianDotTimesV(context);
    ind += e->num_full();
  }
  return Jdotv;
}

template <typename T>
int KinematicEvaluatorSet<T>::add_evaluator(KinematicEvaluator<T>* e) {
  // TODO Confirm that all evaluators share the same plant
  
  evaluators_.push_back(e);
  return evaluators_.size() - 1;
}

template <typename T>
std::vector<int> KinematicEvaluatorSet<T>::FindUnion(
    KinematicEvaluatorSet<T> other) {
  std::vector<int> union_index;
  for (int i = 0; i < other.num_evaluators(); i++) {
    if (std::find(evaluators_.begin(), evaluators_.end(),
        other.get_evaluator(i)) != evaluators_.end()) {
      union_index.push_back(i);
    }
  }
  return union_index;
}

template <typename T>
int KinematicEvaluatorSet<T>::evaluator_full_start(int index) const {
  int start = 0;
  for (int i = 0; i < index; i++) {
    start += evaluators_.at(i)->num_full();
  }
  return start;
}

template <typename T>
int KinematicEvaluatorSet<T>::evaluator_active_start(int index) const {
  int start = 0;
  for (int i = 0; i < index; i++) {
    start += evaluators_.at(i)->num_active();
  }
  return start;
}

template <typename T>
int KinematicEvaluatorSet<T>::count_active() const {
  int count = 0;
  for (const auto& e : evaluators_) {
    count += e->num_active();
  }
  return count;
}

template <typename T>
int KinematicEvaluatorSet<T>::count_full() const {
  int count = 0;
  for (const auto& e : evaluators_) {
    count += e->num_full();
  }
  return count;
}


DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::multibody::KinematicEvaluatorSet)


}  // namespace multibody
}  // namespace dairlib
