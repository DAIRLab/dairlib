#include "multibody/kinematic/kinematic_evaluator_set.h"
#include "drake/math/autodiff_gradient.h"

namespace dairlib {
namespace multibody {

using drake::MatrixX;
using drake::VectorX;
using drake::systems::Context;

template <typename T>
KinematicEvaluatorSet<T>::KinematicEvaluatorSet(
    const drake::multibody::MultibodyPlant<T>& plant) :
    plant_(plant) {}

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
  const int num_velocities = plant_.num_velocities();
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
  VectorX<T> phi(count_full());
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
  VectorX<T> phidot(count_full());
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
  const int num_velocities = plant_.num_velocities();
  MatrixX<T> J(count_full(), num_velocities);
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
  VectorX<T> Jdotv(count_full());
  int ind = 0;
  for (const auto& e : evaluators_) {
    Jdotv.segment(ind, e->num_full()) = e->EvalFullJacobianDotTimesV(context);
    ind += e->num_full();
  }
  return Jdotv;
}

template <typename T>
int KinematicEvaluatorSet<T>::add_evaluator(KinematicEvaluator<T>* e) {
  // Compare plants for equality by reference
  DRAKE_DEMAND(&plant_ == &e->plant());
  
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
VectorX<T> KinematicEvaluatorSet<T>::CalcMassMatrixTimesVDot(
    const Context<T>& context, const VectorX<T>& lambda) const {
  // M(q)vdot + C(q,v) = tau_g(q) + F_app + Bu + J(q)^T lambda
  VectorX<T> C(plant_.num_velocities());
  plant_.CalcBiasTerm(context, &C);

  VectorX<T> Bu = plant_.MakeActuationMatrix()
      * plant_.get_actuation_input_port().Eval(context);

  VectorX<T> tau_g = plant_.CalcGravityGeneralizedForces(context);

  drake::multibody::MultibodyForces<T> f_app(plant_);
  plant_.CalcForceElementsContribution(context, &f_app);

  VectorX<T> J_transpose_lambda =
      EvalFullJacobian(context).transpose() * lambda;

  return tau_g + f_app.generalized_forces()+ Bu + J_transpose_lambda - C;
}

template <typename T>
VectorX<T> KinematicEvaluatorSet<T>::CalcTimeDerivatives(
    const Context<T>& context, const VectorX<T>& lambda) const {
  MatrixX<T> M(plant_.num_velocities(), plant_.num_velocities());
  plant_.CalcMassMatrix(context, &M);

  VectorX<T> right_hand_side = CalcMassMatrixTimesVDot(context, lambda);

  VectorX<T> v_dot = M.llt().solve(right_hand_side);

  VectorX<T> x_dot(plant_.num_positions() + plant_.num_velocities());
  VectorX<T> q_dot(plant_.num_positions());

  plant_.MapVelocityToQDot(context, plant_.GetVelocities(context), &q_dot);

  x_dot << q_dot, v_dot;

  return x_dot;
}

template <typename T>
VectorX<T> KinematicEvaluatorSet<T>::CalcTimeDerivatives(
    const Context<T>& context, double alpha) const {
  VectorX<T> lambda;
  return CalcTimeDerivatives(context, &lambda, alpha);
}

template <typename T>
VectorX<T> KinematicEvaluatorSet<T>::CalcTimeDerivatives(
    const Context<T>& context, VectorX<T>* lambda, double alpha) const {
  // M(q) vdot + C(q,v) = tau_g(q) + Bu + J(q)^T lambda
  // J vdot + Jdotv  + kp phi + kd phidot = 0
  // Produces linear system of equations
  // [[M -J^T]  [[vdot  ]  =  [[tau_g + Bu - C     ]
  //  [J  0 ]]  [lambda]]     [ -Jdotv - kp phi - kd phidot]]

  // Evaluate manipulator equation terms
  MatrixX<T> M(plant_.num_velocities(), plant_.num_velocities());
  plant_.CalcMassMatrix(context, &M);

  VectorX<T> C(plant_.num_velocities());
  plant_.CalcBiasTerm(context, &C);

  VectorX<T> Bu = plant_.MakeActuationMatrix()
      * plant_.get_actuation_input_port().Eval(context);

  VectorX<T> tau_g = plant_.CalcGravityGeneralizedForces(context);

  drake::multibody::MultibodyForces<T> f_app(plant_);
  plant_.CalcForceElementsContribution(context, &f_app);

  // Evaluate constraint terms, phi/phidot for active. Jacboians for full
  VectorX<T> phi = EvalActive(context);
  VectorX<T> phidot = EvalActiveTimeDerivative(context);
  MatrixX<T> J = EvalFullJacobian(context);
  VectorX<T> Jdotv = EvalFullJacobianDotTimesV(context);


  MatrixX<T> A(M.rows() + J.rows(),
               M.cols() + J.rows());
  VectorX<T> b(M.rows() + J.rows());
  A << M, -J.transpose(),
       J, MatrixX<T>::Zero(J.rows(), J.rows());
  b << tau_g + f_app.generalized_forces() + Bu - C,
       -(Jdotv + alpha * alpha * phi + 2 * alpha * phidot);
  const VectorX<T> vdot_lambda = A.ldlt().solve(b);
  
  *lambda = vdot_lambda.tail(J.rows());

  VectorX<T> x_dot(plant_.num_positions() + plant_.num_velocities());
  VectorX<T> q_dot(plant_.num_positions());

  plant_.MapVelocityToQDot(context, plant_.GetVelocities(context), &q_dot);

  x_dot << q_dot, vdot_lambda.head(plant_.num_velocities());

  return x_dot;
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

template <typename T>
bool KinematicEvaluatorSet<T>::is_active(int index) {
  if (index < 0 || index >= count_full()) {
    return false;
  }
  int count = 0;
  for (const auto& e : evaluators_) {
    if (index < count + e->num_full()) {
      bool in_active_set =
          std::find(e->active_inds().begin(), e->active_inds().end(),index)
          != e->active_inds().end();
      return in_active_set
    } else {
      count += e->num_full();
    }
  }
  DRAKE_UNREACHABLE();
}

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::multibody::KinematicEvaluatorSet)


}  // namespace multibody
}  // namespace dairlib
