#include "multibody/kinematic/kinematic_evaluator_set.h"

#include "drake/math/autodiff_gradient.h"

namespace dairlib {
namespace multibody {

using drake::MatrixX;
using drake::VectorX;
using drake::systems::Context;

template <typename T>
KinematicEvaluatorSet<T>::KinematicEvaluatorSet(
    const drake::multibody::MultibodyPlant<T>& plant)
    : plant_(plant) {
  if (plant.geometry_source_is_registered()) {
    drake::log()->warn(
        "Plant in KinematicEvaluatorSet has an associated SceneGraph. This may "
        "introduce undesired contact forces when using "
        "EvalActiveSecondTimeDerivative, EvalFullSecondTimeDerivative, and "
        "CalcTimeDerivativesWithForWithForce.");
  }
}

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
    Jdotv.segment(ind, e->num_active()) =
        e->EvalActiveJacobianDotTimesV(context);
    ind += e->num_active();
  }
  return Jdotv;
}

template <typename T>
VectorX<T> KinematicEvaluatorSet<T>::EvalFull(const Context<T>& context) const {
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
VectorX<T> KinematicEvaluatorSet<T>::EvalFullSecondTimeDerivative(
    Context<T>* context, const VectorX<T>& lambda) const {
  const auto& xdot = CalcTimeDerivativesWithForce(context, lambda);
  const auto& J = EvalFullJacobian(*context);
  const auto& Jdotv = EvalFullJacobianDotTimesV(*context);
  return J * xdot.tail(plant_.num_velocities()) + Jdotv;
}

template <typename T>
VectorX<T> KinematicEvaluatorSet<T>::EvalActiveSecondTimeDerivative(
    Context<T>* context, const VectorX<T>& lambda) const {
  const auto& xdot = CalcTimeDerivativesWithForce(context, lambda);
  const auto& J = EvalActiveJacobian(*context);
  const auto& Jdotv = EvalActiveJacobianDotTimesV(*context);
  return J * xdot.tail(plant_.num_velocities()) + Jdotv;
}

template <typename T>
void KinematicEvaluatorSet<T>::EvalFullJacobian(
    const Context<T>& context, drake::EigenPtr<MatrixX<T>> J) const {
  const int num_velocities = plant_.num_velocities();
  DRAKE_THROW_UNLESS(J->rows() == count_full());
  DRAKE_THROW_UNLESS(J->cols() == num_velocities);
  int ind = 0;
  for (const auto& e : evaluators_) {
    auto J_i = J->block(ind, 0, e->num_full(), num_velocities);
    e->EvalFullJacobian(context, &J_i);
    ind += e->num_full();
  }
}

template <typename T>
MatrixX<T> KinematicEvaluatorSet<T>::EvalFullJacobian(
    const drake::systems::Context<T>& context) const {
  MatrixX<T> J(count_full(), plant_.num_velocities());
  EvalFullJacobian(context, &J);
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
    KinematicEvaluatorSet<T> other) const {
  std::vector<int> union_indices;
  for (int i = 0; i < other.num_evaluators(); i++) {
    if (std::find(evaluators_.begin(), evaluators_.end(),
                  &other.get_evaluator(i)) != evaluators_.end()) {
      union_indices.push_back(i);
    }
  }
  return union_indices;
}

template <typename T>
std::vector<int> KinematicEvaluatorSet<T>::FindFullIndicesUnion(
    KinematicEvaluatorSet<T> other) const {
  std::vector<int> row_indices;
  for (int i = 0; i < other.num_evaluators(); i++) {
    if (std::find(evaluators_.begin(), evaluators_.end(),
                  &other.get_evaluator(i)) != evaluators_.end()) {
      int start_index = other.evaluator_full_start(i);
      for (int j = 0; j < other.get_evaluator(i).num_full(); j++) {
        row_indices.push_back(start_index + j);
      }
    }
  }
  return row_indices;
}

template <typename T>
std::vector<int> KinematicEvaluatorSet<T>::FindActiveIndicesUnion(
    KinematicEvaluatorSet<T> other) const {
  std::vector<int> row_indices;
  for (int i = 0; i < other.num_evaluators(); i++) {
    if (std::find(evaluators_.begin(), evaluators_.end(),
                  &other.get_evaluator(i)) != evaluators_.end()) {
      int start_index = other.evaluator_active_start(i);
      for (int j = 0; j < other.get_evaluator(i).num_active(); j++) {
        row_indices.push_back(start_index + j);
      }
    }
  }
  return row_indices;
}

template <typename T>
VectorX<T> KinematicEvaluatorSet<T>::CalcMassMatrixTimesVDot(
    const Context<T>& context, const VectorX<T>& lambda) const {
  // M(q)vdot + C(q,v) = tau_g(q) + F_app + Bu + J(q)^T lambda
  VectorX<T> C(plant_.num_velocities());
  plant_.CalcBiasTerm(context, &C);

  VectorX<T> Bu = plant_.MakeActuationMatrix() *
                  plant_.get_actuation_input_port().Eval(context);

  VectorX<T> tau_g = plant_.CalcGravityGeneralizedForces(context);

  drake::multibody::MultibodyForces<T> f_app(plant_);
  plant_.CalcForceElementsContribution(context, &f_app);

  VectorX<T> J_transpose_lambda =
      EvalFullJacobian(context).transpose() * lambda;

  return tau_g + f_app.generalized_forces() + Bu + J_transpose_lambda - C;
}

template <typename T>
VectorX<T> KinematicEvaluatorSet<T>::CalcTimeDerivativesWithForce(
    Context<T>* context, const VectorX<T>& lambda) const {
  MatrixX<T> J(count_full(), plant_.num_velocities());
  EvalFullJacobian(*context, &J);
  VectorX<T> J_transpose_lambda = J.transpose() * lambda;

  plant_.get_applied_generalized_force_input_port().FixValue(
      context, J_transpose_lambda);

  // N.B. Evaluating the generalized acceleration port rather than the time
  // derivatives to ensure that this supports continuous and discrete plants
  // (discrete plants would not compute time derivatives)
  const VectorX<T>& v_dot =
      plant_.get_generalized_acceleration_output_port()
          .template Eval<drake::systems::BasicVector<double>>(*context)
          .CopyToVector();
  VectorX<T> x_dot(plant_.num_positions() + plant_.num_velocities());
  VectorX<T> q_dot(plant_.num_positions());
  plant_.MapVelocityToQDot(*context, plant_.GetVelocities(*context), &q_dot);
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
  // M(q) vdot + C(q,v) = tau_g(q) + f_app + Bu + J(q)^T lambda
  // J vdot + Jdotv  + kp phi + kd phidot = 0
  // Produces linear system of equations
  // [[M -J^T]  [[vdot  ]  =  [[tau_g + f_app + Bu - C     ]
  //  [J  0 ]]  [lambda]]     [ -Jdotv - kp phi - kd phidot]]

  // Evaluate manipulator equation terms
  MatrixX<T> M(plant_.num_velocities(), plant_.num_velocities());
  plant_.CalcMassMatrix(context, &M);

  VectorX<T> C(plant_.num_velocities());
  plant_.CalcBiasTerm(context, &C);

  VectorX<T> Bu = plant_.MakeActuationMatrix() *
                  plant_.get_actuation_input_port().Eval(context);

  VectorX<T> tau_g = plant_.CalcGravityGeneralizedForces(context);

  drake::multibody::MultibodyForces<T> f_app(plant_);
  plant_.CalcForceElementsContribution(context, &f_app);

  // Evaluate active constraint terms.
  VectorX<T> phi = EvalActive(context);
  VectorX<T> phidot = EvalActiveTimeDerivative(context);
  MatrixX<T> J = EvalActiveJacobian(context);
  VectorX<T> Jdotv = EvalActiveJacobianDotTimesV(context);

  MatrixX<T> A(M.rows() + J.rows(), M.cols() + J.rows());
  VectorX<T> b(M.rows() + J.rows());
  A << M, -J.transpose(), J, MatrixX<T>::Zero(J.rows(), J.rows());
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
bool KinematicEvaluatorSet<T>::is_active(int index) const {
  if (index < 0 || index >= count_full()) {
    return false;
  }

  // Loop over the individual evaluators. If the given index is in the index
  // range for a particular evaluator, check if it is active.
  // `count` here refers to the starting index of the current evaluator into the
  // full vector.
  int count = 0;
  for (const auto& e : evaluators_) {
    if (index >= count && index < count + e->num_full()) {
      return e->is_active(index - count);
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
