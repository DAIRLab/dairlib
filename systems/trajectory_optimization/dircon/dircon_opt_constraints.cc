#include "systems/trajectory_optimization/dircon/dircon_opt_constraints.h"
#include "multibody/multibody_utils.h"

namespace dairlib {
namespace systems {
namespace trajectory_optimization {

using multibody::KinematicEvaluatorSet;
using solvers::NonlinearConstraint;

using drake::VectorX;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;

using Eigen::MatrixXd;
using Eigen::VectorXd;

template <typename T>
QuaternionNormConstraint<T>::QuaternionNormConstraint()
    : NonlinearConstraint<T>(1, 4, VectorXd::Zero(1), 
          VectorXd::Zero(1), "quaternion_norm_constraint") {}
template <typename T>
void QuaternionNormConstraint<T>::EvaluateConstraint(
    const Eigen::Ref<const VectorX<T>>& x, VectorX<T>* y) const {
  (*y).resize(1);
  // Using x.norm() is better, numerically, than x.squaredNorm() except when
  // x is near zero. The below is a permutation of x.norm() = 1 that will be
  // differentiable everywhere, unlike x.norm(). 
  *y << sqrt(x.squaredNorm() + 1e-3) - sqrt(1 + 1e-3);
}

template <typename T>
DirconCollocationConstraint<T>::DirconCollocationConstraint(
    const MultibodyPlant<T>& plant,
    const KinematicEvaluatorSet<T>& evaluators, Context<T>* context_0,
    Context<T>* context_1, int mode_index, int knot_index)
    : NonlinearConstraint<T>(
          plant.num_positions() + plant.num_velocities(),
          1 + 2 * (plant.num_positions() + plant.num_velocities()
              + plant.num_actuators())
              + (4 * evaluators.count_full())
              + multibody::QuaternionStartIndices(plant).size(),
          VectorXd::Zero(plant.num_positions() + plant.num_velocities()),
          VectorXd::Zero(plant.num_positions() + plant.num_velocities()),
          "collocation[" + std::to_string(mode_index) + "]["
              + std::to_string(knot_index) + "]"),
      plant_(plant),
      evaluators_(evaluators),
      context_0_(context_0),
      context_1_(context_1),
      context_col_(plant.CreateDefaultContext()),
      quat_start_indices_(multibody::QuaternionStartIndices(plant)),
      n_x_(plant.num_positions() + plant.num_velocities()),
      n_u_(plant.num_actuators()),
      n_l_(evaluators.count_full()) {}

/// The format of the input to the eval() function is in the order
///   - timestep h
///   - x0, state at time k
///   - x1, state at time k+1
///   - u0, control input at time k
///   - u1, control input at time k+1
///   - l0, constraint force at time k
///   - l1, constraint force at time k+1
///   - lc, constraint force at collocation point between k and k+1
///   - gamma, velocity slack at collocation point between k and k+1
///   - quat_slack, quaternion slack at collocation point between k and k+1
template <typename T>
void DirconCollocationConstraint<T>::EvaluateConstraint(
    const Eigen::Ref<const VectorX<T>>& x, VectorX<T>* y) const {
  // Extract decision variables
  const T& h = x(0);
  const auto& x0 = x.segment(1, n_x_);
  const auto& x1 = x.segment(1 + n_x_, n_x_);
  const auto& u0 = x.segment(1 + 2 * n_x_, n_u_);
  const auto& u1 = x.segment(1 + 2 * n_x_ + n_u_, n_u_);
  const auto& l0 = x.segment(1 + 2 * (n_x_ + n_u_), n_l_);
  const auto& l1 = x.segment(1 + 2 * (n_x_ + n_u_) + n_l_, n_l_);
  const auto& lc = x.segment(1 + 2 * (n_x_ + n_u_) + 2* n_l_, n_l_);
  const auto& gamma = x.segment(1 + 2 * (n_x_ + n_u_) + 3* n_l_, n_l_);
  const auto& quat_slack = x.segment(1 + 2 * (n_x_ + n_u_) + 4* n_l_,
      quat_start_indices_.size());

  // Evaluate dynamics at k and k+1
  multibody::setContext<T>(plant_, x0, u0, context_0_);
  multibody::setContext<T>(plant_, x1, u1, context_1_);
  const auto& xdot0 = evaluators_.CalcTimeDerivatives(*context_0_, l0);
  const auto& xdot1 = evaluators_.CalcTimeDerivatives(*context_1_, l1);

  // Cubic interpolation to get xcol and xdotcol.
  const auto& xcol = 0.5 * (x0 + x1) + h / 8 * (xdot0 - xdot1);
  const auto& xdotcol = -1.5 * (x0 - x1) / h - .25 * (xdot0 + xdot1);
  const auto& ucol = 0.5 * (u0 + u1);

  // Evaluate dynamics at colocation point
  multibody::setContext<T>(plant_, xcol, ucol, context_col_.get());
  auto g = evaluators_.CalcTimeDerivatives(*context_col_, lc);

  // Add velocity slack contribution, J^T * gamma
  VectorX<T> gamma_in_qdot_space(plant_.num_positions());
  plant_.MapVelocityToQDot(*context_col_,
      evaluators_.EvalFullJacobian(*context_col_).transpose() * gamma,
      &gamma_in_qdot_space);
  g.head(plant_.num_positions()) += gamma_in_qdot_space;

  // Add quaternion slack contribution, quat * slack
  for (uint i = 0; i < quat_start_indices_.size(); i++) {
    g.segment(quat_start_indices_.at(i), 4) +=
        xcol.segment(quat_start_indices_.at(i), 4) * quat_slack(i);
  }

  *y = xdotcol - g;
}

template <typename T>
DirconImpactConstraint<T>::DirconImpactConstraint(
    const MultibodyPlant<T>& plant, const KinematicEvaluatorSet<T>& evaluators,
    Context<T>* context, std::string description)
    : NonlinearConstraint<T>(plant.num_velocities(),
          plant.num_positions() + 2 * plant.num_velocities()
              + evaluators.count_full(),
          VectorXd::Zero(plant.num_velocities()),
          VectorXd::Zero(plant.num_velocities()),
          description),
      plant_(plant),
      evaluators_(evaluators),
      context_(context),
      n_x_(plant.num_positions() + plant.num_velocities()),
      n_l_(evaluators.count_full()) {}

/// The format of the input to the eval() function is in the order
///   - x0, pre-impact state (q,v)
///   - impulse, the impulsive force
///   - v1, the post-impact velocity
template <typename T>
void DirconImpactConstraint<T>::EvaluateConstraint(
    const Eigen::Ref<const VectorX<T>>& vars, VectorX<T>* y) const {
  // Extract decision variables
  const auto& x0 = vars.head(n_x_);
  const auto& impulse = vars.segment(n_x_, n_l_);
  const auto& v1 = vars.segment(n_x_ + n_l_, plant_.num_velocities());

  plant_.SetPositionsAndVelocities(context_, x0);
  drake::MatrixX<T> M(plant_.num_velocities(), plant_.num_velocities());
  plant_.CalcMassMatrix(*context_, &M);

  *y = M * (v1 - x0.tail(plant_.num_velocities()))
      - evaluators_.EvalFullJacobian(*context_).transpose() * impulse;
}

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::systems::trajectory_optimization::QuaternionNormConstraint)
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::systems::trajectory_optimization::DirconCollocationConstraint)
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::systems::trajectory_optimization::DirconImpactConstraint)

}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace dairlib
