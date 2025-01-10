#include "collocation_constraint.h"

namespace dairlib::systems::controllers::id_mpc {

using drake::VectorX;
using drake::MatrixX;
using Eigen::MatrixXd;
using Eigen::VectorXd;

template <typename T>
CollocationConstraint<T>::CollocationConstraint(
    const KnotPoint& k0, const KnotPoint& k1,
    KnotPointState* x0, KnotPointState* x1) :
    solvers::NonlinearConstraint<T>(
        k0.dynamics_constraint_dimension(),
        k0.total_variables() + k1.num_state_variables(),
        VectorXd::Zero(k0.dynamics_constraint_dimension()),
        VectorXd::Zero(k0.dynamics_constraint_dimension())),
        k0_(k0), k1_(k1), x0_(x0), x1_(x1) {
  DRAKE_DEMAND(x0_ != nullptr);
  DRAKE_DEMAND(x1_ != nullptr);
  DRAKE_DEMAND(k0_.index() == k1_.index() - 1);

  // Constraint evaluation code assumes that lambda is well defined for
  // the first knot point so we can call x0.UpdateDynamics()
  const auto& dynamics = x0->get_dynamics();
  DRAKE_DEMAND(k0_.num_input_variables() >= dynamics.nc() + dynamics.nh());
}

template <typename T>
void CollocationConstraint<T>::EvaluateConstraint(
    const Eigen::Ref<const VectorX<T>> &x, VectorX<T> *y) const {
  *y = VectorX<T>::Zero(k0_.dynamics_constraint_dimension());
  double dt = x1_->time() - x0_->time();
  DRAKE_DEMAND(dt > 0);

  const VectorX<T> x0 = x.head(k0_.num_state_variables());
  const VectorX<T> u0 = x.segment(
      k0_.num_state_variables(), k0_.num_input_variables());
  const VectorX<T> x1 = x.tail(k1_.num_state_variables());

  x0_->UpdateKinematics(x0);
  x1_->UpdateKinematics(x1);

  int nq = x0_->get_dynamics().nq();
  int nv = x0_->get_dynamics().nv();

  y->head(nq) = x0.head(nq) - x1.head(nq) -
      0.5 * dt * (x0_->GetQDot<T>() + x1_->GetQDot<T>());

  const VectorX<T> vdot = (x1.tail(nv) - x0.tail(nv)) / dt;
  y->tail(k0_.vdot_constraint_dimension()) = k0_.EvalInverseDynamicsDefect(
      x0_, x0, u0, vdot);
}

template class CollocationConstraint<double>;
template class CollocationConstraint<AutoDiffXd>;

}