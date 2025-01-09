#include "collocation_constraint.h"
#include <iostream>

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
}

template <typename T>
void CollocationConstraint<T>::EvaluateConstraint(
    const Eigen::Ref<const VectorX<T>> &x, VectorX<T> *y) const {

  *y = VectorX<T>::Zero(k0_.dynamics_constraint_dimension());
  double dt = x1_->time() - x0_->time();
  DRAKE_DEMAND(dt > 0);

  const VectorX<T>& q0 = k0_.get_q<T>(x.head(k0_.total_variables()));
  const VectorX<T>& q1 = k1_.get_q<T>(x.tail(k1_.num_state_variables()));

  // TODO (@Brian-Acosta) need to update x0 and x1

  y->head(q0.rows()) =  q0 - q1 - 0.5 * dt * (x0_->GetQDot<T>() + x1_->GetQDot<T>());
    y->segment(i * nx + nq, nv) = B * dynamics.get_u(x0) - k0.GetTau<T>();
  }
}

template class CollocationConstraint<double>;
template class CollocationConstraint<AutoDiffXd>;

}