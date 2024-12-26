#include "collocation_constraint.h"

namespace dairlib::systems::controllers::id_mpc {

using drake::VectorX;
using Eigen::VectorXd;

template <typename T>
TrapezoidalCollocationConstraint<T>::TrapezoidalCollocationConstraint(
    KnotPointState *x0, KnotPointState *x1, double dt) :
    solvers::NonlinearConstraint<T>(
        x0->get_dynamics().nx(), 2 * x0->get_dynamics().variable_count(),
        VectorXd::Zero(x0->get_dynamics().nx()),
        VectorXd::Zero(x0->get_dynamics().nx())),
        x0_state_(x0), x1_state_(x1), dt_(dt) {

  DRAKE_DEMAND(x0_state_ != nullptr);
  DRAKE_DEMAND(x1_state_ != nullptr);
  DRAKE_DEMAND(dt > 0);
}


template <typename T>
void TrapezoidalCollocationConstraint<T>::EvaluateConstraint(
    const Eigen::Ref<const VectorX<T>> &x, VectorX<T> *y) const {
  DRAKE_ASSERT(x.rows() == 2 * x0_state_->get_dynamics().variable_count());

  int nx = x0_state_->get_dynamics().nx();
  int var_count = x0_state_->get_dynamics().variable_count();

  x0_state_->Update<T>(x.head(var_count));
  x1_state_->Update<T>(x.tail(var_count));

  const VectorX<T>& x0 = x.head(nx);
  const VectorX<T>& x1 = x.segment(var_count, nx);

  *y = x1 - x0 - 0.5 * dt_ * (x0_state_->GetXDot<T>() + x1_state_->GetXDot<T>());
}

template class TrapezoidalCollocationConstraint<double>;
template class TrapezoidalCollocationConstraint<AutoDiffXd>;

}