#include "collocation_constraint.h"

namespace dairlib::systems::controllers::id_mpc {

using drake::VectorX;
using Eigen::VectorXd;

template <typename T>
CollocationConstraint<T>::CollocationConstraint(Timeline* timeline) :
    solvers::NonlinearConstraint<T>(
        timeline->total_dynamics_constraints(),
        timeline->total_vars(),
        VectorXd::Zero(timeline->total_dynamics_constraints()),
        VectorXd::Zero(timeline->total_dynamics_constraints())),
        timeline_(timeline) {
  DRAKE_DEMAND(timeline_ != nullptr);
}

template <typename T>
void CollocationConstraint<T>::EvaluateConstraint(
    const Eigen::Ref<const VectorX<T>> &x, VectorX<T> *y) const {
  DRAKE_DEMAND(x.rows() == timeline_->total_vars());
  *y = VectorX<T>::Zero(timeline_->total_dynamics_constraints());

  timeline_->Update<T>(x);
  int nvars = timeline_->knots.front().get_dynamics().variable_count();
  int nx = timeline_->knots.front().get_dynamics().nx();
  int nq = timeline_->knots.front().get_dynamics().nq();
  int nv = timeline_->knots.front().get_dynamics().nv();

  for (int i = 0; i < timeline_->nknots() - 1; ++i) {
    double dt = timeline_->breaks.at(i+1) - timeline_->breaks.at(i);
    const auto& x0 = timeline_->knots.at(i);
    const auto& x1 = timeline_->knots.at(i+1);
    y->segment(i * nx, nq) =
        x.segment((i+1) * nvars, nq) - x.segment(i * nvars, nq)
        - 0.5 * dt * (x0.GetQDot<T>() + x1.GetQDot<T>());
    y->segment(i * nx + nq, nv) = x.segment(i * nx + nx, nv) - x0.GetTau<T>();
  }
}

template class CollocationConstraint<double>;
template class CollocationConstraint<AutoDiffXd>;

}