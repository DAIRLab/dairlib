#include "collocation_constraint.h"
#include <iostream>

namespace dairlib::systems::controllers::id_mpc {

using drake::VectorX;
using drake::MatrixX;
using Eigen::MatrixXd;
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

  const auto& dynamics = timeline_->knots.front().get_dynamics();
  int nvars = dynamics.variable_count();
  int nx = dynamics.nx();
  int nq = dynamics.nq();
  int nv = dynamics.nv();
  const MatrixXd& B = dynamics.get_plant().MakeActuationMatrix();

  for (int i = 0; i < timeline_->nknots() - 1; ++i) {
    double dt = timeline_->breaks.at(i+1) - timeline_->breaks.at(i);
    const auto& k0 = timeline_->knots.at(i);
    const auto& k1 = timeline_->knots.at(i+1);
    const VectorX<T>& x0 = x.segment(i*nvars, nvars);
    const VectorX<T>& x1 = x.segment((i+1) * nvars, nvars);

    y->segment(i * nx, nq) =  dynamics.get_q(x0) - dynamics.get_q(x1)
        - 0.5 * dt * (k0.GetQDot<T>() + k1.GetQDot<T>());
    y->segment(i * nx + nq, nv) = B * dynamics.get_u(x0) - k0.GetTau<T>();
  }
}

template class CollocationConstraint<double>;
template class CollocationConstraint<AutoDiffXd>;

}