#include "qpalm_solver.h"

namespace dairlib::solvers {

using Eigen::VectorXd;
using Eigen::Map;


QPALMSolver::QPALMSolver(long n, long m) : n_(n), data_(n, m){
  settings_.verbose = true;
  settings_.max_rank_update = n + m;
  settings_.max_rank_update_fraction = 1.0;
  settings_.factorization_method = 0;
  settings_.warm_start = true;
  settings_.scaling = 0;
}

VectorXd QPALMSolver::Solve(const dairlib::solvers::QPData &qp) {
  VectorXd result = VectorXd::Zero(data_.n);
  Solve(qp, result);
  return result;
}


// TODO (@Brian-Acosta) need to convert infinite constraint bounds to
//  QPALM_INFTY?
void QPALMSolver::Solve(const dairlib::solvers::QPData &qp, VectorXd &x) {
  if (!init_) {
    // need to make copies to convert to the type expected by qpalm
    qpalm::sparse_mat_t qp_H = qp.H;
    qpalm::sparse_mat_t qp_A = qp.A;
    data_.set_Q(qp_H);
    data_.set_A(qp_A);
    data_.q = qp.g;
    data_.c = qp.c;
    data_.bmin = qp.lb;
    data_.bmax = qp.ub;
    solver_ = std::make_unique<qpalm::Solver>(data_, settings_);
    init_ = true;
  } else {
    solver_->update_Q_A(
        Map<const VectorXd>(qp.H.valuePtr(), qp.H.nonZeros()),
        Map<const VectorXd>(qp.A.valuePtr(), qp.A.nonZeros())
    );
    solver_->update_bounds(qp.lb, qp.ub);
    solver_->update_q(qp.g);
  }
  solver_->solve();
  x = solver_->get_solution().x;
}

}
