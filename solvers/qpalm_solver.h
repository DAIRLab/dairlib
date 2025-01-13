#pragma once
#include "qp_data.h"
#include "qpalm.hpp"

namespace dairlib::solvers {


/*!
 * Lightweight wrapper class to store a solver object and problem data for the
 * QPALM solver
 */
class QPALMSolver {

 public:
  /*!
   * Construct the solver based on the problem dimensions
   * @param n number of decision variables
   * @param m number of rows in the constraints, lb <= Ax < ub
   */
  explicit QPALMSolver(long n, long m);
  explicit QPALMSolver(const QPData& qp) : QPALMSolver(qp.num_vars, qp.num_ineq) {
    DRAKE_DEMAND(qp.num_eq == 0 && "ALL constraints should be forulated as "
                                   "inequalities for QPALM compatibility");
  };

  void Solve(const QPData& qp, Eigen::VectorXd& x);
  Eigen::VectorXd Solve(const QPData& qp);


 private:
  qpalm::Data data_;
  qpalm::Settings settings_{};

  std::unique_ptr<qpalm::Solver> solver_;
  bool init_ = false;

};



}

