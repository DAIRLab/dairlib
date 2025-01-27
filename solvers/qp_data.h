#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "drake/solvers/mathematical_program.h"

namespace dairlib::solvers {

struct QPData {

  long int num_vars{0};
  long int num_eq{0};
  long int num_ineq{0};

  // These need to have long as the storage index for qpalm compatibility
  Eigen::SparseMatrix<double> H{};    // Cost Hessian
  Eigen::SparseMatrix<double> A{};    // Linear constraint A
  Eigen::SparseMatrix<double> A_eq{}; // Linear equality constraint A

  Eigen::VectorXd g = Eigen::VectorXd::Zero(0);    // Cost gradient
  Eigen::VectorXd lb = Eigen::VectorXd::Zero(0);   // linear constraint lb
  Eigen::VectorXd ub = Eigen::VectorXd::Zero(0);   // linear constraint ub
  Eigen::VectorXd b_eq = Eigen::VectorXd::Zero(0); // linear equality constraint
  double c = 0;                         // cost constant term

  void ToMathematicalProgram(drake::solvers::MathematicalProgram& empty_prog) const;
  bool ValidateDimensions() const;
  static QPData ToQPData(const drake::solvers::MathematicalProgram& qp_prog);

  friend
  std::ostream& operator<<(std::ostream& os, const QPData& data) {
    os << "{vars:" << data.num_vars << " eq:" << data.num_eq << " ineq:" << data.num_ineq
       << "\nH:\n" << Eigen::MatrixXd(data.H)
       << "\nA:\n" << Eigen::MatrixXd(data.A)
       << "\nA_eq\n:" << Eigen::MatrixXd(data.A_eq)
       << "\ng:" << data.g.transpose()
       << "\nlb:" << data.lb.transpose()
       << "\nub:" << data.ub.transpose()
       << "\nb_eq:" << data.b_eq.transpose()
       << "\nc:" << data.c << "}";
    return os;
  }

};

struct QPResult {
  Eigen::VectorXd x;  // solution
  Eigen::VectorXd y;  // dual solution
  std::string status;
  double run_time;
  double primal_res;
  double dual_res;
  double objective;
  bool success = false;

  friend
  std::ostream& operator<<(std::ostream& os, const QPResult& result) {
    os << "{status:" << result.status
       << "\nsuccess:" << std::boolalpha << result.success
       << "\nx:" << result.x.transpose()
       << "\ny:" << result.y.transpose()
       << "\nobj:" << result.objective
       << "\np_res:" << result.primal_res
       << "\nd_res:" << result.dual_res
       << "\ntime:" << result.run_time << "}\n";
    return os;
  }
};

void AppendQuadraticCost(
    const std::vector<int>& variable_indices, const Eigen::MatrixXd& H,
    const Eigen::MatrixXd& b, double c,
    std::vector<Eigen::Triplet<double>>& triplets, Eigen::VectorXd& qp_g,
    double* qp_c, bool diagonal_hessian=false);

void AppendLinearConstraint(
    const std::vector<int>& variable_indices, int start_row,
    const Eigen::MatrixXd& A,
    const Eigen::MatrixXd &lb, const Eigen::MatrixXd& ub,
    std::vector<Eigen::Triplet<double>>& A_triplets, Eigen::VectorXd& qp_lb,
    Eigen::VectorXd& qp_ub);

}
