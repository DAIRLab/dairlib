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
  Eigen::SparseMatrix<double> H;    // Cost Hessian
  Eigen::SparseMatrix<double> A;    // Linear constraint A
  Eigen::SparseMatrix<double> A_eq; // Linear equality constraint A

  Eigen::VectorXd g;                // Cost gradient
  Eigen::VectorXd lb;               // linear constraint lb
  Eigen::VectorXd ub;               // linear constraint ub
  Eigen::VectorXd b_eq;             // linear equality constraint
  double c;                         // cost constant term

  void ToMathematicalProgram(drake::solvers::MathematicalProgram& empty_prog) const;
  bool ValidateDimensions() const;
  static QPData ToQPData(drake::solvers::MathematicalProgram& qp_prog);

};

void AppendQuadraticCost(
    const std::vector<int>& variable_indices, const Eigen::MatrixXd& H,
    const Eigen::MatrixXd& b, double c,
    std::vector<Eigen::Triplet<double>>& triplets, Eigen::VectorXd& qp_g,
    double* qp_c);

void AppendLinearConstraint(
    const std::vector<int>& variable_indices, int start_row,
    const Eigen::MatrixXd& A,
    const Eigen::MatrixXd &lb, const Eigen::MatrixXd& ub,
    std::vector<Eigen::Triplet<double>>& A_triplets, Eigen::VectorXd& qp_lb,
    Eigen::VectorXd& qp_ub);

}
