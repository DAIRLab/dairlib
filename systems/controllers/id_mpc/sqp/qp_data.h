#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "drake/solvers/mathematical_program.h"

namespace dairlib::systems::controllers::id_mpc {

struct QPData {

  long int num_vars{0};
  long int num_eq{0};
  long int num_ineq{0};

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

};

}
