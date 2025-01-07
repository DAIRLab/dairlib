#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace dairlib::systems::controllers::id_mpc {

class QPData {
 private:
  int num_vars;
  int num_eq;
  int num_ineq;

  Eigen::SparseMatrix<double> H;    // Cost Hessian
  Eigen::SparseMatrix<double> A;    // Linear constraint A
  Eigen::SparseMatrix<double> A_eq; // Linear equality constraint A
  Eigen::VectorXd g;                // Cost gradient
  Eigen::VectorXd lb;               // linear constraint lb
  Eigen::VectorXd ub;               // linear constraint ub
  Eigen::VectorXd b_eq;             // linear equality constraint
};

}
