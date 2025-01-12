#include "qp_data.h"

namespace dairlib::systems::controllers::id_mpc {

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::SparseMatrix;

using drake::solvers::MathematicalProgram;


void QPData::ToMathematicalProgram(MathematicalProgram& prog) const {
  DRAKE_DEMAND(ValidateDimensions());
  DRAKE_DEMAND(prog.num_vars() == 0);

  auto x = prog.NewContinuousVariables(num_vars);
  prog.AddQuadraticCost(MatrixXd(H), g, c, x);

  if (num_ineq > 0) {
    prog.AddLinearConstraint(A, lb, ub, x);
  }
  if (num_eq > 0) {
    prog.AddLinearEqualityConstraint(A_eq, b_eq, x);
  }
}

bool QPData::ValidateDimensions() const {
  DRAKE_DEMAND(H.rows() == H.cols());
  DRAKE_DEMAND(g.rows() == num_vars);
  DRAKE_DEMAND(H.rows() == num_vars);
  DRAKE_DEMAND(num_eq == A_eq.rows());
  DRAKE_DEMAND(num_ineq == A.rows());
  DRAKE_DEMAND(num_eq == b_eq.rows());
  DRAKE_DEMAND(num_ineq == lb.rows());
  DRAKE_DEMAND(num_ineq == ub.rows());
  return true;
}

void AppendQuadraticCost(
    const std::vector<int>& variable_indices, const Eigen::MatrixXd& H,
    const Eigen::MatrixXd& b, double c,
    std::vector<Eigen::Triplet<double>>& triplets, Eigen::VectorXd& qp_g,
    double* qp_c) {

  for (int j = 0; j < H.cols(); ++j) {
    for (int i = 0; i < H.rows(); ++i) {
      int row = variable_indices[i];
      int col = variable_indices[j];
      // see https://github.com/RobotLocomotion/drake/blob/962483669d015ee2618585ace2c884598fbadbb5/solvers/aggregate_costs_constraints.cc#L476
      const double factor = (i != j && row == col) ? 2 : 1;
      triplets.emplace_back(row, col, factor * H(i,j));
    }
  }

  for (int i = 0; i < b.rows(); ++i) {
    qp_g(variable_indices[i]) += b(i);
  }
  *qp_c += c;
}

}
