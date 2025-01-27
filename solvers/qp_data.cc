#include "solvers/qp_data.h"

namespace dairlib::solvers {

using Eigen::Triplet;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::SparseMatrix;

using drake::solvers::MathematicalProgram;

void QPData::ToMathematicalProgram(MathematicalProgram &prog) const {
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

// TODO (@Brian-Acosta) handle BoundingBox Constraints
QPData QPData::ToQPData(const drake::solvers::MathematicalProgram &prog) {
  QPData qp;

  qp.num_vars = prog.num_vars();
  qp.g = VectorXd::Zero(prog.num_vars());
  qp.c = 0;

  std::vector<Triplet<double>> cost_triplets;
  for (const auto& binding : prog.quadratic_costs()) {
    const auto& vars = binding.variables();
    const auto indices = prog.FindDecisionVariableIndices(vars);
    AppendQuadraticCost(indices,
                        binding.evaluator()->Q(),
                        binding.evaluator()->b(),
                        binding.evaluator()->c(),
                        cost_triplets, qp.g, &qp.c);
  }
  qp.H.resize(prog.num_vars(), prog.num_vars());
  qp.H.setFromTriplets(cost_triplets.begin(), cost_triplets.end());

  for (const auto& binding: prog.linear_costs()) {
    const auto& vars = binding.variables();
    const auto indices = prog.FindDecisionVariableIndices(vars);
    for (int i = 0; i < vars.rows(); ++i) {
      qp.g(indices[i]) += binding.evaluator()->a()(i);
    }
    qp.c += binding.evaluator()->b();
  }

  std::vector<Triplet<double>> constraint_triplets;
  for (const auto& binding : prog.linear_constraints()) {
    const auto& vars = binding.variables();
    const auto indices = prog.FindDecisionVariableIndices(vars);
    AppendLinearConstraint(
        indices,
        qp.num_ineq,
        binding.evaluator()->GetDenseA(),
        binding.evaluator()->lower_bound(),
        binding.evaluator()->upper_bound(),
        constraint_triplets, qp.lb, qp.ub);
    qp.num_ineq += binding.evaluator()->num_constraints();
  }
  for (const auto& binding : prog.linear_equality_constraints()) {
    const auto& vars = binding.variables();
    const auto indices = prog.FindDecisionVariableIndices(vars);
    AppendLinearConstraint(
        indices,
        qp.num_ineq,
        binding.evaluator()->GetDenseA(),
        binding.evaluator()->lower_bound(),
        binding.evaluator()->upper_bound(),
        constraint_triplets, qp.lb, qp.ub);
    qp.num_ineq += binding.evaluator()->num_constraints();
  }
  qp.A.resize(qp.num_ineq, qp.num_vars);
  qp.A.setFromTriplets(constraint_triplets.begin(), constraint_triplets.end());
  return qp;
}

void AppendQuadraticCost(
    const std::vector<int> &variable_indices, const Eigen::MatrixXd &H,
    const Eigen::MatrixXd &b, double c,
    std::vector<Eigen::Triplet<double>> &triplets, Eigen::VectorXd &qp_g,
    double *qp_c, bool diagonal_hessian) {

  if (diagonal_hessian) {
    for (int i = 0; i < H.cols(); ++i) {
      int idx = variable_indices[i];
      triplets.emplace_back(idx, idx, H(i, i));
    }
  } else {
    for (int j = 0; j < H.cols(); ++j) {
      for (int i = 0; i < H.rows(); ++i) {
        int row = variable_indices[i];
        int col = variable_indices[j];
        // see https://github.com/RobotLocomotion/drake/blob/962483669d015ee2618585ace2c884598fbadbb5/solvers/aggregate_costs_constraints.cc#L476
        const double factor = (i != j && row == col) ? 2 : 1;
        triplets.emplace_back(row, col, factor * H(i, j));
      }
    }
  }

  for (int i = 0; i < b.rows(); ++i) {
    qp_g(variable_indices[i]) += b(i);
  }
  *qp_c += c;
}

void AppendLinearConstraint(
    const std::vector<int> &variable_indices, int start_row,
    const MatrixXd &A, const MatrixXd &lb, const MatrixXd &ub,
    std::vector<Triplet<double>> &A_triplets, VectorXd &qp_lb, VectorXd &qp_ub) {
  DRAKE_DEMAND(qp_ub.rows() >= start_row);
  DRAKE_DEMAND(qp_lb.rows() >= start_row);
  DRAKE_DEMAND(A.rows() == ub.rows());
  DRAKE_DEMAND(A.rows() == lb.rows());

  Eigen::Index rows = A.rows();
  qp_lb.conservativeResize(std::max(lb.rows(), start_row + rows));
  qp_ub.conservativeResize(std::max(ub.rows(), start_row + rows));
  qp_lb.segment(start_row, rows) = lb;
  qp_ub.segment(start_row, rows) = ub;

  for (int j = 0; j < A.cols(); ++j) {
    for (int i = 0; i < A.rows(); ++i) {
      int row = start_row + i;
      int col = variable_indices[j];
      A_triplets.emplace_back(row, col, A(i, j));
    }
  }

}

}