#include "fast_proxqp_solver.h"

// Need to include iostream BEFORE including proxsuite to avoid eigen
// compilation error - some header proxsuite is calling is missing that include
#include <iostream>
#include "proxsuite/proxqp/sparse/sparse.hpp"

#include <optional>
#include <unordered_map>
#include <vector>

#include "drake/common/text_logging.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/osqp_solver.h"
#include "drake/math/eigen_sparse_triplet.h"

using drake::math::SparseMatrixToTriplets;
using drake::solvers::internal::BindingDynamicCast;
using drake::solvers::MathematicalProgramResult;
using drake::solvers::VectorXDecisionVariable;
using drake::solvers::MathematicalProgram;
using drake::solvers::OsqpSolverDetails;
using drake::solvers::SolutionResult;
using drake::solvers::SolverOptions;
using drake::solvers::OsqpSolver;
using drake::solvers::Constraint;
using drake::solvers::Binding;

using proxsuite::proxqp::sparse::isize;
using proxsuite::proxqp::sparse::QP;



using c_int = long long;
using c_float = double;

template<typename T>
using SparseMat = Eigen::SparseMatrix<T, Eigen::ColMajor, c_int>;
using ProxQPSettings = proxsuite::proxqp::Settings<c_float>;

namespace dairlib {
namespace solvers{

namespace {
void ParseQuadraticCosts(const MathematicalProgram &prog,
                         SparseMat<c_float> *H,
                         std::vector<c_float> *g, double *constant_cost_term) {
  DRAKE_ASSERT(static_cast<int>(g->size()) == prog.num_vars());

  // Loop through each quadratic costs in prog, and compute the Hessian matrix
  // H, the linear cost g, and the constant cost term.
  std::vector<Eigen::Triplet<c_float>> H_triplets;
  for (const auto &quadratic_cost : prog.quadratic_costs()) {
    const VectorXDecisionVariable &x = quadratic_cost.variables();
    // x_indices are the indices of the variables x (the variables bound with
    // this quadratic cost) in the program decision variables.
    const std::vector<int> x_indices = prog.FindDecisionVariableIndices(x);

    const Eigen::MatrixXd &Q = quadratic_cost.evaluator()->Q();
    for (int col = 0; col < Q.cols(); ++col) {
      for (int row = 0; row < Q.rows(); ++row) {
        const double value = Q(row, col);
        if (value == 0.0) {
          continue;
        }
        const int x_row = x_indices[row];
        const int x_col = x_indices[col];
        H_triplets.emplace_back(x_row, x_col, value);
      }
    }
    
    // Add quadratic_cost.b to the linear cost term q.
    for (int i = 0; i < x.rows(); ++i) {
      g->at(x_indices[i]) += quadratic_cost.evaluator()->b()(i);
    }

    // Add quadratic_cost.c to constant term
    *constant_cost_term += quadratic_cost.evaluator()->c();
  }

  // Scale the matrix P in the cost.
  // Note that the linear term is scaled in ParseLinearCosts().
  const auto &scale_map = prog.GetVariableScaling();
  if (!scale_map.empty()) {
    for (auto &triplet : H_triplets) {
      // Column
      const auto column = scale_map.find(triplet.col());
      if (column != scale_map.end()) {
        triplet = Eigen::Triplet<double>(triplet.row(), triplet.col(),
                                         triplet.value() * (column->second));
      }
      // Row
      const auto row = scale_map.find(triplet.row());
      if (row != scale_map.end()) {
        triplet = Eigen::Triplet<double>(triplet.row(), triplet.col(),
                                         triplet.value() * (row->second));
      }
    }
  }

  H->resize(prog.num_vars(), prog.num_vars());
  H->setFromTriplets(H_triplets.begin(), H_triplets.end());
}

void ParseLinearCosts(const MathematicalProgram& prog, std::vector<c_float>* g,
                      double* constant_cost_term) {
  // Add the linear costs to the osqp cost.
  DRAKE_ASSERT(static_cast<int>(g->size()) == prog.num_vars());

  // Loop over the linear costs stored inside prog.
  for (const auto& linear_cost : prog.linear_costs()) {
    for (int i = 0; i < static_cast<int>(linear_cost.GetNumElements()); ++i) {
      // Append the linear cost term to q.
      if (linear_cost.evaluator()->a()(i) != 0) {
        const int x_index =
            prog.FindDecisionVariableIndex(linear_cost.variables()(i));
        g->at(x_index) += linear_cost.evaluator()->a()(i);
      }
    }
    // Add the constant cost term to constant_cost_term.
    *constant_cost_term += linear_cost.evaluator()->b();
  }

  // Scale the vector q in the cost.
  const auto& scale_map = prog.GetVariableScaling();
  if (!scale_map.empty()) {
    for (const auto& [index, scale] : scale_map) {
      g->at(index) *= scale;
    }
  }
}

// Will call this function to parse both LinearConstraint and
// LinearEqualityConstraint.
template <typename C>
void ParseLinearConstraints(
    const MathematicalProgram& prog,
    const std::vector<Binding<C>>& linear_constraints,
    std::vector<Eigen::Triplet<c_float>>* C_triplets, std::vector<c_float>* l,
    std::vector<c_float>* u, int* num_C_rows,
    std::unordered_map<Binding<Constraint>, int>* constraint_start_row) {
  // Loop over the linear constraints, stack them to get l, u and A.
  for (const auto& constraint : linear_constraints) {
    const std::vector<int> x_indices =
        prog.FindDecisionVariableIndices(constraint.variables());
    const std::vector<Eigen::Triplet<double>> Ci_triplets =
        SparseMatrixToTriplets(constraint.evaluator()->get_sparse_A());
    const Binding<Constraint> constraint_cast =
        BindingDynamicCast<Constraint>(constraint);
    constraint_start_row->emplace(constraint_cast, *num_C_rows);
    // Append constraint.A to ProxQP C.
    for (const auto& Ci_triplet : Ci_triplets) {
      C_triplets->emplace_back(*num_C_rows + Ci_triplet.row(),
                               x_indices[Ci_triplet.col()],
                               static_cast<c_float>(Ci_triplet.value()));
    }
    const int num_Ci_rows = constraint.evaluator()->num_constraints();
    l->reserve(l->size() + num_Ci_rows);
    u->reserve(u->size() + num_Ci_rows);
    for (int i = 0; i < num_Ci_rows; ++i) {
      l->push_back(constraint.evaluator()->lower_bound()(i));
      u->push_back(constraint.evaluator()->upper_bound()(i));
    }
    *num_C_rows += num_Ci_rows;
  }
}

void ParseBoundingBoxConstraints(
    const MathematicalProgram& prog,
    std::vector<Eigen::Triplet<c_float>>* C_triplets, std::vector<c_float>* l,
    std::vector<c_float>* u, int* num_C_rows,
    std::unordered_map<Binding<Constraint>, int>* constraint_start_row) {
  // Loop over the linear constraints, stack them to get l, u and A.
  for (const auto& constraint : prog.bounding_box_constraints()) {
    const Binding<Constraint> constraint_cast =
       BindingDynamicCast<Constraint>(constraint);
    constraint_start_row->emplace(constraint_cast, *num_C_rows);
    // Append constraint.A to ProxQP C.
    for (int i = 0; i < static_cast<int>(constraint.GetNumElements()); ++i) {
      C_triplets->emplace_back(
          *num_C_rows + i,
          prog.FindDecisionVariableIndex(constraint.variables()(i)),
          static_cast<c_float>(1));
    }
    const int num_Ci_rows = constraint.evaluator()->num_constraints();
    l->reserve(l->size() + num_Ci_rows);
    u->reserve(u->size() + num_Ci_rows);
    for (int i = 0; i < num_Ci_rows; ++i) {
      l->push_back(constraint.evaluator()->lower_bound()(i));
      u->push_back(constraint.evaluator()->upper_bound()(i));
    }
    *num_C_rows += num_Ci_rows;
  }
}

void ParseAllLinearConstraints(
    const MathematicalProgram& prog, SparseMat<double>* C,
    std::vector<c_float>* l, std::vector<c_float>* u,
    std::unordered_map<Binding<Constraint>, int>* constraint_start_row) {

  std::vector<Eigen::Triplet<c_float>> C_triplets;
  u->clear();
  int num_C_rows = 0;
  ParseLinearConstraints(prog, prog.linear_constraints(), &C_triplets, l, u,
                         &num_C_rows, constraint_start_row);

  ParseBoundingBoxConstraints(prog, &C_triplets, l, u, &num_C_rows, constraint_start_row);

  // Scale the matrix C.
  // Note that we only scale the columns of C, because the constraint has the
  // form Cx <= u where the scaling of x enters the columns of C

  const auto& scale_map = prog.GetVariableScaling();
  if (!scale_map.empty()) {
    for (auto& triplet : C_triplets) {
      auto column = scale_map.find(triplet.col());
      if (column != scale_map.end()) {
        triplet = Eigen::Triplet<double>(triplet.row(), triplet.col(),
                                         triplet.value() * (column->second));
      }
    }
  }

  C->resize(num_C_rows, prog.num_vars());
  C->setFromTriplets(C_triplets.begin(), C_triplets.end());
}

void ParseLinearEqualityConstraints(
    const MathematicalProgram& prog,
    SparseMat<c_float>* A, std::vector<c_float>* b,
    std::unordered_map<Binding<Constraint>, int>* constraint_start_row) {

  b->clear();
  std::vector<Eigen::Triplet<c_float>> A_triplets;

  int num_A_rows = 0;

  for (const auto& constraint: prog.linear_equality_constraints()) {
    const auto& x = constraint.variables();
    const auto& x_indices = prog.FindDecisionVariableIndices(x);
    const std::vector<Eigen::Triplet<double>> Ai_triplets =
        SparseMatrixToTriplets(constraint.evaluator()->get_sparse_A());
    const Binding<Constraint> constraint_cast =
        BindingDynamicCast<Constraint>(constraint);
    constraint_start_row->emplace(constraint_cast, num_A_rows);
    for (const auto& Ai_triplet : Ai_triplets) {
      A_triplets.emplace_back(
          num_A_rows + Ai_triplet.row(),
          x_indices[Ai_triplet.col()],
          static_cast<c_float>(Ai_triplet.value()));
    }
    const int num_Ai_rows = constraint.evaluator()->num_constraints();
    const auto& bi = constraint.evaluator()->lower_bound();
    b->reserve(b->size() + num_Ai_rows);
    for (int i = 0; i < num_Ai_rows; i++) {
      b->push_back(bi(i));
    }
    num_A_rows += num_Ai_rows;
  }

  A->resize(num_A_rows, prog.num_vars());
  A->setFromTriplets(A_triplets.begin(), A_triplets.end());
}

template <typename C>
void SetDualSolutionInequality(
    const std::vector<Binding<C>>& constraints,
    const Eigen::VectorXd& all_dual_solution,
    const std::unordered_map<Binding<Constraint>, int>& constraint_start_row,
    MathematicalProgramResult* result) {
  for (const auto& constraint : constraints) {
    // OSQP uses the dual variable `y` as the negation of the shadow price, so
    // we need to negate `all_dual_solution` as Drake interprets dual solution
    // as the shadow price.
    const Binding<Constraint> constraint_cast =
        BindingDynamicCast<Constraint>(constraint);
    result->set_dual_solution(
        constraint,
        -all_dual_solution.segment(constraint_start_row.at(constraint_cast),
                                   constraint.evaluator()->num_constraints()));
  }
}

template <typename T1, typename T2>
void SetProxQPSolverSetting(
    const std::unordered_map<std::string, T1>& options,
    const std::string& option_name,
    T2* field) {

  const auto it = options.find(option_name);
  if (it != options.end()) {
    *field = it->second;
  }

}


void SetSolverOptionsFromOsqpOptions(
    const SolverOptions& solver_options, ProxQPSettings* settings) {
  const auto& options_double =
      solver_options.GetOptionsDouble(OsqpSolver::id());
  const auto& options_int =
      solver_options.GetOptionsInt(OsqpSolver::id());

  SetProxQPSolverSetting(options_double, "eps_abs", &(settings->eps_abs));
  SetProxQPSolverSetting(options_double, "eps_rel", &(settings->eps_rel));
  SetProxQPSolverSetting(options_double, "eps_prim_inf", &(settings->eps_primal_inf));
  SetProxQPSolverSetting(options_double, "eps_dual_inf", &(settings->eps_dual_inf));
//  settings->initial_guess = proxsuite::proxqp::InitialGuessStatus::NO_INITIAL_GUESS;
}

} // namespace

bool FastProxQPSolver::is_available() { return true; }

void FastProxQPSolver::DoSolve(
    const drake::solvers::MathematicalProgram& prog,
    const Eigen::VectorXd& initial_guess,
    const drake::solvers::SolverOptions& merged_options,
    drake::solvers::MathematicalProgramResult *result) const {

  auto& solver_details = result->SetSolverDetailsType<ProxQPSolverDetails>();

  // Get the cost for the QP.
  SparseMat<c_float> H_sparse;
  std::vector<c_float> g(prog.num_vars(), 0);
  double constant_cost_term{0};

  ParseQuadraticCosts(prog, &H_sparse, &g, &constant_cost_term);
  ParseLinearCosts(prog, &g, &constant_cost_term);

  std::unordered_map<Binding<Constraint>, int> ineq_constraint_start_row;

  // Parse the linear constraints Cx <= u
  SparseMat<c_float> C_sparse;
  std::vector<c_float> l, u;
  ParseAllLinearConstraints(
      prog, &C_sparse, &l, &u, &ineq_constraint_start_row);

  // Parse the linear equality constraints Ax = b
  std::unordered_map<Binding<Constraint>, int> eq_constraint_start_row;
  SparseMat<c_float> A_sparse;
  std::vector<c_float> b;
  ParseLinearEqualityConstraints(
      prog, &A_sparse, &b, &eq_constraint_start_row);

  isize n = prog.num_vars();
  isize n_eq = A_sparse.rows();
  isize n_ineq = C_sparse.rows();
  Eigen::VectorXd gvect = Eigen::VectorXd::Map(g.data(), g.size());
  Eigen::VectorXd bvect = Eigen::VectorXd::Map(b.data(), b.size());
  Eigen::VectorXd lvect = Eigen::VectorXd::Map(l.data(), l.size());
  Eigen::VectorXd uvect = Eigen::VectorXd::Map(u.data(), u.size());

  QP<c_float, isize> qp(n, n_eq, n_ineq);
  qp.init(H_sparse,
          gvect,
          A_sparse,
          bvect,
          C_sparse,
          lvect,
          uvect);
  qp.solve();
//  qp.solve(initial_guess, std::nullopt, std::nullopt);

  std::optional<SolutionResult> solution_result;

  switch (qp.results.info.status) {
    case proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED:
      solution_result = SolutionResult::kSolutionFound;
      result->set_optimal_cost(qp.results.info.objValue);
      result->set_x_val(qp.results.x);
      break;
    case proxsuite::proxqp::QPSolverOutput::PROXQP_PRIMAL_INFEASIBLE:
      solution_result = SolutionResult::kInfeasibleConstraints;
      break;
    case proxsuite::proxqp::QPSolverOutput::PROXQP_DUAL_INFEASIBLE:
      solution_result = SolutionResult::kDualInfeasible;
      break;
    case proxsuite::proxqp::QPSolverOutput::PROXQP_MAX_ITER_REACHED:
      solution_result = SolutionResult::kIterationLimit;
      break;
    default:
      solution_result = SolutionResult::kInvalidInput;
  }

  result->set_solution_result(solution_result.value());
  if (solution_result.value() != drake::solvers::kInvalidInput) {
    solver_details.setup_time = qp.results.info.setup_time;
    solver_details.solve_time = qp.results.info.solve_time;
    solver_details.run_time = qp.results.info.run_time;
    solver_details.iter = qp.results.info.iter;
    solver_details.primal_res = qp.results.info.pri_res;
    solver_details.dual_res = qp.results.info.dua_res;

  }
}

} //solvers
} //dairlib