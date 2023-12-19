#include "fast_osqp_solver.h"

#include <iostream>
#include <vector>

#include <osqp.h>

#include "drake/common/text_logging.h"
#include "drake/math/eigen_sparse_triplet.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/osqp_solver.h"

using drake::solvers::Binding;
using drake::solvers::Constraint;
using drake::solvers::MathematicalProgram;
using drake::solvers::MathematicalProgramResult;
using drake::solvers::OsqpSolver;
using drake::solvers::OsqpSolverDetails;
using drake::solvers::SolutionResult;
using drake::solvers::SolverOptions;
using drake::solvers::VectorXDecisionVariable;
using drake::solvers::internal::BindingDynamicCast;

namespace dairlib {
namespace solvers {
namespace {

void ParseQuadraticCosts(const MathematicalProgram& prog,
                         std::vector<Eigen::Triplet<c_float>>& P_triplets,
                         Eigen::SparseMatrix<c_float>* P,
                         std::vector<c_float>* q,
                         double* constant_cost_term) {
  DRAKE_ASSERT(static_cast<int>(q->size()) == prog.num_vars());

  // Loop through each quadratic costs in prog, and compute the Hessian matrix
  // P, the linear cost q, and the constant cost term.
  P_triplets.clear();
  for (const auto& quadratic_cost : prog.quadratic_costs()) {
    const VectorXDecisionVariable& x = quadratic_cost.variables();
    // x_indices are the indices of the variables x (the variables bound with
    // this quadratic cost) in the program decision variables.
    const std::vector<int> x_indices = prog.FindDecisionVariableIndices(x);

    // Add quadratic_cost.Q to the Hessian P.
    // Since OSQP 0.6.0 the P matrix is required to be upper triangular, so
    // we only add upper triangular entries to P_triplets.
    const Eigen::MatrixXd& Q = quadratic_cost.evaluator()->Q();
    for (int col = 0; col < Q.cols(); ++col) {
      for (int row = 0; (row <= col) && (row < Q.rows()); ++row) {
        const double value = Q(row, col);
        const int x_row = x_indices[row];
        const int x_col = x_indices[col];
        P_triplets.emplace_back(x_row, x_col, static_cast<c_float>(value));
      }
    }

    // Add quadratic_cost.b to the linear cost term q.
    for (int i = 0; i < x.rows(); ++i) {
      q->at(x_indices[i]) += quadratic_cost.evaluator()->b()(i);
    }

    // Add quadratic_cost.c to constant term
    *constant_cost_term += quadratic_cost.evaluator()->c();
  }

  // Scale the matrix P in the cost.
  // Note that the linear term is scaled in ParseLinearCosts().
  const auto& scale_map = prog.GetVariableScaling();
  if (!scale_map.empty()) {
    for (auto& triplet : P_triplets) {
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

  P->resize(prog.num_vars(), prog.num_vars());
  P->setFromTriplets(P_triplets.begin(), P_triplets.end());
}

void ParseLinearCosts(const MathematicalProgram& prog, std::vector<c_float>* q,
                      double* constant_cost_term) {
  // Add the linear costs to the osqp cost.
  DRAKE_ASSERT(static_cast<int>(q->size()) == prog.num_vars());

  // Loop over the linear costs stored inside prog.
  for (const auto& linear_cost : prog.linear_costs()) {
    for (int i = 0; i < static_cast<int>(linear_cost.GetNumElements()); ++i) {
      // Append the linear cost term to q.
      if (linear_cost.evaluator()->a()(i) != 0) {
        const int x_index =
            prog.FindDecisionVariableIndex(linear_cost.variables()(i));
        q->at(x_index) += linear_cost.evaluator()->a()(i);
      }
    }
    // Add the constant cost term to constant_cost_term.
    *constant_cost_term += linear_cost.evaluator()->b();
  }

  // Scale the vector q in the cost.
  const auto& scale_map = prog.GetVariableScaling();
  if (!scale_map.empty()) {
    for (const auto& [index, scale] : scale_map) {
      q->at(index) *= scale;
    }
  }
}

// OSQP defines its own infinity in osqp/include/glob_opts.h.
c_float ConvertInfinity(double val) {
  if (std::isinf(val)) {
    if (val > 0) {
      return OSQP_INFTY;
    }
    return -OSQP_INFTY;
  }
  return static_cast<c_float>(val);
}

// Will call this function to parse both LinearConstraint and
// LinearEqualityConstraint.
template <typename C>
void ParseLinearConstraints(
    const MathematicalProgram& prog,
    const std::vector<Binding<C>>& linear_constraints,
    std::vector<Eigen::Triplet<c_float>>* A_triplets, std::vector<c_float>* l,
    std::vector<c_float>* u, int* num_A_rows,
    std::unordered_map<Binding<Constraint>, int>* constraint_start_row) {
  // Loop over the linear constraints, stack them to get l, u and A.
  for (const auto& constraint : linear_constraints) {
    const std::vector<int> x_indices =
        prog.FindDecisionVariableIndices(constraint.variables());
    const std::vector<Eigen::Triplet<double>> Ai_triplets =
        SparseOrDenseMatrixToTriplets(constraint.evaluator()->GetDenseA());
    const Binding<Constraint> constraint_cast =
        BindingDynamicCast<Constraint>(constraint);
    constraint_start_row->emplace(constraint_cast, *num_A_rows);
    // Append constraint.A to osqp A.
    for (const auto& Ai_triplet : Ai_triplets) {
      A_triplets->emplace_back(*num_A_rows + Ai_triplet.row(),
                               x_indices[Ai_triplet.col()],
                               static_cast<c_float>(Ai_triplet.value()));
    }
    const int num_Ai_rows = constraint.evaluator()->num_constraints();
    l->reserve(l->size() + num_Ai_rows);
    u->reserve(u->size() + num_Ai_rows);
    for (int i = 0; i < num_Ai_rows; ++i) {
      l->push_back(ConvertInfinity(constraint.evaluator()->lower_bound()(i)));
      u->push_back(ConvertInfinity(constraint.evaluator()->upper_bound()(i)));
    }
    *num_A_rows += num_Ai_rows;
  }
}

void ParseBoundingBoxConstraints(
    const MathematicalProgram& prog,
    std::vector<Eigen::Triplet<c_float>>* A_triplets, std::vector<c_float>* l,
    std::vector<c_float>* u, int* num_A_rows,
    std::unordered_map<Binding<Constraint>, int>* constraint_start_row) {
  // Loop over the linear constraints, stack them to get l, u and A.
  for (const auto& constraint : prog.bounding_box_constraints()) {
    const Binding<Constraint> constraint_cast =
        BindingDynamicCast<Constraint>(constraint);
    constraint_start_row->emplace(constraint_cast, *num_A_rows);
    // Append constraint.A to osqp A.
    for (int i = 0; i < static_cast<int>(constraint.GetNumElements()); ++i) {
      A_triplets->emplace_back(
          *num_A_rows + i,
          prog.FindDecisionVariableIndex(constraint.variables()(i)),
          static_cast<c_float>(1));
    }
    const int num_Ai_rows = constraint.evaluator()->num_constraints();
    l->reserve(l->size() + num_Ai_rows);
    u->reserve(u->size() + num_Ai_rows);
    for (int i = 0; i < num_Ai_rows; ++i) {
      l->push_back(ConvertInfinity(constraint.evaluator()->lower_bound()(i)));
      u->push_back(ConvertInfinity(constraint.evaluator()->upper_bound()(i)));
    }
    *num_A_rows += num_Ai_rows;
  }
}

void ParseAllLinearConstraints(
    const MathematicalProgram& prog,
    std::vector<Eigen::Triplet<c_float>>& A_triplets,
    Eigen::SparseMatrix<c_float>* A,
    std::vector<c_float>* l, std::vector<c_float>* u,
    std::unordered_map<Binding<Constraint>, int>* constraint_start_row) {

  A_triplets.clear();
  l->clear();
  u->clear();

  int num_A_rows = 0;
  ParseLinearConstraints(prog, prog.linear_constraints(), &A_triplets, l, u,
                         &num_A_rows, constraint_start_row);
  ParseLinearConstraints(prog, prog.linear_equality_constraints(), &A_triplets,
                         l, u, &num_A_rows, constraint_start_row);
  ParseBoundingBoxConstraints(prog, &A_triplets, l, u, &num_A_rows,
                              constraint_start_row);

  // Scale the matrix A.
  // Note that we only scale the columns of A, because the constraint has the
  // form l <= Ax <= u where the scaling of x enters the columns of A instead of
  // rows of A.
  const auto& scale_map = prog.GetVariableScaling();
  if (!scale_map.empty()) {
    for (auto& triplet : A_triplets) {
      auto column = scale_map.find(triplet.col());
      if (column != scale_map.end()) {
        triplet = Eigen::Triplet<double>(triplet.row(), triplet.col(),
                                         triplet.value() * (column->second));
      }
    }
  }

  A->resize(num_A_rows, prog.num_vars());
  A->setFromTriplets(A_triplets.begin(), A_triplets.end());
}

// Convert an Eigen::SparseMatrix to csc_matrix, to be used by osqp.
// Make sure the input Eigen sparse matrix is compressed, by calling
// makeCompressed() function.
// The caller of this function is responsible for freeing the memory allocated
// here.
csc* EigenSparseToCSC(const Eigen::SparseMatrix<c_float>& mat) {
  // A csc matrix is in the compressed column major.
  c_float* values =
      static_cast<c_float*>(c_malloc(sizeof(c_float) * mat.nonZeros()));
  c_int* inner_indices =
      static_cast<c_int*>(c_malloc(sizeof(c_int) * mat.nonZeros()));
  c_int* outer_indices =
      static_cast<c_int*>(c_malloc(sizeof(c_int) * (mat.cols() + 1)));
  for (int i = 0; i < mat.nonZeros(); ++i) {
    values[i] = *(mat.valuePtr() + i);
    inner_indices[i] = static_cast<c_int>(*(mat.innerIndexPtr() + i));
  }
  for (int i = 0; i < mat.cols() + 1; ++i) {
    outer_indices[i] = static_cast<c_int>(*(mat.outerIndexPtr() + i));
  }
  return csc_matrix(mat.rows(), mat.cols(), mat.nonZeros(), values,
                    inner_indices, outer_indices);
}

void UpdateCSCFromEigenSparse(
    const Eigen::SparseMatrix<c_float>& mat_from, csc* mat_to)  {
  DRAKE_DEMAND(mat_to != nullptr);

  for (int i = 0; i < mat_from.nonZeros(); i++) {
    mat_to->x[i] = *(mat_from.valuePtr() + i);
    mat_to->i[i] = static_cast<c_int>(*(mat_from.innerIndexPtr() + i));
  }
  for (int i = 0; i < mat_from.cols() + 1; i++) {
    mat_to->p[i] = static_cast<c_int>(*(mat_from.outerIndexPtr() + i));
  }
}

template <typename T1, typename T2>
void SetFastOsqpSolverSetting(
    const std::unordered_map<std::string, T1>& options,
    const std::string& option_name, T2* osqp_setting_field) {
  const auto it = options.find(option_name);
  if (it != options.end()) {
    *osqp_setting_field = it->second;
  }
}

template <typename T1, typename T2>
void SetFastOsqpSolverSettingWithDefaultValue(
    const std::unordered_map<std::string, T1>& options,
    const std::string& option_name, T2* osqp_setting_field,
    const T1& default_field_value) {
  const auto it = options.find(option_name);
  if (it != options.end()) {
    *osqp_setting_field = it->second;
  } else {
    *osqp_setting_field = default_field_value;
  }
}

void SetFastOsqpSolverSettings(const SolverOptions& solver_options,
                               OSQPSettings* settings) {
  const std::unordered_map<std::string, double>& options_double =
      solver_options.GetOptionsDouble(OsqpSolver::id());
  const std::unordered_map<std::string, int>& options_int =
      solver_options.GetOptionsInt(OsqpSolver::id());
  SetFastOsqpSolverSetting(options_double, "rho", &(settings->rho));
  SetFastOsqpSolverSetting(options_double, "sigma", &(settings->sigma));
  SetFastOsqpSolverSetting(options_int, "max_iter", &(settings->max_iter));
  SetFastOsqpSolverSetting(options_double, "eps_abs", &(settings->eps_abs));
  SetFastOsqpSolverSetting(options_double, "eps_rel", &(settings->eps_rel));
  SetFastOsqpSolverSetting(options_double, "eps_prim_inf",
                           &(settings->eps_prim_inf));
  SetFastOsqpSolverSetting(options_double, "eps_dual_inf",
                           &(settings->eps_dual_inf));
  SetFastOsqpSolverSetting(options_double, "alpha", &(settings->alpha));
  SetFastOsqpSolverSetting(options_double, "delta", &(settings->delta));
  // Default polish to true, to get an accurate solution.
  SetFastOsqpSolverSettingWithDefaultValue(options_int, "polish",
                                           &(settings->polish), 1);
  SetFastOsqpSolverSetting(options_int, "polish_refine_iter",
                           &(settings->polish_refine_iter));
  SetFastOsqpSolverSettingWithDefaultValue(options_int, "verbose",
                                           &(settings->verbose), 0);
  SetFastOsqpSolverSetting(options_int, "scaled_termination",
                           &(settings->scaled_termination));
  SetFastOsqpSolverSetting(options_int, "check_termination",
                           &(settings->check_termination));
  SetFastOsqpSolverSetting(options_int, "warm_start", &(settings->warm_start));
  SetFastOsqpSolverSetting(options_int, "scaling", &(settings->scaling));
  SetFastOsqpSolverSetting(options_int, "adaptive_rho",
                           &(settings->adaptive_rho));
  SetFastOsqpSolverSetting(options_double, "adaptive_rho_interval",
                           &(settings->adaptive_rho_interval));
  SetFastOsqpSolverSetting(options_double, "adaptive_rho_tolerance",
                           &(settings->adaptive_rho_tolerance));
  SetFastOsqpSolverSetting(options_double, "adaptive_rho_fraction",
                           &(settings->adaptive_rho_fraction));
  SetFastOsqpSolverSetting(options_double, "time_limit",
                           &(settings->time_limit));
}

template <typename C>
void SetDualSolution(
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
}  // namespace

bool FastOsqpSolver::is_available() { return true; }

void FastOsqpSolver::InitializeSolver(const MathematicalProgram& prog,
                                      const SolverOptions& solver_options) {
  // Get the cost for the QP.
   q_ = std::vector<c_float>(prog.num_vars(), 0);
  double constant_cost_term{0};

  ParseQuadraticCosts(prog, P_triplets_, &P_sparse_, &q_, &constant_cost_term);
  ParseLinearCosts(prog, &q_, &constant_cost_term);

  // linear_constraint_start_row[binding] stores the starting row index in A
  // corresponding to the linear constraint `binding`.
  std::unordered_map<Binding<Constraint>, int> constraint_start_row;

  // Parse the linear constraints.
  ParseAllLinearConstraints(
      prog, A_triplets_, &A_sparse_, &l_, &u_, &constraint_start_row);

  P_csc_ = EigenSparseToCSC(P_sparse_);
  A_csc_ = EigenSparseToCSC(A_sparse_);

  // Now pass the constraint and cost to osqp data.
  osqp_data_ = nullptr;

  // Populate data.
  osqp_data_ = static_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));

  osqp_data_->n = prog.num_vars();
  osqp_data_->m = A_sparse_.rows();
  osqp_data_->P = P_csc_;
  osqp_data_->q = q_.data();
  osqp_data_->A = A_csc_;
  osqp_data_->l = l_.data();
  osqp_data_->u = u_.data();

  osqp_settings_ = static_cast<OSQPSettings*>(c_malloc(sizeof(OSQPSettings)));
  osqp_set_default_settings(osqp_settings_);
  SetFastOsqpSolverSettings(solver_options, osqp_settings_);

  // Setup workspace.
  workspace_ = nullptr;
  const c_int osqp_setup_err =
      osqp_setup(&workspace_, osqp_data_, osqp_settings_);
  DRAKE_DEMAND(osqp_setup_err == 0);
  const c_int osqp_solve_err = osqp_solve(workspace_);

  is_init_ = true;
}

void FastOsqpSolver::WarmStart(const Eigen::VectorXd& primal,
                               const Eigen::VectorXd& dual) {
  std::vector<c_float> x, y;
  x.reserve(primal.size());
  y.reserve(dual.size());
  for (int i = 0; i < primal.size(); ++i) {
    x.push_back(ConvertInfinity(primal(i)));
  }
  for (int i = 0; i < dual.size(); ++i) {
    y.push_back(ConvertInfinity(dual(i)));
  }
  osqp_warm_start(workspace_, x.data(), y.data());
}

void FastOsqpSolver::DoSolve(const MathematicalProgram& prog,
                             const Eigen::VectorXd& initial_guess,
                             const SolverOptions& merged_options,
                             MathematicalProgramResult* result) const {
  auto& solver_details =
      result->SetSolverDetailsType<OsqpSolverDetails>();

  // OSQP solves a convex quadratic programming problem
  // min 0.5 xᵀPx + qᵀx
  // s.t l ≤ Ax ≤ u
  // OSQP is written in C, so this function will be in C style.

  // clear the vectors for cost and constraint bounds
  std::fill(q_.begin(), q_.end(), 0);
  l_.clear();
  u_.clear();

  double constant_cost_term{0};

  ParseQuadraticCosts(prog, P_triplets_, &P_sparse_, &q_, &constant_cost_term);
  ParseLinearCosts(prog, &q_, &constant_cost_term);

  // linear_constraint_start_row[binding] stores the starting row index in A
  // corresponding to the linear constraint `binding`.
  std::unordered_map<Binding<Constraint>, int> constraint_start_row;

  // Parse the linear constraints.
  ParseAllLinearConstraints(
      prog, A_triplets_, &A_sparse_, &l_, &u_, &constraint_start_row);

  UpdateCSCFromEigenSparse(P_sparse_, P_csc_);
  UpdateCSCFromEigenSparse(A_sparse_, A_csc_);

  osqp_update_lin_cost(workspace_, q_.data());
  osqp_update_bounds(workspace_, l_.data(), u_.data());
  osqp_update_P_A(workspace_, P_csc_->x, OSQP_NULL, P_csc_->nzmax, A_csc_->x,
                  OSQP_NULL, A_csc_->nzmax);

  // If any step fails, it will set the solution_result and skip other steps.
  std::optional<SolutionResult> solution_result;

  // Solve problem.
  if (!solution_result) {
    DRAKE_THROW_UNLESS(workspace_ != nullptr);
    const c_int osqp_solve_err = osqp_solve(workspace_);
    DisableWarmStart(); // will only be re-enabled if the solve was successful
    if (osqp_solve_err != 0) {
      solution_result = SolutionResult::kInvalidInput;
    }
  }

  // Extract results.
  if (!solution_result) {
    DRAKE_THROW_UNLESS(workspace_->info != nullptr);

    solver_details.iter = workspace_->info->iter;
    solver_details.status_val = workspace_->info->status_val;
    solver_details.primal_res = workspace_->info->pri_res;
    solver_details.dual_res = workspace_->info->dua_res;
    solver_details.setup_time = workspace_->info->setup_time;
    solver_details.solve_time = workspace_->info->solve_time;
    solver_details.polish_time = workspace_->info->polish_time;
    solver_details.run_time = workspace_->info->run_time;

    switch (workspace_->info->status_val) {
      case OSQP_SOLVED:
        this->EnableWarmStart();
      case OSQP_SOLVED_INACCURATE: {
        const Eigen::Map<Eigen::Matrix<c_float, Eigen::Dynamic, 1>> osqp_sol(
            workspace_->solution->x, prog.num_vars());

        // Scale solution back if `scale_map` is not empty.
        const auto& scale_map = prog.GetVariableScaling();
        if (!scale_map.empty()) {
          drake::VectorX<double> scaled_sol = osqp_sol.cast<double>();
          for (const auto& [index, scale] : scale_map) {
            scaled_sol(index) *= scale;
          }
          result->set_x_val(scaled_sol);
        } else {
          result->set_x_val(osqp_sol.cast<double>());
        }

        result->set_optimal_cost(workspace_->info->obj_val +
                                 constant_cost_term);
        solver_details.y = Eigen::Map<Eigen::VectorXd>(workspace_->solution->y,
                                                       workspace_->data->m);
        solution_result = SolutionResult::kSolutionFound;
        SetDualSolution(prog.linear_constraints(), solver_details.y,
                        constraint_start_row, result);
        SetDualSolution(prog.linear_equality_constraints(), solver_details.y,
                        constraint_start_row, result);
        SetDualSolution(prog.bounding_box_constraints(), solver_details.y,
                        constraint_start_row, result);

        break;
      }
      case OSQP_PRIMAL_INFEASIBLE:
      case OSQP_PRIMAL_INFEASIBLE_INACCURATE: {
        solution_result = SolutionResult::kInfeasibleConstraints;
        result->set_optimal_cost(MathematicalProgram::kGlobalInfeasibleCost);
        break;
      }
      case OSQP_DUAL_INFEASIBLE:
      case OSQP_DUAL_INFEASIBLE_INACCURATE: {
        solution_result = SolutionResult::kDualInfeasible;
        break;
      }
      case OSQP_MAX_ITER_REACHED: {
        solution_result = SolutionResult::kIterationLimit;
        break;
      }
      default: {
        solution_result = SolutionResult::kSolverSpecificError;
        break;
      }
    }
  }
  result->set_solution_result(solution_result.value());

//  c_free(P_csc->x);
//  c_free(P_csc->i);
//  c_free(P_csc->p);
//  c_free(P_csc);
//  c_free(A_csc->x);
//  c_free(A_csc->i);
//  c_free(A_csc->p);
//  c_free(A_csc);
}

}  // namespace solvers
}  // namespace dairlib
