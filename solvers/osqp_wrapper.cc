#include "osqp_wrapper.h"

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
// Convert an Eigen::SparseMatrix to csc_matrix, to be used by osqp.
// Make sure the input Eigen sparse matrix is compressed, by calling
// makeCompressed() function.
// The caller of this function is responsible for freeing the memory allocated
// here.
csc *EigenSparseToCSC(const Eigen::SparseMatrix<c_float> &mat) {
  // A csc matrix is in the compressed column major.
  c_float *values =
      static_cast<c_float *>(c_malloc(sizeof(c_float) * mat.nonZeros()));
  c_int *inner_indices =
      static_cast<c_int *>(c_malloc(sizeof(c_int) * mat.nonZeros()));
  c_int *outer_indices =
      static_cast<c_int *>(c_malloc(sizeof(c_int) * (mat.cols() + 1)));
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
    const Eigen::SparseMatrix<c_float> &mat_from, csc *mat_to) {
  DRAKE_DEMAND(mat_to != nullptr);

  for (int i = 0; i < mat_from.nonZeros(); i++) {
    mat_to->x[i] = *(mat_from.valuePtr() + i);
    mat_to->i[i] = static_cast<c_int>(*(mat_from.innerIndexPtr() + i));
  }
  for (int i = 0; i < mat_from.cols() + 1; i++) {
    mat_to->p[i] = static_cast<c_int>(*(mat_from.outerIndexPtr() + i));
  }
}

template<typename T1, typename T2>
void SetOsqpWrapperSetting(
    const std::unordered_map<std::string, T1> &options,
    const std::string &option_name, T2 *osqp_setting_field) {
  const auto it = options.find(option_name);
  if (it != options.end()) {
    *osqp_setting_field = it->second;
  }
}

template<typename T1, typename T2>
void SetOsqpWrapperSettingWithDefaultValue(
    const std::unordered_map<std::string, T1> &options,
    const std::string &option_name, T2 *osqp_setting_field,
    const T1 &default_field_value) {
  const auto it = options.find(option_name);
  if (it != options.end()) {
    *osqp_setting_field = it->second;
  } else {
    *osqp_setting_field = default_field_value;
  }
}

void SetOsqpWrapperSettings(const SolverOptions &solver_options,
                            OSQPSettings *settings) {
  const std::unordered_map<std::string, double> &options_double =
      solver_options.GetOptionsDouble(OsqpSolver::id());
  const std::unordered_map<std::string, int> &options_int =
      solver_options.GetOptionsInt(OsqpSolver::id());
  SetOsqpWrapperSetting(options_double, "rho", &(settings->rho));
  SetOsqpWrapperSetting(options_double, "sigma", &(settings->sigma));
  SetOsqpWrapperSetting(options_int, "max_iter", &(settings->max_iter));
  SetOsqpWrapperSetting(options_double, "eps_abs", &(settings->eps_abs));
  SetOsqpWrapperSetting(options_double, "eps_rel", &(settings->eps_rel));
  SetOsqpWrapperSetting(options_double, "eps_prim_inf",
                        &(settings->eps_prim_inf));
  SetOsqpWrapperSetting(options_double, "eps_dual_inf",
                        &(settings->eps_dual_inf));
  SetOsqpWrapperSetting(options_double, "alpha", &(settings->alpha));
  SetOsqpWrapperSetting(options_double, "delta", &(settings->delta));
  // Default polish to true, to get an accurate solution.
  SetOsqpWrapperSettingWithDefaultValue(options_int, "polish",
                                        &(settings->polish), 1);
  SetOsqpWrapperSetting(options_int, "polish_refine_iter",
                        &(settings->polish_refine_iter));
  SetOsqpWrapperSettingWithDefaultValue(options_int, "verbose",
                                        &(settings->verbose), 0);
  SetOsqpWrapperSetting(options_int, "scaled_termination",
                        &(settings->scaled_termination));
  SetOsqpWrapperSetting(options_int, "check_termination",
                        &(settings->check_termination));
  SetOsqpWrapperSetting(options_int, "warm_start", &(settings->warm_start));
  SetOsqpWrapperSetting(options_int, "scaling", &(settings->scaling));
  SetOsqpWrapperSetting(options_int, "adaptive_rho",
                        &(settings->adaptive_rho));
  SetOsqpWrapperSetting(options_double, "adaptive_rho_interval",
                        &(settings->adaptive_rho_interval));
  SetOsqpWrapperSetting(options_double, "adaptive_rho_tolerance",
                        &(settings->adaptive_rho_tolerance));
  SetOsqpWrapperSetting(options_double, "adaptive_rho_fraction",
                        &(settings->adaptive_rho_fraction));
  SetOsqpWrapperSetting(options_double, "time_limit",
                        &(settings->time_limit));
}

c_float ConvertInfinity(double val) {
  if (std::isinf(val)) {
    if (val > 0) {
      return OSQP_INFTY;
    }
    return -OSQP_INFTY;
  }
  return static_cast<c_float>(val);
}

void ConvertInfinity(Eigen::VectorXd& v) {
  for (Eigen::Index i = 0; i < v.rows(); ++i) {
    v(i) = ConvertInfinity(v(i));
  }
}

}

OsqpWrapper::~OsqpWrapper() {
  FreeProblemData();
}

void OsqpWrapper::FreeProblemData() {
  if (workspace_ != nullptr) {
    osqp_cleanup(workspace_);
  }
  if (P_csc_ != nullptr) {
    c_free(P_csc_->x);
    c_free(P_csc_->i);
    c_free(P_csc_->p);
    c_free(P_csc_);
    P_csc_ = nullptr;
  }
  if (A_csc_ != nullptr) {
    c_free(A_csc_->x);
    c_free(A_csc_->i);
    c_free(A_csc_->p);
    c_free(A_csc_);
    A_csc_ = nullptr;
  }
  if (osqp_data_ != nullptr) {
    c_free(osqp_data_);
    osqp_data_ = nullptr;
  }
  if (osqp_settings_ != nullptr) {
    c_free(osqp_settings_);
    osqp_settings_ = nullptr;
  }
}

void OsqpWrapper::InitializeSolver(
    QPData& qp, const drake::solvers::SolverOptions& solver_options) {

  FreeProblemData();

  P_csc_ = EigenSparseToCSC(qp.H.triangularView<Eigen::Upper>());
  A_csc_ = EigenSparseToCSC(qp.A);

  // Now pass the constraint and cost to osqp data.
  osqp_data_ = nullptr;

  // Populate data.
  osqp_data_ = static_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));

  osqp_data_->n = qp.num_vars;
  osqp_data_->m = qp.num_ineq;
  osqp_data_->P = P_csc_;
  osqp_data_->q = qp.g.data();
  osqp_data_->A = A_csc_;
  osqp_data_->l = qp.lb.data();
  osqp_data_->u = qp.ub.data();

  osqp_settings_ = static_cast<OSQPSettings*>(c_malloc(sizeof(OSQPSettings)));
  osqp_set_default_settings(osqp_settings_);
  SetOsqpWrapperSettings(solver_options, osqp_settings_);

  // Setup workspace.
  workspace_ = nullptr;
  const c_int osqp_setup_err =
      osqp_setup(&workspace_, osqp_data_, osqp_settings_);

  if (osqp_setup_err != 0) {
    std::cout << "QP:\n" << qp << std::endl;
  }

  DRAKE_DEMAND(osqp_setup_err == 0);

  is_init_ = true;
}

void OsqpWrapper::WarmStart(const Eigen::VectorXd& primal,
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

void OsqpWrapper::Solve(dairlib::solvers::QPData &qp,
                        dairlib::solvers::QPResult &result,
                        bool has_matrix_update) const {

  DRAKE_DEMAND(is_init_);

  if (has_matrix_update) {
    UpdateCSCFromEigenSparse(qp.H.triangularView<Eigen::Upper>(), P_csc_);
    UpdateCSCFromEigenSparse(qp.A, A_csc_);
    osqp_update_P_A(workspace_, P_csc_->x, OSQP_NULL, P_csc_->nzmax, A_csc_->x,
                    OSQP_NULL, A_csc_->nzmax);
  }

  osqp_update_lin_cost(workspace_, qp.g.data());
  osqp_update_bounds(workspace_, qp.lb.data(), qp.ub.data());
  osqp_update_warm_start(workspace_, osqp_settings_->warm_start);

  // If any step fails, it will set the solution_result and skip other steps.
  std::optional<SolutionResult> solution_result;

  // Solve problem.
  const c_int osqp_solve_err = osqp_solve(workspace_);
  DisableWarmStart(); // will only be re-enabled if the solve was successful

  if (osqp_solve_err != 0) {
    result.solution_result = SolutionResult::kInvalidInput;
    std::cout << "OSQP ERROR: " << osqp_solve_err<< std::endl;
    return;
  }

  result.primal_res = workspace_->info->pri_res;
  result.dual_res = workspace_->info->dua_res;
  result.run_time = workspace_->info->run_time;

  switch (workspace_->info->status_val) {
    case OSQP_SOLVED:
      result.solution_result = SolutionResult::kSolutionFound;
    case OSQP_SOLVED_INACCURATE: {
      this->EnableWarmStart();
      result.x = Eigen::Map<Eigen::VectorXd>(workspace_->solution->x, qp.num_vars);
      result.y = Eigen::Map<Eigen::VectorXd>(workspace_->solution->y,
                                             workspace_->data->m);
      result.objective = workspace_->info->obj_val + qp.c;
      if (workspace_->info->status_val == OSQP_SOLVED_INACCURATE) {
        result.solution_result = SolutionResult::kSolutionFound;
      }
      result.success = true;
      break;
    }
    case OSQP_PRIMAL_INFEASIBLE:
    case OSQP_PRIMAL_INFEASIBLE_INACCURATE: {
      result.solution_result = SolutionResult::kInfeasibleConstraints;
      result.success = false;
      break;
    }
    case OSQP_DUAL_INFEASIBLE_INACCURATE:
    case OSQP_DUAL_INFEASIBLE: {
      this->DisableWarmStart();
      result.x = Eigen::Map<Eigen::VectorXd>(workspace_->solution->x, qp.num_vars);
      result.y = Eigen::Map<Eigen::VectorXd>(workspace_->solution->y,
                                              workspace_->data->m);
      result.solution_result = SolutionResult::kDualInfeasible;
      result.success = false;
      break;
    }
    case OSQP_MAX_ITER_REACHED: {
      this->EnableWarmStart();
      result.x = Eigen::Map<Eigen::VectorXd>(workspace_->solution->x, qp.num_vars);
      result.y = Eigen::Map<Eigen::VectorXd>(workspace_->solution->y,
                                             workspace_->data->m);
      result.objective = workspace_->info->obj_val + qp.c;
      result.solution_result = SolutionResult::kIterationLimit;
      result.success = true;
      break;
    }
    case OSQP_TIME_LIMIT_REACHED: {
      result.x = Eigen::Map<Eigen::VectorXd>(workspace_->solution->x, qp.num_vars);
      result.y = Eigen::Map<Eigen::VectorXd>(workspace_->solution->y,
                                             workspace_->data->m);
      result.solution_result = SolutionResult::kSolverSpecificError;
      result.success = false;
      break;
    }
    default: {
      throw std::runtime_error("undefined OSQP return status " +
                                std::to_string(workspace_->info->status_val));
      break;
    }
  }
}

MathematicalProgramResult OsqpWrapper::Solve(
    const MathematicalProgram& prog, bool has_matrix_update) const {
  QPData qp = QPData::ToQPData(prog);
  QPResult solution;
  this->Solve(qp, solution, has_matrix_update);
  MathematicalProgramResult result;
  result.set_decision_variable_index(prog.decision_variable_index());
  result.set_x_val(solution.x);
  result.set_solution_result(solution.solution_result);
  return result;
}

}  // namespace solvers
}  // namespace dairlib
