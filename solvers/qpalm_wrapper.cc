#include "qpalm_wrapper.h"
#include "qpalm.h"

// QPalm defines a mod macro that interferes with fmt
#ifdef mod
#undef mod
#endif

#include <vector>
#include <unordered_map>

#include "drake/math/eigen_sparse_triplet.h"
#include "drake/solvers/osqp_solver.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mathematical_program_result.h"



using drake::solvers::Binding;
using drake::solvers::Constraint;
using drake::solvers::OsqpSolver;
using drake::solvers::MathematicalProgram;
using drake::solvers::MathematicalProgramResult;
using drake::solvers::SolutionResult;
using drake::solvers::SolverOptions;
using drake::solvers::VectorXDecisionVariable;

namespace dairlib {
namespace solvers {

namespace {

template<typename T1, typename T2>
void SetQpalmSetting(
    const std::unordered_map<std::string, T1>& options,
    const std::string& option_name, T2* qpalm_setting_field) {
  const auto it = options.find(option_name);
  if (it != options.end()) {
    *qpalm_setting_field = it->second;
  }
}

void SetQpalmSettings(const SolverOptions& solver_options,
                      QPALMSettings* settings) {
  const std::unordered_map<std::string, double>& options_double =
      solver_options.GetOptionsDouble(OsqpSolver::id());
  const std::unordered_map<std::string, int>& options_int =
      solver_options.GetOptionsInt(OsqpSolver::id());

  SetQpalmSetting(options_double, "eps_abs", &(settings->eps_abs));
  SetQpalmSetting(options_double, "eps_rel", &(settings->eps_rel));
  SetQpalmSetting(options_double, "eps_prim_inf", &(settings->eps_prim_inf));
  SetQpalmSetting(options_double, "eps_dual_inf", &(settings->eps_dual_inf));
  SetQpalmSetting(options_int, "max_iter", &(settings->max_iter));
  SetQpalmSetting(options_double, "gamma_init", &(settings->gamma_init));
  SetQpalmSetting(options_double, "gamma_upd", &(settings->gamma_upd));
  SetQpalmSetting(options_double, "gamma_max", &(settings->gamma_max));
  SetQpalmSetting(options_int, "scaling", &(settings->scaling));
  SetQpalmSetting(options_int, "warm_start", &(settings->warm_start));
  SetQpalmSetting(options_int, "verbose", &(settings->verbose));
  SetQpalmSetting(options_double, "delta", &(settings->delta));
  SetQpalmSetting(options_double, "time_limit", &(settings->time_limit));
}

}  // namespace

QpalmWrapper::~QpalmWrapper() {
  FreeProblemData();
}

void QpalmWrapper::FreeProblemData() {
  if (qpalm_work_ != nullptr) {
    qpalm_cleanup(qpalm_work_);
    qpalm_work_ = nullptr;
  }
  if (qpalm_data_ != nullptr) {
    ladel_sparse_free(qpalm_data_->Q);
    ladel_sparse_free(qpalm_data_->A);
    qpalm_free(qpalm_data_);
    qpalm_data_ = nullptr;
  }
  P_csc_ = nullptr;
  A_csc_ = nullptr;

  if (qpalm_settings_ != nullptr) {
    qpalm_free(qpalm_settings_);
    qpalm_settings_ = nullptr;
  }
}

void QpalmWrapper::InitializeSolver(
    QPData& qp, const drake::solvers::SolverOptions& solver_options) {

  FreeProblemData();

  qpalm::sparse_mat_t qp_H = qp.H;
  qpalm::sparse_mat_t qp_A = qp.A;

  // Convert matrices to CSC format
  P_csc_ = qpalm::eigen_to_ladel_copy(
      qp_H.triangularView<Eigen::Upper>(), UPPER);
  A_csc_ = qpalm::eigen_to_ladel_copy(qp_A, UNSYMMETRIC);

  // Initialize QPALM data structure
  qpalm_data_ = static_cast<QPALMData*>(qpalm_malloc(sizeof(QPALMData)));

  qpalm_data_->n = qp.num_vars;
  qpalm_data_->m = qp.num_ineq;
  qpalm_data_->Q = P_csc_.get();
  qpalm_data_->q = qp.g.data();
  qpalm_data_->A = A_csc_.get();
  qpalm_data_->bmin = qp.lb.data();
  qpalm_data_->bmax = qp.ub.data();

  // Initialize settings
  qpalm_settings_ = static_cast<QPALMSettings*>(qpalm_malloc(sizeof(QPALMSettings)));
  qpalm_set_default_settings(qpalm_settings_);
  SetQpalmSettings(solver_options, qpalm_settings_);

  // Setup workspace
  qpalm_work_ = qpalm_setup(qpalm_data_, qpalm_settings_);

  DRAKE_DEMAND(qpalm_work_ != nullptr);

  is_init_ = true;
}

void QpalmWrapper::WarmStart(const Eigen::VectorXd& primal,
                             const Eigen::VectorXd& dual) {
  std::vector<c_float> x, y;
  x.reserve(primal.size());
  y.reserve(dual.size());

  for (int i = 0; i < primal.size(); ++i) {
    x.push_back(primal(i));
  }
  for (int i = 0; i < dual.size(); ++i) {
    y.push_back(dual(i));
  }

  qpalm_warm_start(qpalm_work_, x.data(), y.data());
}

void QpalmWrapper::Solve(QPData& qp, QPResult& result,
                         bool has_matrix_update) const {
  DRAKE_DEMAND(is_init_);

  if (has_matrix_update) {
    qpalm::sparse_mat_t qp_H = qp.H;
    qpalm::sparse_mat_t qp_A = qp.A;

    // Convert matrices to CSC format
    P_csc_ = qpalm::eigen_to_ladel_copy(
        qp_H.triangularView<Eigen::Upper>(), UPPER);
    A_csc_ = qpalm::eigen_to_ladel_copy(qp_A, UNSYMMETRIC);
    qpalm_update_Q_A(qpalm_work_, P_csc_->x, A_csc_->x);
  }

  qpalm_update_q(qpalm_work_, qp.g.data());
  qpalm_update_bounds(qpalm_work_, qp.lb.data(), qp.ub.data());

  DisableWarmStart();  // will only be re-enabled if the solve was successful

  // Solve the problem
  qpalm_solve(qpalm_work_);

  // Store results
  result.primal_res = qpalm_work_->info->pri_res_norm;
  result.dual_res = qpalm_work_->info->dua_res_norm;

  switch (qpalm_work_->info->status_val) {
    case QPALM_SOLVED: {
      EnableWarmStart();
      result.x = Eigen::Map<Eigen::VectorXd>(qpalm_work_->solution->x, qp.num_vars);
      result.y = Eigen::Map<Eigen::VectorXd>(qpalm_work_->solution->y, qp.num_ineq);
      result.objective = qpalm_work_->info->objective + qp.c;
      result.solution_result = SolutionResult::kSolutionFound;
      result.success = true;
      break;
    }
    case QPALM_PRIMAL_INFEASIBLE: {
      result.solution_result = SolutionResult::kInfeasibleConstraints;
      result.success = false;
      break;
    }
    case QPALM_DUAL_INFEASIBLE: {
      result.solution_result = SolutionResult::kDualInfeasible;
      result.success = false;
      break;
    }
    case QPALM_MAX_ITER_REACHED: {
      EnableWarmStart();
      result.x = Eigen::Map<Eigen::VectorXd>(qpalm_work_->solution->x, qp.num_vars);
      result.y = Eigen::Map<Eigen::VectorXd>(qpalm_work_->solution->y, qp.num_ineq);
      result.objective = qpalm_work_->info->objective + qp.c;
      result.solution_result = SolutionResult::kIterationLimit;
      result.success = true;
      break;
    }
    default: {
      throw std::runtime_error("Undefined QPALM return status " +
          std::to_string(qpalm_work_->info->status_val));
    }
  }
}

MathematicalProgramResult QpalmWrapper::Solve(
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