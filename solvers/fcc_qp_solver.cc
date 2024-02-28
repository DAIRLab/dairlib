#include "fcc_qp_solver.h"

namespace dairlib::solvers {

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::SparseMatrix;

using drake::solvers::OsqpSolver;
using drake::solvers::SolverOptions;
using drake::solvers::MathematicalProgram;
using drake::solvers::MathematicalProgramResult;
using drake::solvers::VectorXDecisionVariable;
using drake::solvers::LinearEqualityConstraint;

namespace {

std::tuple<MatrixXd, VectorXd, double> ParseQuadraticCosts(
    const MathematicalProgram &prog) {
  int n = prog.num_vars();
  MatrixXd Q = MatrixXd::Zero(n, n);
  VectorXd b = VectorXd::Zero(n);
  double d = 0;
  for (const auto &binding : prog.quadratic_costs()) {
    const auto &x = binding.variables();
    const auto indices = prog.FindDecisionVariableIndices(x);
    const auto &H = binding.evaluator()->Q();
    const auto &g = binding.evaluator()->b();
    for (int r = 0; r < H.rows(); ++r) {
      const int x_row = indices[r];
      b(x_row) += g(r);
      for (int c = 0; c < H.cols(); ++c) {
        const double value = H(r, c);
        const int x_col = indices[c];
        Q(x_row, x_col) += value;
      }
    }
    d += binding.evaluator()->c();
  }
  return std::tie(Q, b, d);
}

// Mostly copied from Drake equality_constrained_qp_solver
std::pair<MatrixXd, VectorXd> ParseLinearEqualityConstraints(
    const MathematicalProgram &prog) {

  // figure out how many constraint rows there are
  int rows = 0;
  for (auto const &binding : prog.linear_equality_constraints()) {
    rows += binding.evaluator()->get_sparse_A().rows();
  }
  int num_vars = prog.num_vars();
  MatrixXd A = MatrixXd::Zero(rows, num_vars);
  VectorXd b = VectorXd::Zero(rows);
  int constraint_index = 0;

  for (auto const &binding : prog.linear_equality_constraints()) {
    auto const &bc = binding.evaluator();
    size_t n = bc->get_sparse_A().rows();
    const auto &v = binding.variables();
    int num_var = v.rows();
    for (int i = 0; i < num_var; ++i) {
      const int variable_index = prog.FindDecisionVariableIndex(v(i));
      for (SparseMatrix<double>::InnerIterator it(bc->get_sparse_A(), i); it;
           ++it) {
        A(constraint_index + it.row(), variable_index) = it.value();
      }
    }
    b.segment(constraint_index, n) = bc->lower_bound().segment(0, n);
    constraint_index += n;
  }
  return {A, b};
}

std::pair<VectorXd, VectorXd> ParseBoundingBoxConstraints(
    const MathematicalProgram &prog) {
  VectorXd lb = VectorXd::Constant(prog.num_vars(),
                                   -std::numeric_limits<double>::infinity());
  VectorXd ub = VectorXd::Constant(prog.num_vars(),
                                   std::numeric_limits<double>::infinity());

  for (const auto &binding : prog.bounding_box_constraints()) {
    const auto &v = binding.variables();
    const auto &lb_v = binding.evaluator()->lower_bound();
    const auto &ub_v = binding.evaluator()->upper_bound();
    for (int i = 0; i < v.rows(); ++i) {
      int idx = prog.FindDecisionVariableIndex(v(i));
      lb(idx) = std::max(lb_v(i), lb(idx));
      ub(idx) = std::min(ub_v(i), ub(idx));
    }
  }
  return {lb, ub};
}

template <typename T>
bool has_option(const std::unordered_map<std::string, T> opts,
                const std::string& name) {
  return opts.find(name) != opts.end();
}

}

void FCCQPSolver::InitializeSolver(
    const MathematicalProgram& prog, const SolverOptions& options,
    int num_contact_force_vars, int contact_force_vars_start,
    const std::vector<double>& friction_coeffs) {

  int n = prog.num_vars();
  int n_eq = 0;
  for (auto const &binding : prog.linear_equality_constraints()) {
    n_eq += binding.evaluator()->get_sparse_A().rows();
  }

  DRAKE_DEMAND(num_contact_force_vars % 3 == 0);
  DRAKE_DEMAND(friction_coeffs.size() == num_contact_force_vars / 3);
  friction_coeffs_ = friction_coeffs;
  fcc_qp_ = std::make_unique<fcc_qp::FCCQP>(n, n_eq, num_contact_force_vars,
                                            contact_force_vars_start);

  const auto& double_opts = options.GetOptionsDouble(id());
  const auto& int_opts = options.GetOptionsInt(id());

  if (double_opts.empty() and int_opts.empty()) {
    if (not options.GetOptionsDouble(OsqpSolver::id()).empty() or
        not options.GetOptionsInt(OsqpSolver::id()).empty()) {
      std::cout << "Warning: Initializing FCCQP with solver options "
                   "containing only OSQP solver options. FCCQP will use "
                   "defaults\n";
    }
  }

  if (has_option(double_opts, "rho")) {
    fcc_qp_->set_rho(double_opts.at("rho"));
  }
  if (has_option(double_opts, "eps")) {
    fcc_qp_->set_eps(double_opts.at("eps"));
  }
  if (has_option(int_opts, "max_iter")) {
    fcc_qp_->set_max_iter(int_opts.at("max_iter"));
  }
}

FCCQPSolver::FCCQPSolver() : drake::solvers::SolverBase(
    id(), &is_available, &is_enabled, &ProgramAttributesSatisfied){}

FCCQPSolver::~FCCQPSolver() = default;

void FCCQPSolver::DoSolve(const MathematicalProgram& prog,
                          const Eigen::VectorXd& initial_guess,
                          const SolverOptions& merged_options,
                          MathematicalProgramResult* result) const {

  DRAKE_DEMAND(is_initialized());

  const auto &[Q, b, c] = ParseQuadraticCosts(prog);
  const auto &[Aeq, beq] = ParseLinearEqualityConstraints(prog);
  const auto &[lb, ub] = ParseBoundingBoxConstraints(prog);

  fcc_qp_->Solve(Q, b, Aeq, beq, friction_coeffs_, lb, ub);
  const auto &sol = fcc_qp_->GetSolution();

  auto &details_out = result->SetSolverDetailsType<Details>();
  details_out = sol.details;

  result->set_solution_result(drake::solvers::SolutionResult::kSolutionFound);
  double cost = 0.5 * sol.z.transpose() * Q * sol.z + sol.z.dot(b) + c;
  result->set_optimal_cost(cost);
  result->set_x_val(sol.z);
}
}