#include "solvers/optimization_utils.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using drake::solvers::Constraint;
using drake::solvers::Binding;
using drake::solvers::MathematicalProgram;
using drake::AutoDiffVecXd;
using drake::math::initializeAutoDiff;
using drake::math::autoDiffToGradientMatrix;
using drake::math::autoDiffToValueMatrix;

namespace dairlib {
namespace solvers {

void CheckGenericConstraints(const MathematicalProgram& prog,
    const drake::solvers::MathematicalProgramResult& result,
    double tol) {
  for (auto const& binding : prog.generic_constraints()) {
    auto y = result.EvalBinding(binding);
    auto c = binding.evaluator();
    bool isSatisfied = (y.array() >= c->lower_bound().array() - tol).all() &&
                       (y.array() <= c->upper_bound().array() + tol).all();
    if (!isSatisfied) {
      std::cout << "Constraint violation: " <<
          c->get_description() << std::endl;
      MatrixXd tmp(y.size(), 3);
      tmp << c->lower_bound(), y, c->upper_bound();
      std::cout << tmp << std::endl;
    }
  }
}

double SecondOrderCost(const MathematicalProgram& prog, const VectorXd& x_nom,
    MatrixXd* Q, VectorXd* w, double eps) {

  int num_vars = prog.num_vars();
  *Q = Eigen::MatrixXd::Zero(num_vars, num_vars);
  *w = Eigen::MatrixXd::Zero(num_vars, 1);

  double c = 0;

  for (auto const& binding : prog.GetAllCosts()) {
    // evaluate cost
    auto variables = binding.variables();
    if (variables.size() == 0)
      continue;
    AutoDiffVecXd y_val =
        initializeAutoDiff(VectorXd::Zero(1), variables.size());
    VectorXd x_binding(variables.size());
    for (int i = 0; i < variables.size(); i++) {
      x_binding(i) = x_nom(prog.FindDecisionVariableIndex(variables(i)));
    }
    AutoDiffVecXd x_val = initializeAutoDiff(x_binding);
    binding.evaluator()->Eval(x_val, &y_val);
    MatrixXd gradient_x = autoDiffToGradientMatrix(y_val);
    VectorXd y = autoDiffToValueMatrix(y_val);
    c += y(0);  // costs are length 1
    for (int i = 0; i < variables.size(); i++) {
      (*w)(prog.FindDecisionVariableIndex(variables(i))) += gradient_x(0, i);
    }

    // forward differencing for Hessian
    AutoDiffVecXd y_hessian =
        initializeAutoDiff(VectorXd::Zero(1), variables.size());
    for (int i = 0; i < variables.size(); i++) {
      x_val(i) += eps;
      binding.evaluator()->Eval(x_val, &y_hessian);
      x_val(i) -= eps;
      MatrixXd gradient_hessian = autoDiffToGradientMatrix(y_hessian);
      for (int j = 0; j <= i; j++) {
        int ind_i = prog.FindDecisionVariableIndex(variables(i));
        int ind_j = prog.FindDecisionVariableIndex(variables(j));
        (*Q)(ind_i, ind_j) += (gradient_hessian(0, j) - gradient_x(0, j)) / eps;
        if (ind_i != ind_j) {
          (*Q)(ind_j, ind_i) +=
              (gradient_hessian(0, j) - gradient_x(0, j)) / eps;
        }
      }
    }
  }

  return c;
}


// Evaluate all constraints and construct a linearization of them
void LinearizeConstraints(const MathematicalProgram& prog, const VectorXd& x,
    VectorXd* y, MatrixXd* A, VectorXd* lb, VectorXd* ub) {

  int num_constraints = 0;
  int num_vars = prog.num_vars();

  // First, count constraints
  num_constraints += CountConstraintRows(prog);

  // Initialize data storage
  lb->resize(num_constraints);
  ub->resize(num_constraints);
  y->resize(num_constraints);
  *A = Eigen::MatrixXd::Zero(num_constraints, num_vars);

  int constraint_index = 0;
  auto constraints = prog.GetAllConstraints();

  for (auto const& binding : constraints) {
    auto const& c = binding.evaluator();
    int n = c->num_constraints();
    lb->segment(constraint_index, n) = c->lower_bound();
    ub->segment(constraint_index, n) = c->upper_bound();

    auto variables = binding.variables();

    // Initialize AutoDiff vector for result
    AutoDiffVecXd y_val = initializeAutoDiff(
        VectorXd::Zero(c->num_constraints()), variables.size());

    // Extract subset of decision variable vector
    VectorXd x_binding(variables.size());
    for (int i = 0; i < variables.size(); i++) {
      x_binding(i) = x(prog.FindDecisionVariableIndex(variables(i)));
    }
    AutoDiffVecXd x_val = initializeAutoDiff(x_binding);

    // Evaluate constraint and extract gradient
    binding.evaluator()->Eval(x_val, &y_val);
    MatrixXd dx = autoDiffToGradientMatrix(y_val);

    y->segment(constraint_index, n) = autoDiffToValueMatrix(y_val);
    for (int i = 0; i < variables.size(); i++) {
      A->block(constraint_index,
          prog.FindDecisionVariableIndex(variables(i)), n, 1) = dx.col(i);
    }

    constraint_index += n;
  }
}

/// Helper method, returns a vector of given length
/// [start, start+1, ..., (start + length -1)]
VectorXd NVec(int start, int length) {
    VectorXd ret(length);
  for (int i = 0; i < length; i++) {
    ret(i) = i + start;
  }
  return ret;
}

VectorXd GetConstraintRows(const MathematicalProgram& prog,
    const Binding<Constraint>& c) {
  int n = 0;
  auto constraints = prog.GetAllConstraints();
  for (auto const& binding : constraints) {
    if (c.evaluator() == binding.evaluator() &&
        c.variables() == binding.variables()) {
    int num_constraints = c.evaluator()->num_constraints();
    return NVec(n, num_constraints);
    }
    n += binding.evaluator()->num_constraints();
  }

  return VectorXd(0);
}

int CountConstraintRows(const MathematicalProgram& prog) {
  int n = 0;
  auto constraints = prog.GetAllConstraints();
  for (auto const& binding : constraints) {
    n += binding.evaluator()->num_constraints();
  }
  return n;
}

}  // namespace solvers
}  // namespace dairlib
