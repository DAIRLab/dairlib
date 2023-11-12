#include "solvers/optimization_utils.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using drake::solvers::Constraint;
using drake::solvers::Binding;
using drake::solvers::MathematicalProgram;
using drake::solvers::LinearConstraint;
using drake::solvers::VectorXDecisionVariable;
using drake::solvers::DecisionVariable;
using drake::AutoDiffVecXd;
using drake::math::InitializeAutoDiff;
using drake::math::ExtractGradient;
using drake::math::ExtractValue;

namespace dairlib {
namespace solvers {

bool CheckGenericConstraints(const MathematicalProgram& prog,
    const drake::solvers::MathematicalProgramResult& result,
    double tol) {
  bool allSatisfied = true;
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
      allSatisfied = false;
    }
  }
  return allSatisfied;
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
        InitializeAutoDiff(VectorXd::Zero(1), variables.size());
    VectorXd x_binding(variables.size());
    for (int i = 0; i < variables.size(); i++) {
      x_binding(i) = x_nom(prog.FindDecisionVariableIndex(variables(i)));
    }
    AutoDiffVecXd x_val = InitializeAutoDiff(x_binding);
    binding.evaluator()->Eval(x_val, &y_val);
    MatrixXd gradient_x = ExtractGradient(y_val);
    VectorXd y = ExtractValue(y_val);
    c += y(0);  // costs are length 1
    for (int i = 0; i < variables.size(); i++) {
      (*w)(prog.FindDecisionVariableIndex(variables(i))) += gradient_x(0, i);
    }

    // forward differencing for Hessian
    AutoDiffVecXd y_hessian =
        InitializeAutoDiff(VectorXd::Zero(1), variables.size());
    for (int i = 0; i < variables.size(); i++) {
      x_val(i) += eps;
      binding.evaluator()->Eval(x_val, &y_hessian);
      x_val(i) -= eps;
      MatrixXd gradient_hessian = ExtractGradient(y_hessian);
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
    AutoDiffVecXd y_val;

    // Extract subset of decision variable vector
    VectorXd x_binding(variables.size());
    for (int i = 0; i < variables.size(); i++) {
      x_binding(i) = x(prog.FindDecisionVariableIndex(variables(i)));
    }
    AutoDiffVecXd x_val = InitializeAutoDiff(x_binding);
    // Evaluate constraint and extract gradient
    binding.evaluator()->Eval(x_val, &y_val);
    MatrixXd dx = ExtractGradient(y_val);

    y->segment(constraint_index, n) = ExtractValue(y_val);
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
  for (const auto& binding : constraints) {
    n += binding.evaluator()->num_constraints();
  }
  return n;
}

std::tuple<MatrixXd, VectorXd, VectorXd>
GetBigMFormulation(const MatrixXd& A, const MatrixXd& b, double M) {
  MatrixXd Ac = MatrixXd::Zero(A.rows(), A.cols() + 1);
  Ac.leftCols(A.cols()) = A;
  Ac.rightCols<1>() = M * VectorXd::Ones(A.rows());
  VectorXd ub = b + M * VectorXd::Ones(b.size());
  VectorXd lb =
      -std::numeric_limits<double>::infinity() * VectorXd::Ones(b.size());
  return std::tie(Ac, lb, ub);
}

Binding<LinearConstraint> AddBigMInequalityConstraint(
    drake::solvers::MathematicalProgram& prog,
    const MatrixXd& A, const VectorXd& b, double M,
    const VectorXDecisionVariable& x, const DecisionVariable& z){
  auto [Ac, lb, ub] = GetBigMFormulation(A, b, M);
  return prog.AddLinearConstraint(Ac, lb, ub, {x, drake::Vector1<DecisionVariable>(z)});
}

void UpdateBigMInequalityConstraint(Binding<LinearConstraint>& binding,
                                    const MatrixXd& A, const MatrixXd& b,
                                    double M) {
  MatrixXd Ac = MatrixXd::Zero(A.rows(), A.cols() + 1);
  Ac.leftCols(A.cols()) = A;
  Ac.rightCols<1>() = M * VectorXd::Ones(A.rows());
  VectorXd ub = b + M * VectorXd::Ones(b.size());
  VectorXd lb = VectorXd::Constant(b.size(), -std::numeric_limits<double>::infinity());
  binding.evaluator()->UpdateCoefficients(Ac, lb, ub);
}

void print_constraint(
    const std::vector<drake::solvers::Binding<drake::solvers::LinearConstraint>>& constraints) {
  for (auto &constr : constraints) {
    std::cout << constr.evaluator()->get_description() << ":\n A:\n"
              << constr.evaluator()->get_sparse_A() << "\nub:\n"
              << constr.evaluator()->upper_bound() << "\nlb\n"
              << constr.evaluator()->lower_bound() << std::endl;
  }
}

void print_constraint(
    const std::vector<drake::solvers::Binding<drake::solvers::LinearEqualityConstraint>>& constraints) {
  for (auto &constr : constraints) {
    std::cout << constr.evaluator()->get_description() << ":\n A:\n"
              << constr.evaluator()->get_sparse_A() << "\nb:\n"
              << constr.evaluator()->upper_bound() << std::endl;
  }
}

void print_constraint(
    const std::vector<drake::solvers::Binding<drake::solvers::Constraint>>& constraints) {
  for (auto &constraint : constraints) {
    auto constr = dynamic_cast<drake::solvers::LinearConstraint*>(constraint.evaluator().get());
    std::cout << constr->get_description() << ":\n A:\n"
              << constr->get_sparse_A() << "\nb:\n"
              << constr->upper_bound() << std::endl;
  }
}

}  // namespace solvers
}  // namespace dairlib
