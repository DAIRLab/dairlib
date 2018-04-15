#include "dircon_util.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using drake::solvers::Constraint;
using drake::solvers::Binding;

namespace drake{
namespace systems {
namespace trajectory_optimization{
namespace dircon {

void checkConstraints(const solvers::MathematicalProgram* prog) {
  for (auto const& binding : prog->generic_constraints()) {
    double tol = 1e-6;
    auto y = prog->EvalBindingAtSolution(binding);
    auto c = binding.evaluator();
    bool isSatisfied = (y.array() >= c->lower_bound().array() - tol).all() &&
                       (y.array() <= c->upper_bound().array() + tol).all();
    if (!isSatisfied) {
      std::cout << "Constraint violation: " << c->get_description() << std::endl;
      MatrixXd tmp(y.size(),3);
      tmp << c->lower_bound(), y, c->upper_bound();
      std::cout << tmp << std::endl;
    }
  }
}


// Evaluate all constraints and construct a linearization of them
void linearizeConstraints(const solvers::MathematicalProgram* prog, VectorXd& x,
  VectorXd& y, MatrixXd& A, VectorXd& lb, VectorXd& ub) {


  int num_constraints = 0;
  int num_vars = prog->num_vars();

  // First, count constraints
  num_constraints += countConstraints(prog, prog->linear_constraints());
  num_constraints += countConstraints(prog, prog->linear_equality_constraints());
  num_constraints += countConstraints(prog, prog->lorentz_cone_constraints());
  num_constraints += countConstraints(prog, prog->generic_constraints());

  //Initialize data storage
  lb.resize(num_constraints);
  ub.resize(num_constraints);
  y.resize(num_constraints);
  A = Eigen::MatrixXd::Zero(num_constraints, num_vars);

  int constraint_index = 0;
  constraint_index = updateConstraints(prog, prog->linear_constraints(), x, y, A, lb, ub, constraint_index);
  constraint_index = updateConstraints(prog, prog->linear_equality_constraints(), x, y, A, lb, ub, constraint_index);
  constraint_index = updateConstraints(prog, prog->lorentz_cone_constraints(), x, y, A, lb, ub, constraint_index);
  constraint_index = updateConstraints(prog, prog->generic_constraints(), x, y, A, lb, ub, constraint_index);
}

template <typename Derived>
int countConstraints(const solvers::MathematicalProgram* prog, const std::vector<Binding<Derived>>& constraints) {
  int n = 0;
  for (auto const& binding : constraints) {
    n += binding.evaluator()->num_constraints();
  }
  return n;
}

template <typename Derived>
int updateConstraints(const solvers::MathematicalProgram* prog, const std::vector<Binding<Derived>>& constraints,
      VectorXd& x, VectorXd& y, MatrixXd& A, VectorXd& lb, VectorXd& ub, int constraint_index) {

  for (auto const& binding : constraints) {
    auto const& c = binding.evaluator();
    int n = c->num_constraints();
    lb.segment(constraint_index, n) = c->lower_bound();
    ub.segment(constraint_index, n) = c->upper_bound();

    //evaluate constraint
    auto variables = binding.variables();
    AutoDiffVecXd y_val = math::initializeAutoDiff(VectorXd::Zero(c->num_constraints()), variables.size());
    VectorXd x_binding(variables.size());
    for (int i=0; i < variables.size(); i++) {
      x_binding(i) = x(prog->FindDecisionVariableIndex(variables(i)));
    }
    AutoDiffVecXd x_val = math::initializeAutoDiff(x_binding);
    binding.evaluator()->Eval(x_val, y_val);
    MatrixXd dx = math::autoDiffToGradientMatrix(y_val);

    y.segment(constraint_index, n) = math::autoDiffToValueMatrix(y_val);
    for (int i = 0; i < variables.size(); i++) {
      A.block(constraint_index, prog->FindDecisionVariableIndex(variables(i)),n,1) = dx.col(i);
    }

    constraint_index += n;
  }

  return constraint_index;
}



}
}
}
}