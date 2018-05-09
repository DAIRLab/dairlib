#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/decision_variable.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using drake::solvers::Constraint;
using drake::solvers::Binding;

namespace drake{
namespace systems {
namespace trajectory_optimization{
namespace dircon {

void checkConstraints(const solvers::MathematicalProgram* prog);

// Evaluate all constraints and construct a linearization of them
void linearizeConstraints(const solvers::MathematicalProgram* prog, VectorXd& x,
  VectorXd& y, MatrixXd& A, VectorXd& lb, VectorXd& ub);

double secondOrderCost(const solvers::MathematicalProgram* prog, VectorXd& x,
  MatrixXd& Q, VectorXd& w);

template <typename Derived>
int countConstraints(const solvers::MathematicalProgram* prog, const std::vector<Binding<Derived>>& constraints);

template <typename Derived>
int updateConstraints(const solvers::MathematicalProgram* prog, const std::vector<Binding<Derived>>& constraints,
      VectorXd& x, VectorXd& y, MatrixXd& A, VectorXd& lb, VectorXd& ub, int constraint_index);

}
}
}
}