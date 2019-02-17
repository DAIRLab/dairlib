#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/decision_variable.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"

namespace dairlib {
namespace systems {
namespace trajectory_optimization {

void checkConstraints(const drake::solvers::MathematicalProgram* prog);

Eigen::VectorXd getConstraintRows(
    const drake::solvers::MathematicalProgram* prog,
    drake::solvers::Binding<drake::solvers::Constraint>& c);

// Evaluate all constraints and construct a linearization of them
void linearizeConstraints(const drake::solvers::MathematicalProgram* prog,
                          Eigen::VectorXd& x, Eigen::VectorXd& y,
                          Eigen::MatrixXd& A, Eigen::VectorXd& lb,
                          Eigen::VectorXd& ub);

double secondOrderCost(const drake::solvers::MathematicalProgram* prog,
                       Eigen::VectorXd& x, Eigen::MatrixXd& Q,
                       Eigen::VectorXd& w);

template <typename Derived>
int countConstraints(const drake::solvers::MathematicalProgram* prog,
    const std::vector<drake::solvers::Binding<Derived>>& constraints);

template <typename Derived>
int updateConstraints(const drake::solvers::MathematicalProgram* prog,
    const std::vector<drake::solvers::Binding<Derived>>& constraints,
    Eigen::VectorXd& x, Eigen::VectorXd& y, Eigen::MatrixXd& A,
    Eigen::VectorXd& lb, Eigen::VectorXd& ub, int constraint_index);

}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace dairlib
