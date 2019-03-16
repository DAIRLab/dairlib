#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/decision_variable.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"

namespace dairlib {
namespace solvers {

/// Check if the generic (nonlinear) constraints are satisfied up to the
/// specified tolerance.
/// Prints out any violating constraints
/// @return True if all generic constraints are satisfied
bool CheckConstraints(const drake::solvers::MathematicalProgram& prog,
    const drake::solvers::MathematicalProgramResult& result,
    double tol = 1e-6);

/// Given a MathematicalProgram and associated constraint Binding, returns
/// the vector of row indices associated with that constraint.
/// Note that an exact ordering of constraints is *not* inherant to
/// MathematicalProgram, and so these rows are to be associated with other
/// utility functions, like LinearizeConstraints.
Eigen::VectorXd GetConstraintRows(
    const drake::solvers::MathematicalProgram& prog,
    const drake::solvers::Binding<drake::solvers::Constraint>& c);

/// Evaluate all constraints, f(x) and construct a linearization of them
/// @param prog The MathematicalProgram
/// @param x The value of the decision paramters to linearize about
/// @param y A pointer to the constraint evaluation, y = f(x)
/// @param A A pointer to the gradient matrix, A = dy/dx
/// @param lb A pointer to the lower bound vector lb <= f(x)
/// @param ub A pointer to the upper bound vector ub >= f(x)
void LinearizeConstraints(const drake::solvers::MathematicalProgram& prog,
                          const Eigen::VectorXd& x, Eigen::VectorXd* y,
                          Eigen::MatrixXd* A, Eigen::VectorXd* lb,
                          Eigen::VectorXd* ub);

/// Form a second order approximation to the cost of an optimization program
/// about some nominal value
///
/// The cost is approximately
/// 1/2 (x - x_nom)^T Q (x - x_nom ) + w^T (x - x_nom) + constant
/// Uses (forward) numerical differencing to compute the Hessian
/// @param prog The MathematicalProgra
/// @param x_nom The nominal value of the decision paramters
/// @param Q A pointer to the quadratic part of the cost. Will set to be
///   symmetric
/// @param w A pointer to the linear part of hte cost
/// @param eps The epsilon to use for numerical differencing
/// @return Returns the constant part of the cost
double SecondOrderCost(const drake::solvers::MathematicalProgram& prog,
    const Eigen::VectorXd& x_nom, Eigen::MatrixXd* Q, Eigen::VectorXd* w,
    double eps = 1e-8);

/// Count the total number of constraint rows, if lb <= f(x) <= ub, this is
/// the dimension of f(x)
int CountConstraintRows(const drake::solvers::MathematicalProgram& prog);

}  // namespace solvers
}  // namespace dairlib
