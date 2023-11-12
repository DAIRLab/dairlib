#pragma once

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mathematical_program_result.h"
#include "drake/solvers/decision_variable.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"

namespace dairlib {
namespace solvers {

/// Check if the generic (nonlinear) constraints are satisfied up to the
/// specified tolerance.
/// Prints out any violating constraints
/// @return True if all generic constraints are satisfied
bool CheckGenericConstraints(const drake::solvers::MathematicalProgram& prog,
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

std::tuple<Eigen::MatrixXd, Eigen::VectorXd, Eigen::VectorXd>
GetBigMFormulation(const Eigen::MatrixXd& A, const Eigen::MatrixXd& b, double M);


drake::solvers::Binding<drake::solvers::LinearConstraint>
AddBigMInequalityConstraint(drake::solvers::MathematicalProgram& prog,
    const Eigen::MatrixXd& A, const Eigen::VectorXd& b,
    double M,
    const drake::solvers::VectorXDecisionVariable& x,
    const drake::solvers::DecisionVariable& z);

/// A utility for mixed integer programming,
/// Adds the big M formulation of the constraint (z == 1) implies Ax <= b,
/// transforming it to Ax <= b + M * (1 - z)
/// @param prog The MathematicalProgram
/// @param A the constraint matrix
/// @param b the constraint upper bound
/// @param M the big-M parameter
/// @param x continuous variable
/// @param z integer variable
class LinearBigMConstraint {
 public:
  LinearBigMConstraint(drake::solvers::MathematicalProgram& prog,
                       const Eigen::MatrixXd& A, const Eigen::VectorXd& b,
                       double M,
                       const drake::solvers::VectorXDecisionVariable& x,
                       const drake::solvers::DecisionVariable& z) :
      constraint_(AddBigMInequalityConstraint(prog, A, b, M, x, z)), M_(M),
      z_(z), x_(x){};

  /// Update the Constraint Matrices
  /// \param A the new constraint matrix
  /// \param b the ne constraint upper bound
  void UpdateCoefficients(const Eigen::MatrixXd& A, const Eigen::VectorXd& b) {
    auto [Ac, lb, ub] = GetBigMFormulation(A, b, M_);
    constraint_.evaluator()->UpdateCoefficients(Ac, lb, ub);
    active_ = true;
  }

  /// Make the constraint inactive (trivially satisfied for any x).
  /// Used for MIQPs with a variable number of BigM constraints.
  void deactivate() {
    if (active_) {
      constraint_.evaluator()->UpdateCoefficients(
        Eigen::RowVectorXd::Zero(x_.size() + 1),
        drake::Vector1d::Constant(1, -std::numeric_limits<double>::infinity()),
        drake::Vector1d::Constant(1, std::numeric_limits<double>::infinity()));
    }
    active_ = false;
  }

  /// Make the constraint inactive and set the initial guess for x to zero.
  /// Used for MIQPs with a variable number of BigM constraints.
  void deactivate(drake::solvers::MathematicalProgram& prog) {
    deactivate();
    prog.SetInitialGuess(x_, Eigen::VectorXd::Zero(x_.rows()));
    prog.SetInitialGuess(z_, 0);
  }

  /// Check whether the constraint would be satisfied if the binary variable
  /// were true, for a given value of x.
  /// \param x value of x to check.
  /// \param tol with which to check constraint satisfaction
  /// \return True if Ax <= b, otherwise False
  [[nodiscard]] bool CheckSatisfiedIfActive(const Eigen::VectorXd& x,
                                            double tol=1e-4) const {
    DRAKE_ASSERT(x.size() == x_.size());
    Eigen::VectorXd xtmp = Eigen::VectorXd::Zero(x_.rows() + 1);
    xtmp.head(x.rows()) = x;
    xtmp.tail<1>()(0) = 1;
    return constraint_.evaluator()->CheckSatisfied(xtmp, tol);
  }
 private:
  bool active_ = true;
  drake::solvers::Binding<drake::solvers::LinearConstraint> constraint_;
  const double M_;
  const drake::solvers::DecisionVariable& z_;
  const drake::solvers::VectorXDecisionVariable& x_;
};


/// A utility for mixed integer programming,
/// Adds the big M formulation of the constraint (z == 1) implies Ax == b,
/// transforming it to Ax <= b + M * (1 - z), -Ax <= -b + M * (1 - z)
/// @param prog The MathematicalProgram
/// @param A the constraint matrix
/// @param b the constraint upper bound
/// @param M the big-M parameter
/// @param x continuous variable
/// @param z integer variable
class LinearBigMEqualityConstraint {
 public:
  LinearBigMEqualityConstraint(
      drake::solvers::MathematicalProgram& prog,
      const Eigen::MatrixXd& A, const Eigen::VectorXd& b, double M,
      const drake::solvers::VectorXDecisionVariable& x,
      const drake::solvers::DecisionVariable& z) :
      upper_(prog, A, b, M, x, z),
      lower_(prog, -A, -b, M, x, z){};

  /// Update the Constraint Matrices
  /// \param A the new constraint matrix
  /// \param b the ne constraint upper bound
  void UpdateCoefficients(const Eigen::MatrixXd& A, const Eigen::VectorXd& b) {
    upper_.UpdateCoefficients(A, b);
    lower_.UpdateCoefficients(-A, -b);
  }

  /// Make the constraint inactive (trivially satisfied for any x).
  /// Used for MIQPs with a variable number of BigM constraints.
  void deactivate() {
    upper_.deactivate();
    lower_.deactivate();
  }

  /// Make the constraint inactive and set the initial guess for x to zero.
  /// Used for MIQPs with a variable number of BigM constraints.
  void deactivate(drake::solvers::MathematicalProgram& prog) {
    upper_.deactivate(prog);
    lower_.deactivate(prog);
  }
 private:
  LinearBigMConstraint upper_;
  LinearBigMConstraint lower_;
};

}  // namespace solvers
}  // namespace dairlib
