#include "legendre_coefficient_variables.h"
#include "common/legendre.h"

namespace dairlib {
namespace solvers {

using drake::Vector1d;
using drake::solvers::MathematicalProgram;

using Eigen::VectorXd;

using polynomials::EvalLegendreBasisDerivative;
using polynomials::MakeCostMatrixForMinimizingPathDerivativeSquaredWithLegendreBasis;

LegendreCoefficientVariables::LegendreCoefficientVariables(
    MathematicalProgram &prog, int degree, std::string name) : degree_(degree) {
  DRAKE_DEMAND(degree > 0);
  DRAKE_DEMAND(not name.empty());
  c_ = prog.NewContinuousVariables(degree + 1, name);
}

void LegendreCoefficientVariables::AddDerivativeConstraint(
    MathematicalProgram& prog, std::string name, double time, int deriv_order,
    double value) {

  DRAKE_DEMAND(not my_equality_constraints_.contains({deriv_order, name}));
  DRAKE_DEMAND(deriv_order >= 0);
  DRAKE_DEMAND(-1 <= time and time <= 1);

  my_equality_constraints_[{deriv_order, name}] = prog.AddLinearEqualityConstraint(
      EvalLegendreBasisDerivative(degree_, deriv_order, time).transpose(),
      Vector1d::Constant(value), c_).evaluator();
}

void LegendreCoefficientVariables::UpdateDerivativeConstraint(
    std::string name, double time, int deriv_order, double value) {
  my_equality_constraints_.at({deriv_order, name})->UpdateCoefficients(
      EvalLegendreBasisDerivative(degree_, deriv_order, time).transpose(),
      Vector1d::Constant(value));
}

void LegendreCoefficientVariables::AddDerivativeSoftConstraint(
    MathematicalProgram& prog,
    std::string name, double time, int deriv_order, double value,
    drake::solvers::VectorXDecisionVariable slack_var) {

  Eigen::VectorXd constraint = VectorXd::Ones(degree_ + 2);
  constraint.head(degree_ + 1) = EvalLegendreBasisDerivative(
      degree_, deriv_order, time);

  DRAKE_DEMAND(not my_equality_constraints_.contains({deriv_order, name}));
  DRAKE_DEMAND(deriv_order >= 0);
  DRAKE_DEMAND(-1 <= time and time <= 1);
  DRAKE_DEMAND(slack_var.rows() == 1);

  my_equality_constraints_[{deriv_order, name}] = prog.AddLinearEqualityConstraint(
      constraint.transpose(), Vector1d::Constant(value),
      {c_, slack_var}
  ).evaluator();
}

void LegendreCoefficientVariables::UpdateDerivativeSoftConstraint(
    std::string name, double time, int deriv_order, double value) {
  Eigen::VectorXd constraint = VectorXd::Ones(degree_ + 2);
  constraint.head(degree_ + 1) = EvalLegendreBasisDerivative(
      degree_, deriv_order, time);
  my_equality_constraints_.at({deriv_order, name})->UpdateCoefficients(
      constraint.transpose(), Vector1d::Constant(value));
}

void LegendreCoefficientVariables::AddRunningCostOnDerivativeSquared(
    MathematicalProgram& prog, std::string name, int derivative_order,
    double scale) {
  DRAKE_DEMAND(not my_costs_.contains(name));
  DRAKE_DEMAND(derivative_order >= 1);
  DRAKE_DEMAND(scale >= 0);
  Eigen::MatrixXd Q =  MakeCostMatrixForMinimizingPathDerivativeSquaredWithLegendreBasis(
      degree_, derivative_order);
  Q = scale * (Q + 1e-6 * Eigen::MatrixXd::Identity(Q.rows(), Q.cols()));
  my_costs_[name] = prog.AddQuadraticCost(
      Q, VectorXd::Zero(degree_ + 1), c_).evaluator();
}

}
}