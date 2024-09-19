#include "legendre_polynomial_trajectory.h"
#include "legendre.h"

namespace drake::trajectories {

using Eigen::MatrixXd;
using Eigen::VectorXd;

LegendrePolynomialTrajectory::LegendrePolynomialTrajectory(
    const MatrixXd& coefficients) :
    order_(coefficients.cols() - 1),
    coefficients_(coefficients),
    derivative_operator_(dairlib::polynomials::LegendreBasisDerivativeOperator(order_)){}

std::unique_ptr<Trajectory<double>> LegendrePolynomialTrajectory::Clone() const {
  return std::make_unique<LegendrePolynomialTrajectory>(*this);
}

MatrixXd LegendrePolynomialTrajectory::value(const double &t) const {
  DRAKE_ASSERT(t > -1.0 and t < 1.0);
  VectorXd basis = dairlib::polynomials::EvalLegendreBasis(order_, t);
  return coefficients_ * basis;
}

MatrixXd LegendrePolynomialTrajectory::TakeDerivativeOfLegendreCoefficients(
    const MatrixXd& coeff, int derivative_order) const {
  DRAKE_ASSERT(coeff.rows() == rows() && coeff.cols() == order_ + 1);
  DRAKE_DEMAND(derivative_order >= 0);

  MatrixXd ret = coeff;
  for (int i = 0; i < derivative_order; ++i) {
    ret = ret * derivative_operator_.transpose().cast<double>();
  }
  return ret;
}

MatrixXd LegendrePolynomialTrajectory::DoEvalDerivative(
    const double &t, int derivative_order) const {
  DRAKE_ASSERT(t > -1.0 and t < 1.0);
  
  VectorXd basis = dairlib::polynomials::EvalLegendreBasis(order_, t);
  return TakeDerivativeOfLegendreCoefficients(coefficients_, derivative_order) * basis;
}

std::unique_ptr<Trajectory<double>> LegendrePolynomialTrajectory::DoMakeDerivative(
    int derivative_order) const {
  return std::make_unique<LegendrePolynomialTrajectory>(
      this->TakeDerivativeOfLegendreCoefficients(
          coefficients_, derivative_order));
}

}