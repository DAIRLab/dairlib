#pragma once
#include <vector>
#include <cmath>
#include <assert.h>

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/path_parameterized_trajectory.h"
#include "external/drake/tools/install/libdrake/_virtual_includes/drake_shared_library/drake/common/trajectories/piecewise_polynomial.h"

namespace dairlib::polynomials {

inline std::vector<double> get_chebyshev_points(int n) {
  assert(n >= 2);
  std::vector<double> cheb(n);
  for (int i = 0; i < n; i++) {
    double theta = M_PI *  (1.0 - static_cast<double>(i) / (n - 1.0));
    cheb[i] = 0.5 * (cos(theta) + 1);
  }
  // correct front and back in case of rounding error
  cheb.front() = 0.0;
  cheb.back() = 1.0;
  return cheb;
}

inline std::vector<double> get_even_points(int n) {
  assert(n >= 2);
  std::vector<double> even(n);
  for (int i = 0; i < n; i++) {
    even[i] = static_cast<double>(i) / static_cast<double>(n - 1);
  }
  // correct front and back in case of rounding error
  even.front() = 0.0;
  even.back() = 1.0;
  return even;
}

inline drake::trajectories::PiecewisePolynomial<double>
ConstantAccelerationTrajectory(
    const Eigen::VectorXd& y, const Eigen::VectorXd& ydot,
    const Eigen::VectorXd& yddot,
    double t) {
  DRAKE_DEMAND(y.rows() == ydot.rows());
  DRAKE_DEMAND(ydot.rows() == yddot.rows());

  drake::trajectories::PiecewisePolynomial<double>::PolynomialMatrix polys(y.rows(), 1);
  for (int i = 0; i < y.rows(); ++i) {
    Eigen::Vector3d coeffs(y(i), ydot(i), 0.5 * yddot(i));
    polys(i) = drake::Polynomiald(coeffs);
  }
  return drake::trajectories::PiecewisePolynomial<double>(
      {polys}, {t, std::numeric_limits<double>::infinity()});
}

}