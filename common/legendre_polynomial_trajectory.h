#pragma once

#include "drake/common/trajectories/trajectory.h"


namespace drake {
namespace trajectories {

/*
 *  A vector-valued trajectory represented as a linear combination of orthogonal
 *  polynomials
 */
class LegendrePolynomialTrajectory final : public Trajectory<double> {

 public:

  LegendrePolynomialTrajectory() = default;
  explicit LegendrePolynomialTrajectory(const MatrixX<double>& ceofficients);

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(LegendrePolynomialTrajectory);

  /**
   * @return A deep copy of this Trajectory.
   */
  std::unique_ptr<Trajectory<double>> Clone() const override;

  /**
   * Evaluates the trajectory at the given time \p t.
   * @param t The time at which to evaluate the trajectory.
   * @return The matrix of evaluated values.
   */
  MatrixX<double> value(const double& t) const override;


  /**
   * @return The number of rows in the matrix returned by value().
   */
  Eigen::Index rows() const override { return coefficients_.rows(); };

  /**
   * @return The number of columns in the matrix returned by value().
   */
  Eigen::Index cols() const override { return 1; }

  double start_time() const override { return -1.0; }

  double end_time() const override { return 1.0; }

 private:
  int order_;
  MatrixX<double> coefficients_;
  MatrixX<int> derivative_operator_;

  bool do_has_derivative() const { return true; };

  MatrixX<double> DoEvalDerivative(const double& t, int derivative_order) const override;

  std::unique_ptr<Trajectory<double>> DoMakeDerivative(int derivative_order) const override;

  MatrixX<double> TakeDerivativeOfLegendreCoefficients(const MatrixX<double>&, int) const;

};

}  // namespace trajectories
}  // namespace drake