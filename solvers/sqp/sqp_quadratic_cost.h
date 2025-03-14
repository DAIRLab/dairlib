#pragma once
#include "nonlinear_least_squares_cost.h"

namespace dairlib::solvers::sqp {

class SqpQuadraticCost : public NonlinearLeastSquaresCost<double> {
 public:
  SqpQuadraticCost(const Eigen::MatrixXd &Q, const Eigen::VectorXd& b, double c,
                   const std::string &description="");

  void EvaluateInnerTerm(const Eigen::Ref<const drake::AutoDiffVecXd> &x,
                         drake::AutoDiffVecXd *y) const override;

  void EvaluateInnerTerm(const Eigen::Ref<const Eigen::VectorXd> &x,
                         Eigen::VectorXd *y) const override;

  GaussNewtonApproximation CalcGaussNewtonApproximation(
      const Eigen::Ref<const Eigen::VectorXd>& x) const override;

  void UpdateReference(const Eigen::VectorXd& y) override;

  void UpdateCoefficients(const Eigen::MatrixXd& Q, const Eigen::VectorXd& b,
                          double c);

  void EvaluateCost(const Eigen::Ref<const Eigen::VectorXd>& x,
                    Eigen::VectorXd* y) const override;

 private:

  Eigen::MatrixXd Q_;
  Eigen::VectorXd b_;
  double c_;


};

}