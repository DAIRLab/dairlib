#pragma once
#include "nonlinear_least_squares_cost.h"

namespace dairlib::systems::controllers::id_mpc {

template <typename T>
class QuadraticErrorCost : public NonlinearLeastSquaresCost<T> {
 public:
  QuadraticErrorCost(const Eigen::MatrixXd& x_ref, const Eigen::VectorXd &Q,
                     const std::string &description="");

  void EvaluateInnerTerm(const Eigen::Ref<const drake::AutoDiffVecXd> &x,
                         drake::AutoDiffVecXd *y) const override;

  void EvaluateInnerTerm(const Eigen::Ref<const Eigen::VectorXd> &x,
                         Eigen::VectorXd *y) const override;

  GaussNewtonApproximation CalcGaussNewtonApproximation(
      const Eigen::Ref<const Eigen::VectorXd>& x) const override;

  void UpdateReference(const Eigen::VectorXd& x) {
    DRAKE_ASSERT(x.rows() == x_ref_.rows());
    x_ref_ = x;
  }

 private:
  Eigen::VectorXd x_ref_;

};

}

