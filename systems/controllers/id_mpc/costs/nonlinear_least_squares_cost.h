#pragma once

#include "drake/common/symbolic/expression.h"
#include "drake/solvers/cost.h"

namespace dairlib::systems::controllers::id_mpc {

struct GaussNewtonApproximation {
    Eigen::MatrixXd H; // Gauss Newton Hessian
    Eigen::MatrixXd g; // gradient
    double c;          // constant
};


/*!
 * Abstract class for a nonlinear cost of the form ||f(x) - fx||^2
 * @tparam T double or AutoDiffXd.
 */
template <typename T>
 class NonlinearLeastSquaresCost : public drake::solvers::Cost{
 public:
  NonlinearLeastSquaresCost(int num_vars, const Eigen::MatrixXd& Q,
                            const std::string& description="", double eps=1e-8);

   void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
               Eigen::VectorXd* y) const override;

   void DoEval(const Eigen::Ref<const drake::AutoDiffVecXd>& x,
               drake::AutoDiffVecXd* y) const override;

   void DoEval(
       const Eigen::Ref<const drake::VectorX<drake::symbolic::Variable>>&,
       drake::VectorX<drake::symbolic::Expression>*) const override;

  virtual void EvaluateInnerTerm(const Eigen::Ref<const drake::AutoDiffVecXd>& x,
                                drake::AutoDiffVecXd* y) const = 0;

  virtual void EvaluateInnerTerm(const Eigen::Ref<const Eigen::VectorXd>& x,
                                 Eigen::VectorXd* y) const = 0;

  void EvaluateCost(const Eigen::Ref<const drake::VectorX<T>>& x,
                    drake::VectorX<T>* y) const;

  virtual GaussNewtonApproximation CalcGaussNewtonApproximation(
      const Eigen::Ref<const Eigen::VectorXd>& x) const;

  void UpdateWeights(const Eigen::MatrixXd& Q) {
    DRAKE_ASSERT(Q_.rows() == Q.rows());
    DRAKE_ASSERT(Q_.cols() == Q.cols());
    Q_ = Q;
  }

 protected:
  Eigen::MatrixXd Q_;

 private:
  double eps_;



};

}

