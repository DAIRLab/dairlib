#include "nonlinear_least_squares_cost.h"
#include "drake/math/autodiff_gradient.h"


namespace dairlib::systems::controllers::id_mpc {

using drake::AutoDiffXd;
using drake::AutoDiffVecXd;
using drake::math::InitializeAutoDiff;
using drake::math::ExtractGradient;
using drake::math::ExtractValue;
using drake::solvers::Cost;
using drake::VectorX;

using Eigen::MatrixXd;
using Eigen::VectorXd;

template <typename T>
NonlinearLeastSquaresCost<T>::NonlinearLeastSquaresCost(
    int num_vars, const Eigen::MatrixXd &Q, const std::string &description,
    double eps) : Cost(num_vars, description), Q_(Q), eps_(eps)  {
  DRAKE_DEMAND(eps_ > 0);
  DRAKE_DEMAND(Q_.cols() == Q_.rows());
}

template <>
void NonlinearLeastSquaresCost<double>::DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                                   Eigen::VectorXd* y) const {
  EvaluateCost(x, y);
}

template <>
void NonlinearLeastSquaresCost<AutoDiffXd>::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd* y) const {
  AutoDiffVecXd y_t;
  EvaluateCost(drake::math::InitializeAutoDiff(x), &y_t);
  *y = drake::math::ExtractValue(y_t);
}

template <typename T>
void NonlinearLeastSquaresCost<T>::DoEval(
    const Eigen::Ref<const VectorX<drake::symbolic::Variable>>& x,
    VectorX<drake::symbolic::Expression>* y) const {
  throw std::logic_error("NonlinearCost does not support symbolic evaluation.");
}

template <>
void NonlinearLeastSquaresCost<AutoDiffXd>::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                                       AutoDiffVecXd* y) const {
  EvaluateCost(x, y);
}

template <>
void NonlinearLeastSquaresCost<double>::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                                   AutoDiffVecXd* y) const {
  MatrixXd original_grad = drake::math::ExtractGradient(x);

  // forward differencing
  VectorXd x_val = drake::math::ExtractValue(x);
  VectorXd y0, yi;
  EvaluateCost(x_val, &y0);

  MatrixXd dy = MatrixXd(y0.size(), x_val.size());
  for (int i = 0; i < x_val.size(); i++) {
    x_val(i) += eps_;
    EvaluateCost(x_val, &yi);
    x_val(i) -= eps_;
    dy.col(i) = (yi - y0) / eps_;
  }
  *y = drake::math::InitializeAutoDiff(y0, dy * original_grad);
}

template <typename T>
GaussNewtonApproximation NonlinearLeastSquaresCost<T>::CalcGaussNewtonApproximation(
    const Eigen::Ref<const Eigen::VectorXd>& x) const {

  AutoDiffVecXd y;
  AutoDiffVecXd x_ad = InitializeAutoDiff(x);

  EvaluateInnerTerm(x_ad, &y);

  VectorXd yval = ExtractValue(y);
  MatrixXd J = ExtractGradient(y);

  // Nonlinear cost is y(x)^TQy(x)
  // Guass-Newton SQP cost is (J * dx + y)^TQ(J * dx + y)

  return {
      2 * J.transpose() * Q_ * J,
      2 * J.transpose() * Q_ * yval,
      yval.transpose() * Q_ * yval
  };
}

template<typename T>
void NonlinearLeastSquaresCost<T>::EvaluateCost(
    const Eigen::Ref<const VectorX<T>>& x, VectorX<T>* y) const {
  VectorX<T> yval;
  EvaluateInnerTerm(x, &yval);
  *y = VectorX<T>::Zero(1);
  (*y)(0) = yval.transpose() * Q_ * yval;
}

}

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::systems::controllers::id_mpc::NonlinearLeastSquaresCost)
