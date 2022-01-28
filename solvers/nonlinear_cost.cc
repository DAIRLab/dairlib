#include "solvers/nonlinear_cost.h"
#include "drake/common/default_scalars.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"

namespace dairlib {
namespace solvers {

using drake::AutoDiffVecXd;
using drake::AutoDiffXd;
using drake::VectorX;
using Eigen::MatrixXd;
using Eigen::VectorXd;

template <typename T>
NonlinearCost<T>::NonlinearCost(int num_vars, const std::string& description,
                                double eps)
    : Cost(num_vars, description), eps_(eps) {}

template <>
void NonlinearCost<double>::DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                                   Eigen::VectorXd* y) const {
  EvaluateCost(x, y);
}

template <>
void NonlinearCost<AutoDiffXd>::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd* y) const {
  AutoDiffVecXd y_t;
  EvaluateCost(drake::math::InitializeAutoDiff(x), &y_t);
  *y = drake::math::ExtractValue(y_t);
}

template <typename T>
void NonlinearCost<T>::DoEval(
    const Eigen::Ref<const VectorX<drake::symbolic::Variable>>& x,
    VectorX<drake::symbolic::Expression>* y) const {
  throw std::logic_error("NonlinearCost does not support symbolic evaluation.");
}

template <>
void NonlinearCost<AutoDiffXd>::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                                       AutoDiffVecXd* y) const {
  EvaluateCost(x, y);
}

template <>
void NonlinearCost<double>::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
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

}  // namespace solvers
}  // namespace dairlib

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::solvers::NonlinearCost)
