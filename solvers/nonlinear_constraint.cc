#include "solvers/nonlinear_constraint.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/common/default_scalars.h"

namespace dairlib {
namespace solvers {

using drake::AutoDiffXd;
using drake::AutoDiffVecXd;
using drake::VectorX;
using Eigen::MatrixXd;
using Eigen::VectorXd;

template <typename T>
NonlinearConstraint<T>::NonlinearConstraint(int num_constraints, int num_vars,
    const VectorXd& lb, const VectorXd& ub,
    const std::string& description, double eps)
    : Constraint(num_constraints, num_vars, lb, ub, description),
    eps_(eps) {}

template <typename T>
void NonlinearConstraint<T>::SetConstraintScaling(
    const std::unordered_map<int, double>& map) {
  constraint_scaling_ = map;
}

template <typename T>
template <typename U>
void NonlinearConstraint<T>::ScaleConstraint(VectorX<U>* y) const {
  for (const auto& member : constraint_scaling_) {
    (*y)(member.first) *= member.second;
  }
}

template <>
void NonlinearConstraint<double>::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd* y) const {
  EvaluateConstraint(x, y);
  this->ScaleConstraint<double>(y);
}

template <>
void NonlinearConstraint<AutoDiffXd>::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd* y) const {
  AutoDiffVecXd y_t;
  EvaluateConstraint(drake::math::initializeAutoDiff(x), &y_t);
  *y = drake::math::autoDiffToValueMatrix(y_t);
  this->ScaleConstraint<double>(y);
}

template <typename T>
void NonlinearConstraint<T>::DoEval(
    const Eigen::Ref<const VectorX<drake::symbolic::Variable>>& x,
    VectorX<drake::symbolic::Expression>* y) const {
  throw std::logic_error(
      "NonlinearConstraint does not support symbolic evaluation.");
}

template <>
void NonlinearConstraint<AutoDiffXd>::DoEval(
    const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd* y) const {
  EvaluateConstraint(x, y);
  this->ScaleConstraint<AutoDiffXd>(y);
}

template <>
void NonlinearConstraint<double>::DoEval(
    const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd* y) const {
  MatrixXd original_grad = drake::math::autoDiffToGradientMatrix(x);

  // forward differencing
  VectorXd x_val = drake::math::autoDiffToValueMatrix(x);
  VectorXd y0, yi;
  EvaluateConstraint(x_val, &y0);

  MatrixXd dy = MatrixXd(y0.size(), x_val.size());
  for (int i = 0; i < x_val.size(); i++) {
    x_val(i) += eps_;
    EvaluateConstraint(x_val, &yi);
    x_val(i) -= eps_;
    dy.col(i) = (yi - y0) / eps_;
  }
  drake::math::initializeAutoDiffGivenGradientMatrix(y0, dy * original_grad,
                                                     *y);

  this->ScaleConstraint<AutoDiffXd>(y);
}

}  // namespace solvers
}  // namespace dairlib

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::solvers::NonlinearConstraint)
