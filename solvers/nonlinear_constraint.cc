#include "solvers/nonlinear_constraint.h"

#include <chrono>

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

double nonlinear_constraint_eval_time = 0;

void ResetEvalTime() { nonlinear_constraint_eval_time = 0; }

template <typename T>
NonlinearConstraint<T>::NonlinearConstraint(int num_constraints, int num_vars,
                                            const VectorXd& lb,
                                            const VectorXd& ub,
                                            const std::string& description,
                                            double eps)
    : Constraint(num_constraints, num_vars, lb, ub, description), eps_(eps) {}

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
  //  auto start = std::chrono::high_resolution_clock::now();

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

  // Profiling identified dy * original_grad as a significant runtime event,
  // even though it is almost always the identity matrix.
  if (original_grad.isIdentity(1e-16)) {
    *y = drake::math::initializeAutoDiffGivenGradientMatrix(y0, dy);
  } else {
    *y = drake::math::initializeAutoDiffGivenGradientMatrix(y0,
                                                            dy * original_grad);
  }

  this->ScaleConstraint<AutoDiffXd>(y);

  /*auto finish = std::chrono::high_resolution_clock::now();

  std::chrono::duration<double> elapsed = finish - start;
  std::cout << "count: " << elapsed.count() << "\n";
  std::cout << "nonlinear_constraint_eval_time: "
            << nonlinear_constraint_eval_time << "\n";
  nonlinear_constraint_eval_time += elapsed.count();
  std::cout << "nonlinear_constraint_eval_time: "
            << nonlinear_constraint_eval_time << "\n\n";*/
}

}  // namespace solvers
}  // namespace dairlib

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::solvers::NonlinearConstraint)
