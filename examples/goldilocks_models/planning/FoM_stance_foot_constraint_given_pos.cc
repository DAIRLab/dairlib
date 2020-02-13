#include "examples/goldilocks_models/planning/FoM_stance_foot_constraint_given_pos.h"


namespace dairlib {
namespace goldilocks_models {
namespace planning {

FomStanceFootConstraintGivenPos::FomStanceFootConstraintGivenPos(
  bool left_stance, int n_q, VectorXd pos,
  const std::string& description):
  Constraint(2,
             n_q,
             VectorXd::Zero(2),
             VectorXd::Zero(2),
             description),
  pos_(pos),
  left_stance_(left_stance) {
}

void FomStanceFootConstraintGivenPos::DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                                     Eigen::VectorXd* y) const {
  EvaluateConstraint(x, y);
}

void FomStanceFootConstraintGivenPos::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                                     AutoDiffVecXd* y) const {
  // forward differencing
  /*double dx = 1e-8;

  VectorXd x_val = autoDiffToValueMatrix(x);
  VectorXd y0, yi;
  EvaluateConstraint(x_val, &y0);

  MatrixXd dy = MatrixXd(y0.size(), x_val.size());
  for (int i = 0; i < x_val.size(); i++) {
    x_val(i) += dx;
    EvaluateConstraint(x_val, &yi);
    x_val(i) -= dx;
    dy.col(i) = (yi - y0) / dx;
  }
  drake::math::initializeAutoDiffGivenGradientMatrix(y0, dy, *y);*/

  // central differencing
  double dx = 1e-8;

  VectorXd x_val = autoDiffToValueMatrix(x);
  VectorXd y0, yi;
  EvaluateConstraint(x_val, &y0);

  MatrixXd dy = MatrixXd(y0.size(), x_val.size());
  for (int i = 0; i < x_val.size(); i++) {
    x_val(i) -= dx / 2;
    EvaluateConstraint(x_val, &y0);
    x_val(i) += dx;
    EvaluateConstraint(x_val, &yi);
    x_val(i) -= dx / 2;
    dy.col(i) = (yi - y0) / dx;
  }
  EvaluateConstraint(x_val, &y0);
  drake::math::initializeAutoDiffGivenGradientMatrix(y0, dy, *y);
}

void FomStanceFootConstraintGivenPos::DoEval(const
                                     Eigen::Ref<const VectorX<Variable>>& x,
                                     VectorX<Expression>*y) const {
  throw std::logic_error(
    "This constraint class does not support symbolic evaluation.");
}

void FomStanceFootConstraintGivenPos::EvaluateConstraint(
  const Eigen::Ref<const VectorX<double>>& x, VectorX<double>* y) const {
  VectorX<double> q0 = x;

  if (left_stance_) {
    VectorX<double> left_foot_pos_xz_0(2);
    left_foot_pos_xz_0 <<
                       q0(0) - 0.5 * sin(q0(2) + q0(3)) - 0.5 * sin(q0(2) + q0(3) + q0(5)),
                       q0(1) - 0.5 * cos(q0(2) + q0(3)) - 0.5 * cos(q0(2) + q0(3) + q0(5));
    *y = left_foot_pos_xz_0 - pos_;
  } else {
    VectorX<double> right_foot_pos_xz_0(2);
    right_foot_pos_xz_0 <<
                        q0(0) - 0.5 * sin(q0(2) + q0(4)) - 0.5 * sin(q0(2) + q0(4) + q0(6)),
                        q0(1) - 0.5 * cos(q0(2) + q0(4)) - 0.5 * cos(q0(2) + q0(4) + q0(6));
    *y = right_foot_pos_xz_0 - pos_;
  }
}


}  // namespace planning
}  // namespace goldilocks_models
}  // namespace dairlib
