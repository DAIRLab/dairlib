#include "examples/goldilocks_models/planning/FoM_guard_constraint.h"


namespace dairlib {
namespace goldilocks_models {
namespace planning {

FomGuardConstraint::FomGuardConstraint(
  bool left_stance, int n_q, int n_v, VectorXd lb, VectorXd ub,
  const std::string& description):
  Constraint(2,
             n_q + n_v,
             lb,
             ub,
             description),
  left_stance_(left_stance) {
}

void FomGuardConstraint::DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                                Eigen::VectorXd* y) const {
  EvaluateConstraint(x, y);
}

void FomGuardConstraint::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
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

void FomGuardConstraint::DoEval(const
                                Eigen::Ref<const VectorX<Variable>>& x,
                                VectorX<Expression>*y) const {
  throw std::logic_error(
    "This constraint class does not support symbolic evaluation.");
}

void FomGuardConstraint::EvaluateConstraint(
  const Eigen::Ref<const VectorX<double>>& x, VectorX<double>* y) const {
  // Manually get the jacobain (tested in misc/foot_jacobian_test.cc)
  if (left_stance_) {
    VectorX<double> right_foot_pos_z(1);
    right_foot_pos_z <<
                     x(1) - 0.5 * cos(x(2) + x(4)) - 0.5 * cos(x(2) + x(4) + x(6));
    MatrixX<double> J_right_foot_pos_z(1, 7);
    J_right_foot_pos_z << 0,
                       1,
                       0.5 * sin(x(2) + x(4)) + 0.5 * sin(x(2) + x(4) + x(6)),
                       0,
                       0.5 * sin(x(2) + x(4)) + 0.5 * sin(x(2) + x(4) + x(6)),
                       0,
                       0.5 * sin(x(2) + x(4) + x(6));
    VectorX<double> right_foot_vel_z = J_right_foot_pos_z * x.tail(7);

    VectorX<double> output(2);
    output << right_foot_pos_z, right_foot_vel_z;
    *y = output;
  } else {
    VectorX<double> left_foot_pos_z(1);
    left_foot_pos_z <<
                    x(1) - 0.5 * cos(x(2) + x(3)) - 0.5 * cos(x(2) + x(3) + x(5));
    MatrixX<double> J_left_foot_pos_z(1, 7);
    J_left_foot_pos_z << 0,
                      1,
                      0.5 * sin(x(2) + x(3)) + 0.5 * sin(x(2) + x(3) + x(5)),
                      0.5 * sin(x(2) + x(3)) + 0.5 * sin(x(2) + x(3) + x(5)),
                      0,
                      0.5 * sin(x(2) + x(3) + x(5)),
                      0;
    VectorX<double> left_foot_vel_z = J_left_foot_pos_z * x.tail(7);

    VectorX<double> output(2);
    output << left_foot_pos_z, left_foot_vel_z;
    *y = output;
  }
}


}  // namespace planning
}  // namespace goldilocks_models
}  // namespace dairlib
