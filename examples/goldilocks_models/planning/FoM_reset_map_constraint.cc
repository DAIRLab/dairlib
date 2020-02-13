#include "examples/goldilocks_models/planning/FoM_reset_map_constraint.h"


namespace dairlib {
namespace goldilocks_models {
namespace planning {

FomResetMapConstraint::FomResetMapConstraint(
  bool left_stance, int n_q, int n_v, int n_J,
  const MultibodyPlant<double> & plant,
  const std::string& description):
  Constraint(n_v + n_J,
             2 * (n_q + n_v) + n_J,
             VectorXd::Zero(n_v + n_J),
             VectorXd::Zero(n_v + n_J),
             description),
  left_stance_(left_stance),
  plant_(plant),
  n_J_(n_J),
  n_q_(n_q) {
}

void FomResetMapConstraint::DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                                   Eigen::VectorXd* y) const {
  EvaluateConstraint(x, y);
}

void FomResetMapConstraint::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
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

void FomResetMapConstraint::DoEval(const Eigen::Ref<const VectorX<Variable>>& x,
                                   VectorX<Expression>*y) const {
  throw std::logic_error(
    "This constraint class does not support symbolic evaluation.");
}

void FomResetMapConstraint::EvaluateConstraint(
  const Eigen::Ref<const VectorX<double>>& x, VectorX<double>* y) const {
  VectorX<double> qm = x.segment(0, 7);  // m stands for minus (pre-impact)
  VectorX<double> vm = x.segment(7, 7);
  VectorX<double> qp = x.segment(14, 7);  // p stands for plus (post-impact)
  VectorX<double> vp = x.segment(21, 7);
  VectorX<double> Lambda = x.tail(n_J_);

  MatrixXd M = MatrixXd(n_q_, n_q_);
  MatrixXd M_ext = MatrixXd(n_q_ + n_J_, n_q_ + n_J_);
  std::unique_ptr<drake::systems::Context<double>> context =
        plant_.CreateDefaultContext();
  plant_.SetPositions(context.get(), qm);
  plant_.CalcMassMatrixViaInverseDynamics(*context, &M);

  M_ext.block(0, 0, n_q_, n_q_) = M;
  VectorXd vm_ext(n_q_ + n_J_);
  vm_ext << -M*vm, VectorXd::Zero(n_J_);
  VectorXd vp_ext(n_q_ + n_J_);
  vp_ext << vp, Lambda;

  if (left_stance_) {
    // VectorX<double> left_foot_pos_xz(2);
    // left_foot_pos_xz <<
    //                  qm(0) - 0.5 * sin(qm(2) + qm(3)) - 0.5 * sin(qm(2) + qm(3) + qm(5)),
    //                  qm(1) - 0.5 * cos(qm(2) + qm(3)) - 0.5 * cos(qm(2) + qm(3) + qm(5));
    MatrixX<double> J_left_foot_pos_xz(2, 7);
    J_left_foot_pos_xz << 1,
                       0,
                       - 0.5 * cos(qm(2) + qm(3)) - 0.5 * cos(qm(2) + qm(3) + qm(5)),
                       - 0.5 * cos(qm(2) + qm(3)) - 0.5 * cos(qm(2) + qm(3) + qm(5)),
                       0,
                       - 0.5 * cos(qm(2) + qm(3) + qm(5)),
                       0,
                       0,
                       1,
                       0.5 * sin(qm(2) + qm(3)) + 0.5 * sin(qm(2) + qm(3) + qm(5)),
                       0.5 * sin(qm(2) + qm(3)) + 0.5 * sin(qm(2) + qm(3) + qm(5)),
                       0,
                       0.5 * sin(qm(2) + qm(3) + qm(5)),
                       0;

    M_ext.block(0, n_q_, n_q_, n_J_) = -J_left_foot_pos_xz.transpose();
    M_ext.block(n_q_, 0, n_J_, n_q_) = J_left_foot_pos_xz;

    *y = M_ext * vp_ext + vm_ext;

  } else {
    // VectorX<double> right_foot_pos_xz(2);
    // right_foot_pos_xz <<
    //                   qm(0) - 0.5 * sin(qm(2) + qm(4)) - 0.5 * sin(qm(2) + qm(4) + qm(6)),
    //                   qm(1) - 0.5 * cos(qm(2) + qm(4)) - 0.5 * cos(qm(2) + qm(4) + qm(6));
    MatrixX<double> J_right_foot_pos_xz(2, 7);
    J_right_foot_pos_xz << 1,
                        0,
                        - 0.5 * cos(qm(2) + qm(4)) - 0.5 * cos(qm(2) + qm(4) + qm(6)),
                        0,
                        - 0.5 * cos(qm(2) + qm(4)) - 0.5 * cos(qm(2) + qm(4) + qm(6)),
                        0,
                        - 0.5 * cos(qm(2) + qm(4) + qm(6)),
                        0,
                        1,
                        0.5 * sin(x(2) + x(4)) + 0.5 * sin(x(2) + x(4) + x(6)),
                        0,
                        0.5 * sin(x(2) + x(4)) + 0.5 * sin(x(2) + x(4) + x(6)),
                        0,
                        0.5 * sin(x(2) + x(4) + x(6));

    M_ext.block(0, n_q_, n_q_, n_J_) = -J_right_foot_pos_xz.transpose();
    M_ext.block(n_q_, 0, n_J_, n_q_) = J_right_foot_pos_xz;

    *y = M_ext * vp_ext + vm_ext;
  }
}


}  // namespace planning
}  // namespace goldilocks_models
}  // namespace dairlib
