#include "examples/goldilocks_models/planning/kinematics_constraint.h"


namespace dairlib {
namespace goldilocks_models {
namespace planning {

KinematicsConstraint::KinematicsConstraint(
  int n_r, int n_q, int n_feature_kin,
  const VectorXd & theta_kin,
  int robot_option,
  const std::string& description):
  Constraint(2 * n_r,
             2 * (n_r + n_q),
             VectorXd::Zero(2 * n_r),
             VectorXd::Zero(2 * n_r),
             description),
  n_r_(n_r),
  n_y_(2 * n_r),
  n_q_(n_q),
  n_x_(2 * n_q),
  theta_kin_(theta_kin),
  kin_expression_(KinematicsExpression<AutoDiffXd>(n_r, n_feature_kin, robot_option)) {

  // Check the theta size
  DRAKE_DEMAND(n_r * n_feature_kin == theta_kin.size());

  // Check the feature size implemented in the model expression
  AutoDiffVecXd q_temp = initializeAutoDiff(VectorXd::Ones(n_q));
  DRAKE_DEMAND(n_feature_kin == kin_expression_.getFeature(q_temp).size());
}


void KinematicsConstraint::DoEval(const Eigen::Ref<const Eigen::VectorXd>& q,
                                  Eigen::VectorXd* y) const {
  AutoDiffVecXd y_t;
  Eval(initializeAutoDiff(q), &y_t);
  *y = autoDiffToValueMatrix(y_t);
}

void KinematicsConstraint::DoEval(const Eigen::Ref<const AutoDiffVecXd>& yx,
                                  AutoDiffVecXd* value) const {
  // Extract elements
  AutoDiffVecXd r = yx.segment(0, n_r_);
  // cout << "grad of r = \n" << autoDiffToGradientMatrix(r) << endl;
  AutoDiffVecXd dr = yx.segment(n_r_, n_r_);
  // cout << "grad of dr = \n" << autoDiffToGradientMatrix(dr) << endl;
  AutoDiffVecXd q = yx.segment(n_y_, n_q_);
  // cout << "grad of q = \n" << autoDiffToGradientMatrix(q) << endl;
  AutoDiffVecXd v = yx.segment(n_y_ + n_q_, n_q_);
  // cout << "grad of v = \n" << autoDiffToGradientMatrix(v) << endl;

  AutoDiffVecXd h = kin_expression_.getExpression(theta_kin_, q);

  // Way 1: numerically by autodiff ////////////////////////////////////////////
  /*VectorXd dhdt0 = autoDiffToGradientMatrix(h).block(0, n_y_, n_r_, n_q_) *
                   DiscardGradient(v);
  MatrixXd grad_dhdt = MatrixXd::Zero(n_r_, n_y_ + n_x_);
  for (int i = n_y_; i < n_y_ + n_q_; i++) {
    // Forward differencing
    q(i - n_y_) += eps_;
    VectorXd dhdti = autoDiffToGradientMatrix(
                       kin_expression_.getExpression(theta_kin_, q)).
                     block(0, n_y_, n_r_, n_q_) * DiscardGradient(v);
    q(i - n_y_) -= eps_;
    grad_dhdt.col(i) = (dhdti - dhdt0) / eps_;
  }
  grad_dhdt.block(0, n_y_ + n_q_, n_r_, n_q_) =
    autoDiffToGradientMatrix(h).block(0, n_y_, n_r_, n_q_);

  AutoDiffVecXd dhdt = initializeAutoDiff(dhdt0);
  drake::math::initializeAutoDiffGivenGradientMatrix(
    dhdt0, grad_dhdt, dhdt);*/

  // Way 2: analytically by hand ///////////////////////////////////////////////
  AutoDiffVecXd dhdt_analytic = kin_expression_.getExpressionDot(
                                  theta_kin_, q, v);

  // Comparison ////////////////////////////////////////////////////////////////
  // AutoDiffVecXd difference_dr = dhdt0.transpose() - dhdt_analytic.transpose();
  // cout << "dhdt_numerical = " << dhdt0.transpose() << endl;
  // cout << "dhdt_analytic = " << dhdt_analytic.transpose() << endl<< endl;
  // cout << "dhdt_numerical - dhdt_analytic = " << difference_dr << endl << endl;
  // for (int i = 0; i < 4; i++) {
  //   if (difference_dr(i) > 0.01) {
  //     cout << "the difference in dr is > 0.01\n";
  //     cout << "dhdt_numerical - dhdt_analytic = " << difference_dr << endl << endl;
  //   }
  // }

  // MatrixXd difference_ddrdq = grad_dhdt - autoDiffToGradientMatrix(dhdt_analytic);
  // double max_diff = 0;
  // for (int i = 0; i < difference_ddrdq.rows(); i++) {
  //   for (int j = 0; j < difference_ddrdq.cols(); j++) {
  //     if (difference_ddrdq(i, j) > 0.1) cout << "the difference in ddrdq is > 0.1\n";
  //     if (abs(difference_ddrdq(i, j)) > max_diff)
  //       max_diff = abs(difference_ddrdq(i, j));
  //   }
  // }
  // cout << "max_diff = " << max_diff << endl;
  //////////////////////////////////////////////////////////////////////////////

  VectorX<AutoDiffXd> output(n_y_);
  output << r - h,
         dr - dhdt_analytic;

  // Impose dynamics constraint
  *value = output;
}

void KinematicsConstraint::DoEval(const
                                  Eigen::Ref<const VectorX<Variable>>& x,
                                  VectorX<Expression>*y) const {
  throw std::logic_error(
    "This constraint class does not support symbolic evaluation.");
}



}  // namespace planning
}  // namespace goldilocks_models
}  // namespace dairlib
