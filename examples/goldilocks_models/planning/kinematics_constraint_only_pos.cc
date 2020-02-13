#include "examples/goldilocks_models/planning/kinematics_constraint_only_pos.h"


namespace dairlib {
namespace goldilocks_models {
namespace planning {

KinematicsConstraintOnlyPos::KinematicsConstraintOnlyPos(
  int n_r, int n_q, int n_feature_kin,
  const VectorXd & theta_kin,
  int robot_option,
  const std::string& description):
  Constraint(n_r,
             (n_r + n_q),
             VectorXd::Zero(n_r),
             VectorXd::Zero(n_r),
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


void KinematicsConstraintOnlyPos::DoEval(const Eigen::Ref<const Eigen::VectorXd>& q,
                                  Eigen::VectorXd* y) const {
  AutoDiffVecXd y_t;
  Eval(initializeAutoDiff(q), &y_t);
  *y = autoDiffToValueMatrix(y_t);
}

void KinematicsConstraintOnlyPos::DoEval(const Eigen::Ref<const AutoDiffVecXd>& rq,
                                  AutoDiffVecXd* value) const {
  // Extract elements
  AutoDiffVecXd r = rq.segment(0, n_r_);
  // cout << "grad of r = \n" << autoDiffToGradientMatrix(r) << endl;
  AutoDiffVecXd q = rq.segment(n_r_, n_q_);
  // cout << "grad of q = \n" << autoDiffToGradientMatrix(q) << endl;

  AutoDiffVecXd h = kin_expression_.getExpression(theta_kin_, q);

  VectorX<AutoDiffXd> output(n_r_);
  output << r - h;

  // Impose dynamics constraint
  *value = output;
}

void KinematicsConstraintOnlyPos::DoEval(const
                                  Eigen::Ref<const VectorX<Variable>>& x,
                                  VectorX<Expression>*y) const {
  throw std::logic_error(
    "This constraint class does not support symbolic evaluation.");
}



}  // namespace planning
}  // namespace goldilocks_models
}  // namespace dairlib
