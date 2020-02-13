#include "examples/goldilocks_models/planning/kinematics_constraint_given_r.h"


namespace dairlib {
namespace goldilocks_models {
namespace planning {

KinematicsConstraintGivenR::KinematicsConstraintGivenR(
  int n_r, AutoDiffVecXd r,int n_q, int n_feature_kin,
  const VectorXd & theta_kin,
  int robot_option,
  const std::string& description):
  Constraint(n_r,
             n_q,
             VectorXd::Zero(n_r),
             VectorXd::Zero(n_r),
             description),
  r_(r),
  n_r_(n_r),
  n_q_(n_q),
  theta_kin_(theta_kin),
  kin_expression_(KinematicsExpression<AutoDiffXd>(n_r, n_feature_kin, robot_option)) {

  // Check the theta size
  DRAKE_DEMAND(n_r * n_feature_kin == theta_kin.size());

  // Check the feature size implemented in the model expression
  AutoDiffVecXd q_temp = initializeAutoDiff(VectorXd::Ones(n_q));
  DRAKE_DEMAND(n_feature_kin == kin_expression_.getFeature(q_temp).size());
}


void KinematicsConstraintGivenR::DoEval(const Eigen::Ref<const Eigen::VectorXd>& q,
                                  Eigen::VectorXd* y) const {
  AutoDiffVecXd y_t;
  Eval(initializeAutoDiff(q), &y_t);
  *y = autoDiffToValueMatrix(y_t);
}

void KinematicsConstraintGivenR::DoEval(const Eigen::Ref<const AutoDiffVecXd>& q,
                                  AutoDiffVecXd* value) const {
  AutoDiffVecXd h = kin_expression_.getExpression(theta_kin_, q);

  VectorX<AutoDiffXd> output(n_r_);
    output << r_ - h;

  // Impose dynamics constraint
  // *value = r_ - kin_expression_.getExpression(theta_kin_, q);
  *value = output;
}

void KinematicsConstraintGivenR::DoEval(const
                                  Eigen::Ref<const VectorX<Variable>>& x,
                                  VectorX<Expression>*y) const {
  throw std::logic_error(
    "This constraint class does not support symbolic evaluation.");
}



}  // namespace planning
}  // namespace goldilocks_models
}  // namespace dairlib
