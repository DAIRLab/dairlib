#include "examples/goldilocks_models/find_models/kinematics_constraint.h"


namespace dairlib {
namespace goldilocks_models {

KinematicsConstraint::KinematicsConstraint(
                                 int n_s, int n_feature, VectorXd & theta_s,
                                 const MultibodyPlant<AutoDiffXd> * plant,
                                 int robot_option,
                                 const std::string& description):
  Constraint(n_s,
             n_s + plant->num_positions(),
             VectorXd::Zero(n_s),
             VectorXd::Zero(n_s),
             description),
  expression_double(KinematicsExpression<double>(n_s, n_feature, robot_option)),
  expression_autoDiff(KinematicsExpression<AutoDiffXd>(n_s, n_feature, robot_option)),
  plant_(plant),
  n_constraint_(n_s),
  n_feature_(n_feature),
  n_q_(plant->num_positions()),
  theta_s_(theta_s) {

  // Check the theta size
  DRAKE_DEMAND(n_s * n_feature == theta_s.size());

  // Check the feature size implemented in the model expression
  VectorXd q_temp = VectorXd::Ones(plant->num_positions());
  DRAKE_DEMAND(n_feature == expression_double.getFeature(q_temp).size());
}

void KinematicsConstraint::DoEval(const
                             Eigen::Ref<const Eigen::VectorXd>& s_q,
                             Eigen::VectorXd* y) const {
  AutoDiffVecXd y_t;
  Eval(initializeAutoDiff(s_q), &y_t);
  *y = autoDiffToValueMatrix(y_t);
}

void KinematicsConstraint::DoEval(const
                             Eigen::Ref<const AutoDiffVecXd>& s_q,
                             AutoDiffVecXd* y) const {
  const AutoDiffVecXd s = s_q.head(n_constraint_);
  const AutoDiffVecXd q = s_q.tail(n_q_);

  *y = getKinematicsConstraint(s, q, theta_s_);
}

void KinematicsConstraint::DoEval(const
                             Eigen::Ref<const VectorX<Variable>>& q,
                             VectorX<Expression>*y) const {
  throw std::logic_error(
    "This constraint class does not support symbolic evaluation.");
}

VectorXd KinematicsConstraint::getGradientWrtTheta(const VectorXd & q){
  VectorXd s = VectorXd::Zero(n_constraint_);
  VectorXd gradient(n_feature_);
  for(int i = 0; i<n_feature_; i++){
    VectorXd theta_unit = VectorXd::Zero(theta_s_.size());
    theta_unit(i) = 1;
    gradient(i) = getKinematicsConstraint(s,q,theta_unit)(0);
  }
  return gradient;
}

AutoDiffVecXd KinematicsConstraint::getKinematicsConstraint(
  const AutoDiffVecXd & s, const AutoDiffVecXd & q, const VectorXd & theta) const{
  return s - expression_autoDiff.getExpression(theta, q);
}
VectorXd KinematicsConstraint::getKinematicsConstraint(
  const VectorXd & s, const VectorXd & q, const VectorXd & theta) const{
  return s - expression_double.getExpression(theta, q);
}




}  // namespace goldilocks_models
}  // namespace dairlib
