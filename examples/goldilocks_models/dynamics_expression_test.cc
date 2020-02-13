#include <iostream>
#include "math.h"
#include <Eigen/Dense>

#include "examples/goldilocks_models/dynamics_expression.h"
#include "drake/common/drake_assert.h"

using std::cout;
using std::endl;
using Eigen::VectorXd;
using Eigen::MatrixXd;

using drake::AutoDiffVecXd;
using drake::AutoDiffXd;
using drake::math::DiscardGradient;
using drake::math::autoDiffToValueMatrix;
using drake::math::autoDiffToGradientMatrix;
using drake::math::initializeAutoDiff;

int main() {
  int n_sDDot = 3;
  int n_s = n_sDDot;
  int n_feature = 5;
  dairlib::goldilocks_models::DynamicsExpression expr(n_sDDot, n_feature, 0);

  VectorXd s(n_s);
  // Matrix<double, Dynamic, 1> s(2);
  s << M_PI / 2, 3, 0;
  AutoDiffVecXd s_autoDiff = initializeAutoDiff(s);
  DRAKE_DEMAND(n_s == s.size());

  // Copy to velocity
  VectorXd ds = s;
  AutoDiffVecXd ds_autoDiff = s_autoDiff;

  ////// getFeature() //////
  VectorXd feature = expr.getFeature(s, ds);
  // cout << "feature = \n" << feature << "\n\n";
  auto feature_autoDiff = expr.getFeature(s_autoDiff, ds_autoDiff);
  cout << "feature_autoDiff = \n" << feature_autoDiff << "\n\n";

  ////// getDimFeature() //////
  // int num_feature = expr.getDimFeature();
  // cout << "num_feature = \n" << num_feature << "\n\n";
  // int num_feature_autoDiff = expr.getDimFeature();
  // cout << "num_feature_autoDiff = \n" << n_feature_autoDiff << "\n\n";

  ///// getExpression() //////
  VectorXd theta = VectorXd::Zero(n_sDDot * n_feature);
  theta << 1, 1, 0, 0, 0,
           0, 0, 1, 0, 0,
           0, 0, 0, 1, 1;
  DRAKE_DEMAND(n_sDDot * n_feature == theta.size());
  // Features implemented in DynamicsExpression should be:
  // feature << s(0),
  //            s(1)*s(1)*s(1),
  //            s(0) * s(1),
  //            cos(s(0)),
  //            sqrt(s(1));

  // expression =
  //      s(0) + s(1)*s(1)*s(1),
  //      s(0) * s(1),
  //      cos(s(0)) + sqrt(s(1));

  // We don't have getExpression() that returns VectorX<double>, so we use
  // DiscardGradient here.
  VectorX<double> expression = expr.getExpression(theta, s, ds);
  cout << "expression = \n" << expression << "\n\n";

  AutoDiffVecXd theta_autoDiff =  initializeAutoDiff(theta);
  // auto expression_autoDiff = expr.getExpression(theta_autoDiff,s_autoDiff);
  auto expression_autoDiff = expr.getExpression(theta, s_autoDiff, ds_autoDiff);
  cout << "expression_autoDiff = \n" << expression_autoDiff << "\n\n";

  // Checking autoDiff
  MatrixXd jacobian  = autoDiffToGradientMatrix(expression_autoDiff);
  cout << "jacobian = \n" << jacobian << "\n\n";


  return 0;
}
