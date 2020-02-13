#include <iostream>
#include <string>
#include "math.h"
#include <Eigen/Dense>

#include "examples/goldilocks_models/kinematics_expression.h"
#include "drake/common/drake_assert.h"


#include "drake/systems/framework/system.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/rendering/multibody_position_to_geometry_pose.h"
#include "multibody/multibody_utils.h"
#include "common/find_resource.h"



using std::cout;
using std::endl;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;

using drake::AutoDiffVecXd;
using drake::AutoDiffXd;
using drake::math::DiscardGradient;
using drake::math::autoDiffToValueMatrix;
using drake::math::autoDiffToGradientMatrix;
using drake::math::initializeAutoDiff;

using dairlib::FindResourceOrThrow;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;

int main() {

  ////////////////////////////// Test 1 ////////////////////////////////////////

  int n_s = 3;
  int n_x = 4; // constraint input
  int n_q = 2;
  int n_feature = 5;
  dairlib::goldilocks_models::KinematicsExpression<double> expr_double(
    n_s, n_feature);
  dairlib::goldilocks_models::KinematicsExpression<AutoDiffXd> expr(
    n_s, n_feature);

  VectorXd x(n_x);
  // Matrix<double, Dynamic, 1> x(n_x);
  x << M_PI / 2, 3, 9, 7.7;
  AutoDiffVecXd x_autoDiff = initializeAutoDiff(x);

  AutoDiffVecXd q_autoDiff = x_autoDiff.head(n_q);
  VectorXd q = DiscardGradient(q_autoDiff);
  DRAKE_DEMAND(n_q == q.size());

  ////// getFeature() //////
  // VectorXd feature = expr.getFeature(q);
  // cout << "feature = \n" << feature << "\n\n";
  auto feature_autoDiff = expr.getFeature(q_autoDiff);
  cout << "feature_autoDiff = \n" << feature_autoDiff << "\n\n";

  ////// getDimFeature() //////
  // int num_feature = expr.getDimFeature();
  // cout << "num_feature = \n" << num_feature << "\n\n";
  // int num_feature_autoDiff = expr.getDimFeature();
  // cout << "num_feature_autoDiff = \n" << n_feature_autoDiff << "\n\n";

  ///// getExpression() //////
  VectorXd theta = VectorXd::Zero(n_s * n_feature);
  theta << 1, 1, 0, 0, 0,
           0, 0, 1, 0, 0,
           0, 0, 0, 1, 1;
  DRAKE_DEMAND(n_s * n_feature == theta.size());
  // Features implemented in KinematicsExpression should be:
  // feature << q(0),
  //            q(1)*q(1)*q(1),
  //            q(0) * q(1),
  //            cos(q(0)),
  //            sqrt(q(1));

  // expression =
  //      q(0) + q(1)*q(1)*q(1),
  //      q(0) * q(1),
  //      cos(q(0)) + sqrt(q(1));

  VectorX<double> expression_double = expr_double.getExpression(theta, q);
  cout << "double expression (double class) = \n" << expression_double << "\n\n";
  // VectorX<double> expression = DiscardGradient(expr.getExpression(theta, q));
  // cout << "double expression (AutoDiffXd class) = \n" << expression << "\n\n";

  AutoDiffVecXd theta_autoDiff =  initializeAutoDiff(theta);
  // auto expression_autoDiff = expr.getExpression(theta_autoDiff,q_autoDiff);
  auto expression_autoDiff = expr.getExpression(theta, q_autoDiff);
  cout << "expression_autoDiff = \n" << expression_autoDiff << "\n\n";

  // Checking autoDiff
  MatrixXd jacobian  = autoDiffToGradientMatrix(expression_autoDiff);
  cout << "jacobian = \n" << jacobian << "\n\n";


  ////////////////////////////// Test 2 ////////////////////////////////////////
  // Test the autoDiff plant
  // Get the position of the foot and check the gradient wrt state

  /*MultibodyPlant<double> plant;
  Parser parser(&plant);

  std::string full_name =
    FindResourceOrThrow("examples/goldilocks_models/PlanarWalkerWithTorso.urdf");
  parser.AddModelFromFile(full_name);
  plant.AddForceElement<drake::multibody::UniformGravityFieldElement>(
    -9.81 * Eigen::Vector3d::UnitZ());
  plant.WeldFrames(
    plant.world_frame(), plant.GetFrameByName("base"),
    drake::math::RigidTransform<double>(Vector3d::Zero()).GetAsIsometry3());
  plant.Finalize();

  MultibodyPlant<AutoDiffXd> plant_autoDiff(plant);

  int n_s = 2; // Doesn't matter here
  int n_q = plant_autoDiff.num_positions();
  int n_feature = 3; // Doesn't matter here either actually
  dairlib::goldilocks_models::KinematicsExpression<double> expr_double(
    n_s, n_feature, &plant);
  dairlib::goldilocks_models::KinematicsExpression<AutoDiffXd> expr(
    n_s, n_feature, &plant_autoDiff);

  // Matrix<double, Dynamic, 1> q(n_q);
  VectorXd q = VectorXd::Zero(n_q);
  // q(3) = -M_PI/4;
  // q(5) =  M_PI/4;
  AutoDiffVecXd q_autoDiff = initializeAutoDiff(q);
  DRAKE_DEMAND(n_q == q.size());

  auto feature_autoDiff = expr.getFeature(q_autoDiff);
  cout << "feature_autoDiff = \n" << feature_autoDiff << "\n\n";

  // Checking autoDiff
  MatrixXd jacobian  = autoDiffToGradientMatrix(feature_autoDiff);
  cout << "jacobian = \n" << jacobian << "\n\n";*/

  ////////////////////////////// Test 3 ////////////////////////////////////////
  // Test when theta is autoDiff
  // seems that it is working correctly.

  /*cout << "When theta is autoDiff and q is double\n";
  int n_s = 2;
  int n_feature = 2;
  VectorXd theta_val(n_s*n_feature);
  theta_val << M_PI/2, 3, 6, -0.3;
  AutoDiffVecXd theta = initializeAutoDiff(theta_val);
  VectorXd feature(n_feature);
  feature << 2, 1;

  VectorX<AutoDiffXd> expression(n_s);
  for (int i = 0; i < n_s ; i++)
    expression(i) =
      theta.segment(i * n_feature, n_feature).dot(feature);

  cout << "expression_autoDiff = \n" << expression << "\n\n";
  MatrixXd jacobian  = autoDiffToGradientMatrix(expression);
  cout << "jacobian = \n" << jacobian << "\n\n";*/

  ////////////////////////////// Test 4 ////////////////////////////////////////
  // Test the case when theta and feature are both in autodiff
  // seems that it doesn't work, so I commented it out.

  /*int n_s = 3;
  int n_q = 2;
  int n_feature = 5;
  dairlib::goldilocks_models::KinematicsExpression<double> expr_double(
    n_s, n_feature);
  dairlib::goldilocks_models::KinematicsExpression<AutoDiffXd> expr(
    n_s, n_feature);

  VectorXd q(n_q);
  q << M_PI / 2, 3;

  int n_theta = n_s * n_feature;
  VectorXd theta = VectorXd::Zero(n_theta);
  theta << 1, 1, 0, 0, 0,
           0, 0, 1, 0, 0,
           0, 0, 0, 1, 1;
  DRAKE_DEMAND(n_theta == theta.size());

  VectorXd theta_q_val(n_theta + n_q);
  theta_q_val << theta, q;

  AutoDiffVecXd theta_q_autoDiff = initializeAutoDiff(theta_q_val);

  ////// getFeature() //////
  // // VectorXd feature = expr.getFeature(q);
  // // cout << "feature = \n" << feature << "\n\n";
  // auto feature_autoDiff = expr.getFeature(q_autoDiff);
  // cout << "feature_autoDiff = \n" << feature_autoDiff << "\n\n";

  ///// getExpression() //////

  // Features implemented in KinematicsExpression should be:
  // feature << q(0),
  //            q(1)*q(1)*q(1),
  //            q(0) * q(1),
  //            cos(q(0)),
  //            sqrt(q(1));

  // expression =
  //      theta(0) * q(0) + theta(1) * q(1)*q(1)*q(1),
  //      theta(5+2) * q(0) * q(1),
  //      theta(10+3) * cos(q(0)) + theta(10+4) * sqrt(q(1));

  AutoDiffVecXd theta_autoDiff = theta_q_autoDiff.head(n_theta);
  AutoDiffVecXd q_autoDiff = theta_q_autoDiff.tail(n_q);
  auto expression_autoDiff = expr.getExpression(theta_autoDiff,q_autoDiff);
  cout << "expression_autoDiff = \n" << expression_autoDiff << "\n\n";

  // Checking autoDiff
  MatrixXd jacobian  = autoDiffToGradientMatrix(expression_autoDiff);
  cout << "jacobian = \n";
  for(int i=0; i<jacobian.rows(); i++){
    cout << i << "-th row: " << jacobian.row(i) << "\n\n";
  }*/


  return 0;
}
