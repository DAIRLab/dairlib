#include <memory>

#include <gflags/gflags.h>
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/kinematics_cache.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::Matrix3Xd;
//using Eigen::MatrixX;
using std::cout;
using std::endl;
using drake::AutoDiffVecXd;
using drake::math::autoDiffToGradientMatrix;
using drake::math::initializeAutoDiff;
using drake::AutoDiffXd;

namespace dairlib {
namespace {

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  drake::lcm::DrakeLcm lcm;
  auto tree = std::make_unique<RigidBodyTree<double>>();
  drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      "PlanarWalker.urdf",
      drake::multibody::joints::kFixed, tree.get());
  drake::multibody::AddFlatTerrainToWorld(tree.get(), 100., 10.);  

  Matrix3Xd xA, xB, normal;
  std::vector<int> idxA;
  std::vector<int> idxB;
  VectorXd q(6,1);
  VectorXd v(6,1);
  q << 0,1,1,0,0,0;
  v << 1,1,1,1,1,0; 
  VectorXd phi;
  
  KinematicsCache<double> cache = tree.get()->doKinematics(q,v,true);
  tree.get()->collisionDetect(cache, phi, normal, xA, xB, idxA, idxB);  
  cout << "Phi: " << phi << endl << endl;
  cout << "normal: " << normal << endl << endl;
  cout << "xA: " << xA << endl << endl;
  cout << "xB: " << xB << endl << endl; 


  int n = 6;
  //AutoDiffVecXd q_autodiff = initializeAutoDiff(q);
  //AutoDiffVecXd v_autodiff = initializeAutoDiff(v);

  VectorXd x(2*n);
  x << q, v;
  AutoDiffVecXd x_autodiff = initializeAutoDiff(x);  
  AutoDiffVecXd q_autodiff = x_autodiff.head(n);
  AutoDiffVecXd v_autodiff = x_autodiff.tail(n);


  //cout << "qad: " << q_autodiff << endl;
  KinematicsCache<AutoDiffXd> cache_autodiff = tree.get()->doKinematics(q_autodiff,v_autodiff,true);

  AutoDiffVecXd pts = tree.get()->transformPoints(cache_autodiff,xB.col(0),idxB[0],0);
  cout << "***********PTS ***********" << endl;
  cout << pts << endl;
  cout << "***********Derivatives***********" << endl;
  cout << autoDiffToGradientMatrix(pts) << endl;  

  int num_pts = static_cast<int>(xA.cols());
  MatrixXd J = MatrixXd::Zero(num_pts, tree.get()->get_num_positions());
  MatrixXd JA = MatrixXd::Zero(3, tree.get()->get_num_positions());
  MatrixXd JB = MatrixXd::Zero(3, tree.get()->get_num_positions());
  for (int i = 0; i < num_pts; ++i) {
    JA = tree.get()->transformPointsJacobian(cache, xA.col(i),
                                                    idxA.at(i), 0, true);
    JB = tree.get()->transformPointsJacobian(cache, xB.col(i),
                                                    idxB.at(i), 0, true);
    J.row(i) = normal.col(i).transpose() * (JA - JB);
  }   

  cout << "*********** J ***********" << endl;
  cout << J << endl;

  Eigen::Vector3d test;
  test << -1, 1, .5;


  //MatrixXd JdotV = tree.get()->transformPointsJacobianDotTimesV(cache, xB.col(0), idxB.at(0), 0);
   //VectorXd JdotV = tree.get()->transformPointsJacobianDotTimesV<double,Eigen::Block<Eigen::Matrix<double, 3, -1, 0, 3, -1>, 3, 1, true>>(cache, xB.col(0), idxB.at(0), 0); 
//  auto JdotV = tree.get()->transformPointsJacobianDotTimesV(cache, Matrix3Xd(xB.col(0)), idxB.at(0), 0); 
  
  //const Eigen::Map<Matrix3Xd> xB_col0(xB.data(), 3, 1);
  const Eigen::Map<const Eigen::Matrix3Xd> xB_col0(xB.data(), 3, 1);
  //auto JdotV = tree.get()->transformPointsJacobianDotTimesV(cache_autodiff, xB_col0, idxB.at(0), 0); 
  const Eigen::Vector3d xBc0 = Eigen::Vector3d(xB.col(0));
  //AutoDiffVecXd tmp = xB.col(0).template cast<AutoDiffXd>();
  auto tmp = xBc0.template cast<AutoDiffXd>();

  cout << "*********** xB0 ***********" << endl;
  cout << xBc0 << endl;
  cout << "*********** xB0 ***********" << endl;
  cout << tmp << endl;
  cout << "*********** xB0 derivatives ***********" << endl;
  cout << autoDiffToGradientMatrix(tmp) << endl;

  auto Jdiff = tree.get()->transformPointsJacobian(cache_autodiff, xBc0.template cast<AutoDiffXd>(), idxB.at(0), 0, true); 

  cout << "*********** Jdiff  ***********" << endl;
  cout << Jdiff << endl;

  auto phidot = Jdiff*v_autodiff;

  cout << "*********** phidot  ***********" << endl;
  cout << phidot << endl;

    cout << "*********** phidot derivatives ***********" << endl;
  cout <<  autoDiffToGradientMatrix(phidot) << endl;

  //AutoDiffVecXd JdotV = tree.get()->transformPointsJacobianDotTimesV(cache_autodiff, xB_col0, idxB.at(0), 0); 
  AutoDiffVecXd JdotV = tree.get()->transformPointsJacobianDotTimesV(cache_autodiff, xBc0, idxB.at(0), 0); 

  cout << "*********** JdotV ***********" << endl;
  cout << JdotV << endl;

  cout << "*********** JdotV derivatives ***********" << endl;
  cout << autoDiffToGradientMatrix(JdotV) << endl;

  cout << "*********** JdotV derivatives ***********" << endl;
  cout << autoDiffToGradientMatrix(JdotV) << endl;


  const Eigen::Map<Eigen::Matrix3Xd> n_world(normal.data(),3,2);
  std::vector<Eigen::Map<Matrix3Xd>> d_world;
  Matrix3Xd d1 = Matrix3Xd::Zero(3,2);
  Matrix3Xd d2 = Matrix3Xd::Zero(3,2);
  Eigen::Map<Matrix3Xd> d1map(d1.data(),3,2);
  Eigen::Map<Matrix3Xd> d2map(d2.data(),3,2);
  d_world.push_back(d1map);
  d_world.push_back(d2map);
  tree.get()->surfaceTangents(n_world, d_world);

  Eigen::Matrix<double,3,2> d;
  //  surfaceTangents(normal.col(0),d);

  cout << "*********** d ***********" << endl;
  cout << d << endl;

  cout << "*********** d_again ***********" << endl;
  cout << d_world[1] << endl;

  // see https://github.com/RobotLocomotion/drake/blob/master/multibody/rigid_body_constraint.cc#L1930~L1959
  // for how to generat ejacobian
  return 0;
}

}  // namespace
}  // namespace dairlib

int main(int argc, char* argv[]) {
  return dairlib::do_main(argc, argv);
}