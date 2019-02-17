#include <memory>

#include <gflags/gflags.h>
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/kinematics_cache.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "dircon_position_data.h"
#include "dircon_kinematic_data_set.h"

using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix3Xd;
using std::cout;
using std::endl;

//template VectorXd RigidBodyTree<double>::transformPointsJacobianDotTimesV<double, Matrix3Xd>(KinematicsCache<double> const&, Eigen::MatrixBase<Matrix3Xd> const&, int, int);

namespace dairlib{
namespace {

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  RigidBodyTree<double> tree;
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld("../../examples/Acrobot/Acrobot.urdf", multibody::joints::kRollPitchYaw, &tree);

  cout << tree.get_num_positions() << endl;
  cout << tree.get_num_actuators() << endl;
  //return 0;
  //cout << tree.getBodyOrFrameName(1) << endl;
  //cout << tree.getBodyOrFrameName(2) << endl;
  //cout << tree.getBodyOrFrameName(3) << endl;

  int bodyIdx = 3;
  Vector3d pt;
  pt << 0,0,-1;
  Vector3d pt2;
  pt << 1,0,1;
  bool isXZ = true;
  DirconPositionData<AutoDiffXd> constraintd = DirconPositionData<AutoDiffXd>(tree,bodyIdx,pt,isXZ);
  DirconPositionData<AutoDiffXd> constraintd2 = DirconPositionData<AutoDiffXd>(tree,bodyIdx,pt2,isXZ);
  DirconPositionData<double> constraint = DirconPositionData<double>(tree,bodyIdx,pt,isXZ);
  DirconPositionData<double> constraint2 = DirconPositionData<double>(tree,bodyIdx,pt2,isXZ);

  int n = 8;
  int nl = 4;
  int nu = 1;

  VectorXd q(n,1);
  VectorXd v(n,1);
  q << 0,0,0,0,0,0,0,0;
  v << 1,1,1,1,1,0,0,0;
  VectorXd x(2*n,1);
  x << q, v;
  VectorXd u(nu,1);
  u << 2;
  VectorXd l(nl,1);
  l << -2, 2, 1, 1;

  VectorXd vars(2*n + nu + nl);
  vars << x,u,l;

  AutoDiffVecXd vars_autodiff = math::initializeAutoDiff(vars);
  AutoDiffVecXd x_autodiff = vars_autodiff.head(2*n);  
  AutoDiffVecXd q_autodiff = x_autodiff.head(n);
  AutoDiffVecXd v_autodiff = x_autodiff.tail(n);

  AutoDiffVecXd u_autodiff = vars_autodiff.segment(2*n, nu);
  AutoDiffVecXd l_autodiff = vars_autodiff.segment(2*n + nu, nl);


  KinematicsCache<AutoDiffXd> cache = tree.doKinematics(q_autodiff,v_autodiff,true);
  constraintd.updateConstraint(cache);

  auto c = constraintd.getC();
  auto cdot = constraintd.getCDot();
  auto J = constraintd.getJ();
  auto Jdotv = constraintd.getJdotv();
  cout << "***********c ***********" << endl;
  cout << c << endl;

  cout << "***********J ***********" << endl;
  cout << J << endl;  

  cout << "***********cdot ***********" << endl;
  cout << cdot << endl;  

  cout << "***********Jdotv ***********" << endl;
  cout << Jdotv << endl;  

  cout << "***********dc ***********" << endl;
  cout << math::autoDiffToGradientMatrix(c) << endl;

  cout << "***********dcdot ***********" << endl;
  cout << math::autoDiffToGradientMatrix(cdot) << endl;  

  cout << "***********dJdotv ***********" << endl;
  cout << math::autoDiffToGradientMatrix(Jdotv) << endl;

  std::vector<DirconKinematicData<AutoDiffXd>*> constraintsd;
  constraintsd.push_back(&constraintd);
  constraintsd.push_back(&constraintd2);
  auto datasetd = DirconKinematicDataSet<AutoDiffXd>(tree, &constraintsd);

  datasetd.updateData(x_autodiff, u_autodiff, l_autodiff);

  cout << "***********c ***********" << endl;
  cout << datasetd.getC() << endl;

  cout << "***********J ***********" << endl;
  cout << datasetd.getJ() << endl;  

  cout << "***********cdot ***********" << endl;
  cout << datasetd.getCDot() << endl;  

  cout << "***********Jdotv ***********" << endl;
  cout << datasetd.getJdotv() << endl;  

  cout << "***********dc ***********" << endl;
  cout << math::autoDiffToGradientMatrix(datasetd.getC()) << endl;

  cout << "***********dcdot ***********" << endl;
  cout << math::autoDiffToGradientMatrix(datasetd.getCDot()) << endl;  

  cout << "***********dJdotv ***********" << endl;
  cout << math::autoDiffToGradientMatrix(datasetd.getJdotv()) << endl;

  cout << "***********vdot ***********" << endl;
  cout << datasetd.getVDot() << endl;
  cout << "***********xdot ***********" << endl;
  cout << datasetd.getXDot() << endl;


  std::vector<DirconKinematicData<double>*> constraints;
  constraints.push_back(&constraint);
  constraints.push_back(&constraint2);
  auto dataset = DirconKinematicDataSet<double>(tree, &constraints);

  dataset.updateData(x, u, l);




  //TODO: add tests for constraint set and dircondynamicconstraint, probably should make these constraints more than just a does-run check (check values)

/*
  Matrix3Xd xA, xB, normal;
  std::vector<int> idxA;
  std::vector<int> idxB;
  VectorXd phi;
  
  
  tree.get()->collisionDetect(cache, phi, normal, xA, xB, idxA, idxB);  
  cout << "Phi: " << phi << endl << endl;
  cout << "normal: " << normal << endl << endl;
  cout << "xA: " << xA << endl << endl;
  cout << "xB: " << xB << endl << endl; 


  int n = 6;
  //AutoDiffVecXd q_autodiff = math::initializeAutoDiff(q);
  //AutoDiffVecXd v_autodiff = math::initializeAutoDiff(v);

  VectorXd x(2*n);
  x << q, v;
  AutoDiffVecXd x_autodiff = math::initializeAutoDiff(x);  
  AutoDiffVecXd q_autodiff = x_autodiff.head(n);
  AutoDiffVecXd v_autodiff = x_autodiff.tail(n);


  //cout << "qad: " << q_autodiff << endl;
  KinematicsCache<AutoDiffXd> cache_autodiff = tree.get()->doKinematics(q_autodiff,v_autodiff,true);

  AutoDiffVecXd pts = tree.get()->transformPoints(cache_autodiff,xB.col(0),idxB[0],0);
  cout << "***********PTS ***********" << endl;
  cout << pts << endl;
  cout << "***********Derivatives***********" << endl;
  cout << math::autoDiffToGradientMatrix(pts) << endl;  

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
  AutoDiffVecXd JdotV = tree.get()->transformPointsJacobianDotTimesV(cache_autodiff, xB_col0, idxB.at(0), 0); 

  cout << "*********** JdotV ***********" << endl;
  cout << JdotV << endl;

  cout << "*********** JdotV derivatives ***********" << endl;
  cout << math::autoDiffToGradientMatrix(JdotV) << endl;

  cout << "*********** JdotV derivatives ***********" << endl;
  cout << math::autoDiffToGradientMatrix(JdotV) << endl;


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
  surfaceTangents(normal.col(0),d);

  cout << "*********** d ***********" << endl;
  cout << d << endl;  

  cout << "*********** d_again ***********" << endl;
  cout << d_world[1] << endl;  

  // see https://github.com/RobotLocomotdion/drake/blob/master/multibody/rigid_body_constraint.cc#L1930~L1959
  // for how to generat ejacobian
  */
  return 0;
}

}  // namespace
}  // namespace dairlib

int main(int argc, char* argv[]) {
  return drake::goldilocks::examples::do_main(argc, argv);
}