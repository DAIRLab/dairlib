#include <memory>

#include <gflags/gflags.h>
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/manipulation/util/sim_diagram_builder.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/kinematics_cache.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix3Xd;
using std::cout;
using std::endl;

//template VectorXd RigidBodyTree<double>::transformPointsJacobianDotTimesV<double, Matrix3Xd>(KinematicsCache<double> const&, Eigen::MatrixBase<Matrix3Xd> const&, int, int);

namespace drake{
namespace goldilocks {
namespace examples {
namespace planarwalker {
namespace {

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  lcm::DrakeLcm lcm;
  auto tree = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      "PlanarWalker.urdf",
      multibody::joints::kFixed, tree.get());
  multibody::AddFlatTerrainToWorld(tree.get(), 100., 10.);  

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

  AutoDiffVecXd q_autodiff = math::initializeAutoDiff(q);
  AutoDiffVecXd v_autodiff = math::initializeAutoDiff(v);

  


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
  
  const Eigen::Map<Matrix3Xd> xB_col0(xB.data(), 3, 1);
  //auto JdotV = tree.get()->transformPointsJacobianDotTimesV(cache_autodiff, xB_col0, idxB.at(0), 0); 
  auto JdotV = tree.get()->transformPointsJacobianDotTimesV(cache_autodiff, xB_col0, idxB.at(0), 0); 

  cout << "*********** JdotV ***********" << endl;
  cout << JdotV << endl;

  //cout << "*********** JdotV derivatives ***********" << endl;
  //cout << math::autoDiffToGradientMatrix(JdotV) << endl;

  // see https://github.com/RobotLocomotion/drake/blob/master/multibody/rigid_body_constraint.cc#L1930~L1959
  // for how to generat ejacobian
  return 0;
}

}  // namespace
}  // namespace planarwalker
}  // namespace examples
}  // namespace goldilocks
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::goldilocks::examples::planarwalker::do_main(argc, argv);
}