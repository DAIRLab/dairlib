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
#include "dircon.h"

using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix3Xd;
using std::cout;
using std::endl;
using drake::systems::trajectory_optimization::DirconDynamicConstraint;
using drake::systems::trajectory_optimization::DirconKinematicConstraint;


//template VectorXd RigidBodyTree<double>::transformPointsJacobianDotTimesV<double, Matrix3Xd>(KinematicsCache<double> const&, Eigen::MatrixBase<Matrix3Xd> const&, int, int);

namespace drake{
namespace goldilocks {
namespace examples {
namespace {

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  //auto model = std::make_unique<RigidBodyTree<double>>();
  RigidBodyTree<double> tree;
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld("../../examples/Acrobot/Acrobot.urdf", multibody::joints::kRollPitchYaw, &tree);
  //const std::unique_ptr<const RigidBodyTree<double>> tree =  std::unique_ptr<const RigidBodyTree<double>>(&model);

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

  VectorXd q1(n,1);
  VectorXd v1(n,1);
  q1 << 0,1,0,1,0,1,1,0;
  v1 << 1,1,1,-1,-1,0,1,0;
  VectorXd x1(2*n,1);
  x1 << q1, v1;
  VectorXd u1(nu,1);
  u1 << -1;
  VectorXd l1(nl,1);
  l1 << 2, 1, -2, 3;
  VectorXd lc(nl,1);
  lc << 2, 2, -1, 0;
  VectorXd vc(nl,1);
  vc << 2, -1, 0, 1;

  VectorXd h(1,1);
  h << .1;

  VectorXd vars(1+4*n + 2*nu + 4*nl);
  vars << h,x,x1,u,u1,l,l1,lc,vc;

  AutoDiffVecXd vars_autodiff = math::initializeAutoDiff(vars);
  AutoDiffVecXd x_autodiff = vars_autodiff.segment(1,2*n);  
  AutoDiffVecXd q_autodiff = x_autodiff.head(n);
  AutoDiffVecXd v_autodiff = x_autodiff.tail(n);

  AutoDiffVecXd u_autodiff = vars_autodiff.segment(2*n+1, nu);
  AutoDiffVecXd l_autodiff = vars_autodiff.segment(2*n+1 + nu, nl);

  std::vector<DirconKinematicData<AutoDiffXd>*> constraintsd;
  constraintsd.push_back(&constraintd);
  constraintsd.push_back(&constraintd2);
  auto datasetd = DirconKinematicDataSet<AutoDiffXd>(tree, &constraintsd);

  //auto dataset_ptr =  std::unique_ptr<DirconKinematicDataSet<AutoDiffXd>>(&datasetd);

  datasetd.updateData(x_autodiff, u_autodiff, l_autodiff);

  auto dynamicConstraint = std::make_shared<DirconDynamicConstraint>(tree, datasetd);
  auto kinematicConstraint = std::make_shared<DirconKinematicConstraint>(tree, datasetd);


  //AutoDiffVecXd xul 
  //VectorXd y(2*n);
  AutoDiffVecXd x_dynamic = vars_autodiff;
  AutoDiffVecXd y_dynamic;// = y.template cast<AutoDiffXd>();
  //std::cout << dynamicY << endl;
  //std::cout << x_dynamic.rows() << " " << dynamicConstraint.num_vars() << endl;
  dynamicConstraint->Eval(x_dynamic,y_dynamic);

  cout << "*********** y_dynamic  ***********" << endl;
  std::cout << y_dynamic << endl;
  cout << "*********** dy_dynamic  ***********" << endl;
  std::cout << math::autoDiffToGradientMatrix(y_dynamic) << endl;

  AutoDiffVecXd x_kinematic = VectorX<AutoDiffXd>(2*n+nu+nl);
  x_kinematic << x_autodiff, u_autodiff, l_autodiff;
  AutoDiffVecXd y_kinematic;

  kinematicConstraint->Eval(x_kinematic,y_kinematic);

  cout << "*********** y_kinematic  ***********" << endl;
  std::cout << y_kinematic << endl;
  cout << "*********** dy_kinematic  ***********" << endl;
  std::cout << math::autoDiffToGradientMatrix(y_kinematic) << endl;

  return 0;
}

}  // namespace
}  // namespace examples
}  // namespace goldilocks
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::goldilocks::examples::do_main(argc, argv);
}