#include "examples/Cassie/cassie_utils.h"
#include "common/find_resource.h"
#include "drake/solvers/mathematical_program.h"

namespace dairlib {
using Eigen::VectorXd;
using Eigen::Vector3d;
using drake::solvers::Constraint;
using drake::AutoDiffVecXd;
using drake::solvers::MathematicalProgram;

std::unique_ptr<RigidBodyTree<double>> makeFixedBaseCassieTreePointer(
    std::string filename) {
  auto tree = std::make_unique<RigidBodyTree<double>>();
  buildFixedBaseCassieTree(*tree.get());
  return tree;
}

void buildFixedBaseCassieTree(RigidBodyTree<double>& tree,
                              std::string filename) {
  drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      FindResourceOrThrow(filename),
      drake::multibody::joints::kFixed, &tree);

  // Add distance constraints for the two legs
  double achilles_length = .5012;
  int heel_spring_left = tree.FindBodyIndex("heel_spring_left");
  int thigh_left = tree.FindBodyIndex("thigh_left");

  int heel_spring_right = tree.FindBodyIndex("heel_spring_right");
  int thigh_right = tree.FindBodyIndex("thigh_right");

  Vector3d rod_on_heel_spring;  // symmetric left and right
  rod_on_heel_spring << .11877, -.01, 0.0;

  Vector3d rod_on_thigh_left;
  rod_on_thigh_left << 0.0, 0.0, 0.045;

  Vector3d rod_on_thigh_right;
  rod_on_thigh_right << 0.0, 0.0, -0.045;


  tree.addDistanceConstraint(heel_spring_left, rod_on_heel_spring,
                             thigh_left, rod_on_thigh_left,
                             achilles_length);

  tree.addDistanceConstraint(heel_spring_right, rod_on_heel_spring,
                           thigh_right, rod_on_thigh_right,
                           achilles_length);
}

VectorXd solvePositionConstraints(const RigidBodyTree<double>& tree,
                                  VectorXd q_init,
                                  std::vector<int> fixed_joints) {
  MathematicalProgram prog;
  auto q = prog.NewContinuousVariables(tree.get_num_positions(), "q");
  auto constraint = std::make_shared<TreePositionConstraint>(tree);
  prog.AddConstraint(constraint, q);
  for (uint i = 0; i < fixed_joints.size(); i++) {
    int j = fixed_joints[i];
    prog.AddConstraint(q(j) == q_init(j));
  }
  prog.AddQuadraticCost((q - q_init).dot(q - q_init));
  prog.SetInitialGuessForAllVariables(q_init);
  prog.Solve();
  return prog.GetSolution(q);
}

TreePositionConstraint::TreePositionConstraint(
    const RigidBodyTree<double>& tree, const std::string& description) :
    Constraint(tree.getNumPositionConstraints(),
               tree.get_num_positions(),
               VectorXd::Zero(tree.getNumPositionConstraints()),
               VectorXd::Zero(tree.getNumPositionConstraints()),
               description) {
  tree_ = &tree;
}

void TreePositionConstraint::DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                                    Eigen::VectorXd* y) const {
  AutoDiffVecXd y_t;
  Eval(drake::math::initializeAutoDiff(x), &y_t);
  *y = drake::math::autoDiffToValueMatrix(y_t);
}

void TreePositionConstraint::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                                    AutoDiffVecXd* y) const {
  const AutoDiffVecXd q = x.head(tree_->get_num_positions());
  KinematicsCache<drake::AutoDiffXd> cache = tree_->doKinematics(q);
  *y = tree_->positionConstraints(cache);
}

void TreePositionConstraint::DoEval(
    const Eigen::Ref<const drake::VectorX<drake::symbolic::Variable>>& x,
    drake::VectorX<drake::symbolic::Expression>* y) const {
  throw std::logic_error(
      "TreePositionConstraint does not support symbolic evaluation.");
}

}  // namespace dairlib
