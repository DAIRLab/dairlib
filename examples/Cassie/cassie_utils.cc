#include "examples/Cassie/cassie_utils.h"
#include "common/find_resource.h"
#include "drake/solvers/mathematical_program.h"

namespace dairlib {
using Eigen::VectorXd;
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
                                    Eigen::VectorXd& y) const {
  AutoDiffVecXd y_t;
  Eval(drake::math::initializeAutoDiff(x), y_t);
  y = drake::math::autoDiffToValueMatrix(y_t);
}

void TreePositionConstraint::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                                    AutoDiffVecXd& y) const {
  KinematicsCache<drake::AutoDiffXd> cache = tree_->doKinematics((AutoDiffVecXd) x);
  y = tree_->positionConstraints(cache);
}

}