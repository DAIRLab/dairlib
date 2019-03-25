#include "examples/Cassie/cassie_rbt_state_estimator.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"
#include "attic/multibody/multibody_solvers.h"

#include <memory>
#include <utility>

#include <gtest/gtest.h>

namespace dairlib {
namespace systems {
namespace {

using drake::systems::BasicVector;
using std::make_unique;
using Eigen::VectorXd;
using dairlib::multibody::PositionSolver;

class CassieRbtStateEstimatorTest : public ::testing::Test {
};

TEST_F(CassieRbtStateEstimatorTest, solveFourbarLinkageTest) {
  RigidBodyTree<double> tree;
  drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
    "examples/Cassie/urdf/cassie_v2.urdf",
    drake::multibody::joints::kFixed, &tree);

  CassieRbtStateEstimator estimator(tree);

  // Example position
  VectorXd q_init;
  q_init << -0.084017,
         0.0826151,
         -0.00120735,
         0.00217829,
         0.366012,
         0.365803,
         -0.6305,
         -0.630502,
         0.00205363,
         0.00205356,
         0.838878,
         0.838882,
         0,
         0,
         0.205351,
         0.20456;
  double calculated_left_heel_spring, calculated_right_heel_spring;

  // Get the angles analytically
  estimator.solveFourbarLinkage(q_init,
                    calculated_left_heel_spring, calculated_right_heel_spring);

  // Get the angles from nonlinear programming
  std::map<std::string, int> positionIndexMap =
      multibody::makeNameToPositionsMap(tree);
  std::vector<int> fixed_joints;
  fixed_joints.push_back(positionIndexMap.at("knee_left"));
  fixed_joints.push_back(positionIndexMap.at("knee_joint_left"));
  fixed_joints.push_back(positionIndexMap.at("ankle_joint_left"));
  fixed_joints.push_back(positionIndexMap.at("knee_right"));
  fixed_joints.push_back(positionIndexMap.at("knee_joint_right"));
  fixed_joints.push_back(positionIndexMap.at("ankle_joint_right"));

  PositionSolver position_solver(tree, q_init);
  position_solver.SetInitialGuessQ(q_init);

  std::map<int, double> fixed_joints_map;
  for (auto& ind : fixed_joints) {
    fixed_joints_map[ind] = q_init(ind);
  }
  position_solver.AddFixedJointsConstraint(fixed_joints_map);

  position_solver.Solve();
  VectorXd q_sol = position_solver.GetSolutionQ();

  double nlp_left_heel_spring =
    q_sol(positionIndexMap.at("ankle_spring_joint_left"));
  double nlp_right_heel_spring =
    q_sol(positionIndexMap.at("ankle_spring_joint_right"));

  ASSERT_EQ(calculated_left_heel_spring, nlp_left_heel_spring);
  ASSERT_EQ(calculated_right_heel_spring, nlp_right_heel_spring);

}


}  // namespace
}  // namespace systems
}  // namespace dairlib


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}