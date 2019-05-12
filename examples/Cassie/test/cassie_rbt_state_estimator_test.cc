#include "examples/Cassie/cassie_rbt_state_estimator.h"
#include <gtest/gtest.h>
#include "attic/multibody/multibody_solvers.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"

namespace dairlib {
namespace systems {
namespace {

using std::abs;
using Eigen::VectorXd;
using dairlib::multibody::PositionSolver;
using drake::multibody::joints::kFixed;
using drake::multibody::joints::kRollPitchYaw;
using drake::multibody::joints::kQuaternion;

class CassieRbtStateEstimatorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    std::string filename = "examples/Cassie/urdf/cassie_v2.urdf";
    buildCassieTree(tree_fixed_, filename, kFixed);
    buildCassieTree(tree_rpy_, filename, kRollPitchYaw);
    buildCassieTree(tree_quaternion_, filename, kQuaternion);

    num_positions_fixed_ = tree_fixed_.get_num_positions();
    num_velocities_fixed_ = tree_fixed_.get_num_velocities();
    num_states_fixed_ = num_positions_fixed_ + num_velocities_fixed_;

    num_positions_rpy_ = tree_rpy_.get_num_positions();
    num_velocities_rpy_ = tree_rpy_.get_num_velocities();
    num_states_rpy_ = num_positions_rpy_ + num_velocities_rpy_;

    num_positions_quaternion_ = tree_quaternion_.get_num_positions();
    num_velocities_quaternion_ = tree_quaternion_.get_num_velocities();
    num_states_quaternion_ =
        num_positions_quaternion_ + num_velocities_quaternion_;
  }

  RigidBodyTree<double> tree_fixed_;
  RigidBodyTree<double> tree_rpy_;
  RigidBodyTree<double> tree_quaternion_;
  int num_positions_fixed_;
  int num_positions_rpy_;
  int num_positions_quaternion_;
  int num_velocities_fixed_;
  int num_velocities_rpy_;
  int num_velocities_quaternion_;
  int num_states_fixed_;
  int num_states_rpy_;
  int num_states_quaternion_;
};

TEST_F(CassieRbtStateEstimatorTest, solveFourbarLinkageTest) {
  CassieRbtStateEstimator estimator(tree_fixed_, false);

  // Example position
  VectorXd q_init(num_positions_fixed_);
  q_init << -0.084017, 0.0826151, -0.00120735, 0.00217829, 0.366012, 0.365803,
      -0.6305, -0.630502, 0.00205363, 0.00205356, 0.838878, 0.838882, 0, 0,
      0.205351, 0.20456;

  // Get the angles analytically
  double calc_left_heel_spring, calc_right_heel_spring;
  estimator.solveFourbarLinkage(q_init, calc_left_heel_spring,
                                calc_right_heel_spring);

  // Get the angles from nonlinear programming
  std::map<std::string, int> positionIndexMap =
      multibody::makeNameToPositionsMap(tree_fixed_);
  std::vector<int> fixed_joints;
  fixed_joints.push_back(positionIndexMap.at("knee_left"));
  fixed_joints.push_back(positionIndexMap.at("knee_joint_left"));
  fixed_joints.push_back(positionIndexMap.at("ankle_joint_left"));
  fixed_joints.push_back(positionIndexMap.at("knee_right"));
  fixed_joints.push_back(positionIndexMap.at("knee_joint_right"));
  fixed_joints.push_back(positionIndexMap.at("ankle_joint_right"));

  PositionSolver position_solver(tree_fixed_, q_init);
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

  EXPECT_TRUE(abs(calc_left_heel_spring - nlp_left_heel_spring) < 1e-10);
  EXPECT_TRUE(abs(calc_right_heel_spring - nlp_right_heel_spring) < 1e-10);
}

}  // namespace
}  // namespace systems
}  // namespace dairlib

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
