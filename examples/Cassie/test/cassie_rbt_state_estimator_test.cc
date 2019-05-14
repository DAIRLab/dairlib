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
using Eigen::MatrixXd;
using Eigen::Matrix3Xd;
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
  CassieRbtStateEstimator estimator(tree_fixed_, VectorXd::Zero(27),
                                    VectorXd::Zero(2), false);

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

// The Matrix functions are only tested for the rpy tree version as they are
// independent of the input tree
TEST_F(CassieRbtStateEstimatorTest, TestExtractRotation) {
  VectorXd ekf_x = VectorXd::Random(27);
  VectorXd ekf_b = VectorXd::Random(2);

  CassieRbtStateEstimator estimator(tree_rpy_, ekf_x, ekf_b, true);
  MatrixXd R = estimator.ExtractRotationMatrix(ekf_x);

  ASSERT_EQ(R(0, 0), ekf_x(0));
  ASSERT_EQ(R(0, 1), ekf_x(1));
  ASSERT_EQ(R(0, 2), ekf_x(2));
  ASSERT_EQ(R(1, 0), ekf_x(3));
  ASSERT_EQ(R(1, 1), ekf_x(4));
  ASSERT_EQ(R(1, 2), ekf_x(5));
  ASSERT_EQ(R(2, 0), ekf_x(6));
  ASSERT_EQ(R(2, 1), ekf_x(7));
  ASSERT_EQ(R(2, 2), ekf_x(8));
}

TEST_F(CassieRbtStateEstimatorTest, TestExtractVelocity) {
  VectorXd ekf_x = VectorXd::Random(27);
  VectorXd ekf_b = VectorXd::Random(2);

  CassieRbtStateEstimator estimator(tree_rpy_, ekf_x, ekf_b, true);
  VectorXd v = estimator.ExtractFloatingBaseVelocities(ekf_x);

  ASSERT_EQ(v(0), ekf_x(9));
  ASSERT_EQ(v(1), ekf_x(10));
  ASSERT_EQ(v(2), ekf_x(11));
}

TEST_F(CassieRbtStateEstimatorTest, TestExtractPosition) {
  VectorXd ekf_x = VectorXd::Random(27);
  VectorXd ekf_b = VectorXd::Random(2);

  CassieRbtStateEstimator estimator(tree_rpy_, ekf_x, ekf_b, true);
  VectorXd p = estimator.ExtractFloatingBasePositions(ekf_x);

  ASSERT_EQ(p(0), ekf_x(12));
  ASSERT_EQ(p(1), ekf_x(13));
  ASSERT_EQ(p(2), ekf_x(14));
}

TEST_F(CassieRbtStateEstimatorTest, TestExtractContactFull) {
  VectorXd ekf_x = VectorXd::Random(27);
  VectorXd ekf_b = VectorXd::Random(2);

  CassieRbtStateEstimator estimator(tree_rpy_, ekf_x, ekf_b, true);
  MatrixXd d = estimator.ExtractContactPositions(ekf_x);

  ASSERT_EQ(d(0, 0), ekf_x(15));
  ASSERT_EQ(d(1, 0), ekf_x(16));
  ASSERT_EQ(d(2, 0), ekf_x(17));
  ASSERT_EQ(d(0, 1), ekf_x(18));
  ASSERT_EQ(d(1, 1), ekf_x(19));
  ASSERT_EQ(d(2, 1), ekf_x(20));
  ASSERT_EQ(d(0, 2), ekf_x(21));
  ASSERT_EQ(d(1, 2), ekf_x(22));
  ASSERT_EQ(d(2, 2), ekf_x(23));
  ASSERT_EQ(d(0, 3), ekf_x(24));
  ASSERT_EQ(d(1, 3), ekf_x(25));
  ASSERT_EQ(d(2, 3), ekf_x(26));
}

TEST_F(CassieRbtStateEstimatorTest, TestExtractContactPartial) {
  VectorXd ekf_x = VectorXd::Random(27);
  VectorXd ekf_b = VectorXd::Random(2);

  CassieRbtStateEstimator estimator(tree_rpy_, ekf_x, ekf_b, true);

  // States with less than four contacts (Different permuations)
  VectorXd ekf_x_partial1 = ekf_x;
  VectorXd ekf_x_partial2 = ekf_x;
  VectorXd ekf_x_partial3 = ekf_x;
  VectorXd ekf_x_partial4 = ekf_x;
  VectorXd ekf_x_partial5 = ekf_x;

  // No contacts
  ekf_x_partial1.segment(15, 12) = -VectorXd::Ones(12);
  // No first contact
  ekf_x_partial2.segment(15, 3) = -VectorXd::Ones(3);
  // No second contact
  ekf_x_partial3.segment(18, 3) = -VectorXd::Ones(3);
  // No first and third contact
  ekf_x_partial4.segment(15, 3) = -VectorXd::Ones(3);
  ekf_x_partial4.segment(21, 3) = -VectorXd::Ones(3);
  // No first, second and third contact
  ekf_x_partial5.segment(15, 3) = -VectorXd::Ones(3);
  ekf_x_partial5.segment(18, 3) = -VectorXd::Ones(3);
  ekf_x_partial5.segment(21, 3) = -VectorXd::Ones(3);

  // Computing d
  MatrixXd d1 = estimator.ExtractContactPositions(ekf_x_partial1);
  MatrixXd d2 = estimator.ExtractContactPositions(ekf_x_partial2);
  MatrixXd d3 = estimator.ExtractContactPositions(ekf_x_partial3);
  MatrixXd d4 = estimator.ExtractContactPositions(ekf_x_partial4);
  MatrixXd d5 = estimator.ExtractContactPositions(ekf_x_partial5);

  ASSERT_TRUE(d1.cols() == 0);
  ASSERT_TRUE(d2.cols() == 3);
  ASSERT_TRUE(d3.cols() == 3);
  ASSERT_TRUE(d4.cols() == 2);
  ASSERT_TRUE(d5.cols() == 1);

  ASSERT_TRUE(d2.col(0).isApprox(ekf_x_partial2.segment(18, 3)));
  ASSERT_TRUE(d2.col(1).isApprox(ekf_x_partial2.segment(21, 3)));
  ASSERT_TRUE(d2.col(2).isApprox(ekf_x_partial2.segment(24, 3)));

  ASSERT_TRUE(d3.col(0).isApprox(ekf_x_partial3.segment(15, 3)));
  ASSERT_TRUE(d3.col(1).isApprox(ekf_x_partial3.segment(21, 3)));
  ASSERT_TRUE(d3.col(2).isApprox(ekf_x_partial3.segment(24, 3)));

  ASSERT_TRUE(d4.col(0).isApprox(ekf_x_partial4.segment(18, 3)));
  ASSERT_TRUE(d4.col(1).isApprox(ekf_x_partial4.segment(24, 3)));

  ASSERT_TRUE(d5.col(0).isApprox(ekf_x_partial5.segment(24, 3)));
}

TEST_F(CassieRbtStateEstimatorTest, TestSkewSymmetric) {
  VectorXd ekf_x = VectorXd::Random(27);
  VectorXd ekf_b = VectorXd::Random(2);

  CassieRbtStateEstimator estimator(tree_rpy_, ekf_x, ekf_b, true);

  MatrixXd S = estimator.CreateSkewSymmetricMatrix(VectorXd::Random(3));
  ASSERT_TRUE(S.isApprox(-S.transpose()));

}

}  // namespace
}  // namespace systems
}  // namespace dairlib

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
