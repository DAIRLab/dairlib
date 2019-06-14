#include "examples/Cassie/cassie_rbt_state_estimator.h"
#include <gtest/gtest.h>
#include "attic/multibody/multibody_solvers.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"

namespace dairlib {
namespace systems {
namespace {

using std::abs;
using std::map;
using std::vector;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix3Xd;
using Eigen::Transform;
using Eigen::Isometry;
using dairlib::multibody::ContactInfo;
using dairlib::multibody::ContactSolver;
using dairlib::multibody::GetBodyIndexFromName;
using dairlib::multibody::PositionSolver;
using drake::multibody::joints::kFixed;
using drake::multibody::joints::kRollPitchYaw;
using drake::multibody::joints::kQuaternion;
using drake::solvers::MathematicalProgramResult;

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

    // Initial configuration for the solvers
    map<string, int> position_map_rpy =
        tree_rpy_.computePositionNameToIndexMap();
    x0_rpy_ = VectorXd::Zero(num_states_rpy_);
    x0_rpy_(position_map_rpy.at("base_z")) = 3;
    x0_rpy_(position_map_rpy.at("hip_roll_left")) = 0.1;
    x0_rpy_(position_map_rpy.at("hip_roll_right")) = -0.1;
    x0_rpy_(position_map_rpy.at("hip_yaw_left")) = 0.01;
    x0_rpy_(position_map_rpy.at("hip_yaw_right")) = -0.01;
    x0_rpy_(position_map_rpy.at("hip_pitch_left")) = .269;
    x0_rpy_(position_map_rpy.at("hip_pitch_right")) = .269;
    x0_rpy_(position_map_rpy.at("knee_left")) = -.744;
    x0_rpy_(position_map_rpy.at("knee_right")) = -.744;
    x0_rpy_(position_map_rpy.at("ankle_joint_left")) = .81;
    x0_rpy_(position_map_rpy.at("ankle_joint_right")) = .81;
    x0_rpy_(position_map_rpy.at("toe_left")) = -60.0 * M_PI / 180.0;
    x0_rpy_(position_map_rpy.at("toe_right")) = -60.0 * M_PI / 180.0;

    map<string, int> position_map_quaternion =
        tree_quaternion_.computePositionNameToIndexMap();
    x0_quaternion_ = VectorXd::Zero(num_states_quaternion_);
    x0_quaternion_(position_map_quaternion.at("base_z")) = 3;
    x0_quaternion_(position_map_quaternion.at("base_qw")) = 1.0;
    x0_quaternion_(position_map_quaternion.at("hip_roll_left")) = 0.1;
    x0_quaternion_(position_map_quaternion.at("hip_roll_right")) = -0.1;
    x0_quaternion_(position_map_quaternion.at("hip_yaw_left")) = 0.01;
    x0_quaternion_(position_map_quaternion.at("hip_yaw_right")) = -0.01;
    x0_quaternion_(position_map_quaternion.at("hip_pitch_left")) = .269;
    x0_quaternion_(position_map_quaternion.at("hip_pitch_right")) = .269;
    x0_quaternion_(position_map_quaternion.at("knee_left")) = -.744;
    x0_quaternion_(position_map_quaternion.at("knee_right")) = -.744;
    x0_quaternion_(position_map_quaternion.at("ankle_joint_left")) = .81;
    x0_quaternion_(position_map_quaternion.at("ankle_joint_right")) = .81;
    x0_quaternion_(position_map_quaternion.at("toe_left")) =
        -60.0 * M_PI / 180.0;
    x0_quaternion_(position_map_quaternion.at("toe_right")) =
        -60.0 * M_PI / 180.0;

    local_collision_pt1_.resize(3);
    local_collision_pt2_.resize(3);
    local_collision_pt1_ << -0.0457, 0.112, 0;
    local_collision_pt2_ << 0.088, 0, 0;

    MatrixXd xA(3, 4);
    xA.col(0) = local_collision_pt1_;
    xA.col(1) = local_collision_pt2_;
    xA.col(2) = local_collision_pt1_;
    xA.col(3) = local_collision_pt2_;

    const int toe_left_ind = GetBodyIndexFromName(tree_rpy_, "toe_left");
    const int toe_right_ind = GetBodyIndexFromName(tree_rpy_, "toe_right");

    vector<int> idxA(4);
    idxA.at(0) = toe_left_ind;
    idxA.at(1) = toe_left_ind;
    idxA.at(2) = toe_right_ind;
    idxA.at(3) = toe_right_ind;

    contact_info_ = {xA, idxA};
  }

  RigidBodyTree<double> tree_fixed_;
  RigidBodyTree<double> tree_rpy_;
  RigidBodyTree<double> tree_quaternion_;
  ContactInfo contact_info_;
  VectorXd x0_rpy_;
  VectorXd x0_quaternion_;
  VectorXd local_collision_pt1_;
  VectorXd local_collision_pt2_;
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
  MatrixXd X = MatrixXd::Random(9, 9);
  VectorXd ekf_bias = VectorXd::Random(6);

  CassieRbtStateEstimator estimator(tree_rpy_, X, ekf_bias, true);
  VectorXd ekf_x = estimator.ComputeEkfX(X);
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
  MatrixXd X = MatrixXd::Random(9, 9);
  VectorXd ekf_bias = VectorXd::Random(6);

  CassieRbtStateEstimator estimator(tree_rpy_, X, ekf_bias, true);
  VectorXd ekf_x = estimator.ComputeEkfX(X);
  VectorXd v = estimator.ExtractFloatingBaseVelocities(ekf_x);

  ASSERT_EQ(v(0), ekf_x(9));
  ASSERT_EQ(v(1), ekf_x(10));
  ASSERT_EQ(v(2), ekf_x(11));
}

TEST_F(CassieRbtStateEstimatorTest, TestExtractPosition) {
  MatrixXd X = MatrixXd::Random(9, 9);
  VectorXd ekf_bias = VectorXd::Random(6);

  CassieRbtStateEstimator estimator(tree_rpy_, X, ekf_bias, true);
  VectorXd ekf_x = estimator.ComputeEkfX(X);
  VectorXd p = estimator.ExtractFloatingBasePositions(ekf_x);

  ASSERT_EQ(p(0), ekf_x(12));
  ASSERT_EQ(p(1), ekf_x(13));
  ASSERT_EQ(p(2), ekf_x(14));
}

TEST_F(CassieRbtStateEstimatorTest, TestExtractContact) {
  MatrixXd X = MatrixXd::Random(9, 9);
  VectorXd ekf_bias = VectorXd::Random(6);

  CassieRbtStateEstimator estimator(tree_rpy_, X, ekf_bias, true);
  VectorXd ekf_x = estimator.ComputeEkfX(X);
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

TEST_F(CassieRbtStateEstimatorTest, TestSkewSymmetric) {
  MatrixXd X = MatrixXd::Random(9, 9);
  VectorXd ekf_bias = VectorXd::Random(6);

  CassieRbtStateEstimator estimator(tree_rpy_, X, ekf_bias, true);

  MatrixXd S = estimator.CreateSkewSymmetricMatrix(VectorXd::Random(3));
  ASSERT_TRUE(S.isApprox(-S.transpose()));
}

TEST_F(CassieRbtStateEstimatorTest, TestLieExponential) {
  MatrixXd X = MatrixXd::Random(9, 9);
  VectorXd ekf_bias = VectorXd::Random(6);

  CassieRbtStateEstimator estimator(tree_rpy_, X, ekf_bias, true);

  VectorXd xi = VectorXd::Random(21);
  MatrixXd exp = estimator.ComputeLieExponential(xi);

  // Testing if the exponential map returns a valid state matrix
  ASSERT_TRUE(exp.block(3, 0, 6, 3).isApprox(MatrixXd::Zero(6, 3)));
  ASSERT_TRUE(exp.block(3, 3, 6, 6).isApprox(MatrixXd::Identity(6, 6)));
}

TEST_F(CassieRbtStateEstimatorTest, TestTransformationToeLeft) {
  MatrixXd X = MatrixXd::Random(9, 9);
  VectorXd ekf_bias = VectorXd::Random(6);

  CassieRbtStateEstimator estimator_rpy(tree_rpy_, X, ekf_bias, true);
  CassieRbtStateEstimator estimator_quaternion(tree_quaternion_, X, ekf_bias,
                                               true);

  VectorXd q_rpy = Eigen::VectorXd::Random(tree_rpy_.get_num_positions());
  MatrixXd T_rpy = estimator_rpy.ComputeTransformationToeLeftWrtIMU(q_rpy);

  VectorXd q_quaternion =
      Eigen::VectorXd::Random(tree_quaternion_.get_num_positions());
  MatrixXd T_quaternion =
      estimator_quaternion.ComputeTransformationToeLeftWrtIMU(q_quaternion);

  ASSERT_TRUE(T_rpy.rows() == 4);
  ASSERT_TRUE(T_rpy.cols() == 4);
  ASSERT_TRUE(T_quaternion.rows() == 4);
  ASSERT_TRUE(T_quaternion.cols() == 4);

  q_rpy.head(6) = VectorXd::Random(6);
  q_quaternion.head(7) = VectorXd::Random(7);

  ASSERT_TRUE(
      T_rpy.isApprox(estimator_rpy.ComputeTransformationToeLeftWrtIMU(q_rpy)));
  ASSERT_TRUE(T_quaternion.isApprox(
      estimator_quaternion.ComputeTransformationToeLeftWrtIMU(q_quaternion)));

  // Testing if the transformation matrix gives the correct transformation.
  // First the contact solver is used to solve for a configuration that has the
  // feet in contact. The transformation matrix is then used to verify if the
  // local collision points when transformed lie on the ground plane.

  VectorXd q0_rpy = x0_rpy_.head(num_positions_rpy_);
  ContactSolver contact_solver_rpy(tree_rpy_, contact_info_, q0_rpy);
  contact_solver_rpy.SetInitialGuessQ(q0_rpy);
  contact_solver_rpy.AddJointLimitConstraint(0.001);

  MathematicalProgramResult program_result_rpy = contact_solver_rpy.Solve();
  std::cout << "Contact solver result (Rpy Floating base): "
            << program_result_rpy.get_solution_result() << std::endl;

  VectorXd q_sol_rpy = contact_solver_rpy.GetSolutionQ();
  MatrixXd T_sol_rpy =
      estimator_rpy.ComputeTransformationToeLeftWrtIMU(q_sol_rpy);

  // Rotation matrix between the pelvis and the world
  KinematicsCache<double> cache_rpy = tree_rpy_.doKinematics(q_sol_rpy);
  int world_ind = GetBodyIndexFromName(tree_rpy_, "world");
  int pelvis_ind = GetBodyIndexFromName(tree_rpy_, "pelvis");
  Transform<double, 3, Isometry> T =
      tree_rpy_.relativeTransform(cache_rpy, world_ind, pelvis_ind);
  MatrixXd R = T.matrix().block(0, 0, 3, 3);

  // The local collision points in homogeneous coordinates
  VectorXd local_collision_pt1_hom(4), local_collision_pt2_hom(4);
  local_collision_pt1_hom.head(3) = local_collision_pt1_;
  local_collision_pt1_hom(3) = 1;
  local_collision_pt2_hom.head(3) = local_collision_pt2_;
  local_collision_pt2_hom(3) = 1;

  VectorXd d1_hom = T_sol_rpy * local_collision_pt1_hom;
  VectorXd d2_hom = T_sol_rpy * local_collision_pt2_hom;
  VectorXd d1 = d1_hom.head(3);
  VectorXd d2 = d2_hom.head(3);

  // Contact point in world coordinates
  VectorXd p1 = R * d1 + q_sol_rpy.head(3);
  VectorXd p2 = R * d2 + q_sol_rpy.head(3);

  // Checking if the contact points in world frame are on the ground plane.
  ASSERT_TRUE(p1(2) < 1e-6);
  ASSERT_TRUE(p2(2) < 1e-6);
}

TEST_F(CassieRbtStateEstimatorTest, TestTransformationToeRight) {
  MatrixXd X = MatrixXd::Random(9, 9);
  VectorXd ekf_bias = VectorXd::Random(6);

  CassieRbtStateEstimator estimator_rpy(tree_rpy_, X, ekf_bias, true);
  CassieRbtStateEstimator estimator_quaternion(tree_quaternion_, X, ekf_bias,
                                               true);

  VectorXd q_rpy = Eigen::VectorXd::Random(tree_rpy_.get_num_positions());
  MatrixXd T_rpy = estimator_rpy.ComputeTransformationToeRightWrtIMU(q_rpy);

  VectorXd q_quaternion =
      Eigen::VectorXd::Random(tree_quaternion_.get_num_positions());
  MatrixXd T_quaternion =
      estimator_quaternion.ComputeTransformationToeRightWrtIMU(q_quaternion);

  ASSERT_TRUE(T_rpy.rows() == 4);
  ASSERT_TRUE(T_rpy.cols() == 4);

  ASSERT_TRUE(T_quaternion.rows() == 4);
  ASSERT_TRUE(T_quaternion.cols() == 4);

  q_rpy.head(6) = VectorXd::Random(6);
  q_quaternion.head(7) = VectorXd::Random(7);

  ASSERT_TRUE(
      T_rpy.isApprox(estimator_rpy.ComputeTransformationToeRightWrtIMU(q_rpy)));
  ASSERT_TRUE(T_quaternion.isApprox(
      estimator_quaternion.ComputeTransformationToeRightWrtIMU(q_quaternion)));

  // Testing if the transformation matrix gives the correct transformation.
  // First the contact solver is used to solve for a configuration that has the
  // feet in contact. The transformation matrix is then used to verify if the
  // local collision points when transformed lie on the ground plane.

  VectorXd q0_rpy = x0_rpy_.head(num_positions_rpy_);
  ContactSolver contact_solver_rpy(tree_rpy_, contact_info_, q0_rpy);
  contact_solver_rpy.SetInitialGuessQ(q0_rpy);
  contact_solver_rpy.AddJointLimitConstraint(0.001);

  MathematicalProgramResult program_result_rpy = contact_solver_rpy.Solve();
  std::cout << "Contact solver result (Rpy Floating base): "
            << program_result_rpy.get_solution_result() << std::endl;

  VectorXd q_sol_rpy = contact_solver_rpy.GetSolutionQ();
  MatrixXd T_sol_rpy =
      estimator_rpy.ComputeTransformationToeRightWrtIMU(q_sol_rpy);

  // Rotation matrix between the pelvis and the world
  KinematicsCache<double> cache_rpy = tree_rpy_.doKinematics(q_sol_rpy);
  int world_ind = GetBodyIndexFromName(tree_rpy_, "world");
  int pelvis_ind = GetBodyIndexFromName(tree_rpy_, "pelvis");
  Transform<double, 3, Isometry> T =
      tree_rpy_.relativeTransform(cache_rpy, world_ind, pelvis_ind);
  MatrixXd R = T.matrix().block(0, 0, 3, 3);

  // The local collision points in homogeneous coordinates
  VectorXd local_collision_pt1_hom(4), local_collision_pt2_hom(4);
  local_collision_pt1_hom.head(3) = local_collision_pt1_;
  local_collision_pt1_hom(3) = 1;
  local_collision_pt2_hom.head(3) = local_collision_pt2_;
  local_collision_pt2_hom(3) = 1;

  VectorXd d1_hom = T_sol_rpy * local_collision_pt1_hom;
  VectorXd d2_hom = T_sol_rpy * local_collision_pt2_hom;
  VectorXd d1 = d1_hom.head(3);
  VectorXd d2 = d2_hom.head(3);

  // Contact point in world coordinates
  VectorXd p1 = R * d1 + q_sol_rpy.head(3);
  VectorXd p2 = R * d2 + q_sol_rpy.head(3);

  // Checking if the contact points in world frame are on the ground plane.
  ASSERT_TRUE(p1(2) < 1e-6);
  ASSERT_TRUE(p2(2) < 1e-6);
}

TEST_F(CassieRbtStateEstimatorTest, TestRotationToeLeft) {
  MatrixXd X = MatrixXd::Random(9, 9);
  VectorXd ekf_bias = VectorXd::Random(6);

  CassieRbtStateEstimator estimator_rpy(tree_rpy_, X, ekf_bias, true);
  CassieRbtStateEstimator estimator_quaternion(tree_quaternion_, X, ekf_bias,
                                               true);

  VectorXd q_rpy = Eigen::VectorXd::Random(tree_rpy_.get_num_positions());
  MatrixXd R_rpy = estimator_rpy.ComputeRotationToeLeftWrtIMU(q_rpy);

  VectorXd q_quaternion =
      Eigen::VectorXd::Random(tree_quaternion_.get_num_positions());
  MatrixXd R_quaternion =
      estimator_quaternion.ComputeRotationToeLeftWrtIMU(q_quaternion);

  // Checking for a valid rotation matrix
  ASSERT_TRUE(MatrixXd::Identity(3, 3).isApprox(R_rpy.transpose() * R_rpy));
  ASSERT_TRUE(MatrixXd::Identity(3, 3).isApprox(R_quaternion.transpose() *
                                                R_quaternion));

  // Changing the floating base position and orientation and recomputing the
  // rotation matrix. As it is relative to the pelvis, the rotation matrix wrt
  // to the pelvis must remain unchanged.

  q_rpy.head(6) = VectorXd::Random(6);
  q_quaternion.head(7) = VectorXd::Random(7);

  ASSERT_TRUE(
      R_rpy.isApprox(estimator_rpy.ComputeRotationToeLeftWrtIMU(q_rpy)));
  ASSERT_TRUE(R_quaternion.isApprox(
      estimator_quaternion.ComputeRotationToeLeftWrtIMU(q_quaternion)));
}

TEST_F(CassieRbtStateEstimatorTest, TestRotationToeRight) {
  MatrixXd X = MatrixXd::Random(9, 9);
  VectorXd ekf_bias = VectorXd::Random(6);

  CassieRbtStateEstimator estimator_rpy(tree_rpy_, X, ekf_bias, true);
  CassieRbtStateEstimator estimator_quaternion(tree_quaternion_, X, ekf_bias,
                                               true);

  VectorXd q_rpy = VectorXd::Random(tree_rpy_.get_num_positions());
  MatrixXd R_rpy = estimator_rpy.ComputeRotationToeRightWrtIMU(q_rpy);

  VectorXd q_quaternion =
      VectorXd::Random(tree_quaternion_.get_num_positions());
  MatrixXd R_quaternion =
      estimator_quaternion.ComputeRotationToeRightWrtIMU(q_quaternion);

  // Checking for a valid rotation matrix
  ASSERT_TRUE(MatrixXd::Identity(3, 3).isApprox(R_rpy.transpose() * R_rpy));
  ASSERT_TRUE(MatrixXd::Identity(3, 3).isApprox(R_quaternion.transpose() *
                                                R_quaternion));

  // Changing the floating base position and orientation and recomputing the
  // rotation matrix. As it is relative to the pelvis, the rotation matrix wrt
  // to the pelvis must remain unchanged.

  q_rpy.head(6) = VectorXd::Random(6);
  q_quaternion.head(6) = VectorXd::Random(6);

  ASSERT_TRUE(
      R_rpy.isApprox(estimator_rpy.ComputeRotationToeRightWrtIMU(q_rpy)));
  ASSERT_TRUE(R_quaternion.isApprox(
      estimator_quaternion.ComputeRotationToeRightWrtIMU(q_quaternion)));
}

TEST_F(CassieRbtStateEstimatorTest, TestLeftToeJacobian) {
  MatrixXd X = MatrixXd::Random(9, 9);
  VectorXd ekf_bias = VectorXd::Random(6);

  CassieRbtStateEstimator estimator_rpy(tree_rpy_, X, ekf_bias, true);
  CassieRbtStateEstimator estimator_quaternion(tree_quaternion_, X, ekf_bias,
                                               true);

  VectorXd q_rpy = VectorXd::Random(tree_rpy_.get_num_positions());
  VectorXd p_rpy = VectorXd::Random(3);

  VectorXd q_quaternion =
      VectorXd::Random(tree_quaternion_.get_num_positions());
  VectorXd p_quaternion = VectorXd::Random(3);

  MatrixXd J_rpy = estimator_rpy.ComputeToeLeftJacobianWrtIMU(q_rpy, p_rpy);
  MatrixXd J_quaternion = estimator_quaternion.ComputeToeLeftJacobianWrtIMU(
      q_quaternion, p_quaternion);

  // Making sure that the Jacobian is independent of the floating base
  // coordinates
  q_rpy.head(6) = VectorXd::Random(6);
  q_quaternion.head(7) = VectorXd::Random(7);

  ASSERT_TRUE(
      J_rpy.isApprox(estimator_rpy.ComputeToeLeftJacobianWrtIMU(q_rpy, p_rpy)));
  ASSERT_TRUE(
      J_quaternion.isApprox(estimator_quaternion.ComputeToeLeftJacobianWrtIMU(
          q_quaternion, p_quaternion)));
}

TEST_F(CassieRbtStateEstimatorTest, TestRightToeJacobian) {
  MatrixXd X = MatrixXd::Random(9, 9);
  VectorXd ekf_bias = VectorXd::Random(6);

  CassieRbtStateEstimator estimator_rpy(tree_rpy_, X, ekf_bias, true);
  CassieRbtStateEstimator estimator_quaternion(tree_quaternion_, X, ekf_bias,
                                               true);

  VectorXd q_rpy = VectorXd::Random(tree_rpy_.get_num_positions());
  VectorXd p_rpy = VectorXd::Random(3);

  VectorXd q_quaternion =
      VectorXd::Random(tree_quaternion_.get_num_positions());
  VectorXd p_quaternion = VectorXd::Random(3);

  MatrixXd J_rpy = estimator_rpy.ComputeToeRightJacobianWrtIMU(q_rpy, p_rpy);
  MatrixXd J_quaternion = estimator_quaternion.ComputeToeRightJacobianWrtIMU(
      q_quaternion, p_quaternion);

  // Making sure that the Jacobian is independent of the floating base
  // coordinates
  q_rpy.head(6) = VectorXd::Random(6);
  q_quaternion.head(7) = VectorXd::Random(7);

  ASSERT_TRUE(J_rpy.isApprox(
      estimator_rpy.ComputeToeRightJacobianWrtIMU(q_rpy, p_rpy)));
  ASSERT_TRUE(
      J_quaternion.isApprox(estimator_quaternion.ComputeToeRightJacobianWrtIMU(
          q_quaternion, p_quaternion)));
}

TEST_F(CassieRbtStateEstimatorTest, TestComputeX) {
  MatrixXd X = MatrixXd::Random(9, 9);
  // A valid state representation for this test. Manually filling in the zeros
  // and the identity block
  X.block(3, 0, 6, 9) = MatrixXd::Zero(6, 9);
  X.block(3, 3, 6, 6) = MatrixXd::Identity(6, 6);
  VectorXd ekf_bias = VectorXd::Random(6);

  CassieRbtStateEstimator estimator(tree_rpy_, X, ekf_bias, true);

  VectorXd ekf_x = estimator.ComputeEkfX(X);

  // Testing the other overloaded function.
  MatrixXd R = estimator.ExtractRotationMatrix(ekf_x);
  MatrixXd v = estimator.ExtractFloatingBaseVelocities(ekf_x);
  MatrixXd p = estimator.ExtractFloatingBasePositions(ekf_x);
  MatrixXd d = estimator.ExtractContactPositions(ekf_x);
  ASSERT_TRUE(X.isApprox(estimator.ComputeX(R, v, p, d)));
}

TEST_F(CassieRbtStateEstimatorTest, TestComputeEkfX) {
  MatrixXd X = MatrixXd::Random(9, 9);
  // A valid state representation for this test. Manually filling in the zeros
  // and the identity block
  X.block(3, 0, 6, 9) = MatrixXd::Zero(6, 9);
  X.block(3, 3, 6, 6) = MatrixXd::Identity(6, 6);
  VectorXd ekf_bias = VectorXd::Random(6);

  CassieRbtStateEstimator estimator(tree_rpy_, X, ekf_bias, true);

  VectorXd ekf_x = estimator.ComputeEkfX(X);
  VectorXd X_reconstructed = estimator.ComputeX(ekf_x);

  ASSERT_TRUE(X.isApprox(X_reconstructed));
}

TEST_F(CassieRbtStateEstimatorTest, TestComputeP) {
  MatrixXd X = MatrixXd::Random(9, 9);
  VectorXd ekf_bias = VectorXd::Random(6);

  CassieRbtStateEstimator estimator(tree_rpy_, X, ekf_bias, true);

  VectorXd ekf_p = VectorXd::Random(25);
  MatrixXd P = estimator.ComputeP(ekf_p);
  VectorXd ekf_p_reconstructed = estimator.ComputeEkfP(P);

  ASSERT_TRUE(ekf_p.isApprox(ekf_p_reconstructed));
}

// TODO: Complete this test
TEST_F(CassieRbtStateEstimatorTest, TestPredictX) {
  MatrixXd X = MatrixXd::Random(9, 9);
  VectorXd ekf_bias = VectorXd::Random(6);

  CassieRbtStateEstimator estimator(tree_rpy_, X, ekf_bias, true);
  VectorXd ekf_x = estimator.ComputeEkfX(X);

  VectorXd q_rpy = VectorXd::Random(tree_rpy_.get_num_positions());
  VectorXd u_rpy = VectorXd::Random(tree_rpy_.get_num_actuators());
  double dt = 0.001;

  MatrixXd X_pred = estimator.PredictX(ekf_x, ekf_bias, u_rpy, q_rpy, dt);
}

// TODO: Complete this test
TEST_F(CassieRbtStateEstimatorTest, TestAdjoint) {
  MatrixXd X = MatrixXd::Random(9, 9);
  VectorXd ekf_bias = VectorXd::Random(6);

  CassieRbtStateEstimator estimator(tree_rpy_, X, ekf_bias, true);
  VectorXd ekf_x = estimator.ComputeEkfX(X);

  MatrixXd adj = estimator.ComputeAdjointOperator(ekf_x);
}

// TODO: Complete this test
TEST_F(CassieRbtStateEstimatorTest, TestA) {
  MatrixXd X = MatrixXd::Random(9, 9);
  VectorXd ekf_bias = VectorXd::Random(6);

  CassieRbtStateEstimator estimator(tree_rpy_, X, ekf_bias, true);
  VectorXd ekf_x = estimator.ComputeEkfX(X);

  MatrixXd A = estimator.ComputeA(ekf_x);

  // std::cout << A << std::endl;
}

// TODO: Complete this test
TEST_F(CassieRbtStateEstimatorTest, TestCovariance) {
  MatrixXd X = MatrixXd::Random(9, 9);
  VectorXd ekf_bias = VectorXd::Random(6);

  CassieRbtStateEstimator estimator(tree_rpy_, X, ekf_bias, true);

  VectorXd q = VectorXd::Random(tree_rpy_.get_num_positions());

  estimator.set_contacts(1, 0, 1, 0);
  MatrixXd Cov = estimator.ComputeCov(q);
}

// TODO: Complete this test
TEST_F(CassieRbtStateEstimatorTest, TestPredictP) {
  MatrixXd X = MatrixXd::Random(9, 9);
  VectorXd ekf_bias = VectorXd::Random(6);
  VectorXd ekf_p = VectorXd::Random(27 * 27);

  CassieRbtStateEstimator estimator(tree_rpy_, X, ekf_bias, true);
  VectorXd ekf_x = estimator.ComputeEkfX(X);

  VectorXd q_rpy = VectorXd::Random(tree_rpy_.get_num_positions());
  double dt = 0.01;

  MatrixXd P_pred = estimator.PredictP(ekf_x, ekf_p, q_rpy, dt);
}

TEST_F(CassieRbtStateEstimatorTest, TestComputeUpdateParams) {
  MatrixXd X = MatrixXd::Random(9, 9);
  VectorXd ekf_bias = VectorXd::Random(6);
  VectorXd ekf_p = VectorXd::Random(27 * 27);

  CassieRbtStateEstimator estimator_rpy(tree_rpy_, X, ekf_bias, true);
  CassieRbtStateEstimator estimator_quaternion(tree_quaternion_, X, ekf_bias,
                                               true);
  VectorXd ekf_x = estimator_rpy.ComputeEkfX(X);

  VectorXd q_rpy = VectorXd::Random(tree_rpy_.get_num_positions());
  VectorXd q_quaternion =
      VectorXd::Random(tree_quaternion_.get_num_positions());

  VectorXd y, b, delta;
  MatrixXd H, N, Pi, X_full, S, K;

  // Testing the update parameters for various contact configurations

  // All contacts active
  estimator_rpy.ComputeUpdateParams(ekf_x, ekf_p, q_rpy, y, b, H, N, Pi, X_full,
                                    S, K, delta);

  ASSERT_TRUE(H.rows() == N.rows());
  ASSERT_TRUE(H.cols() == 27);
  ASSERT_TRUE(K.rows() == 27);
  ASSERT_TRUE(K.cols() == Pi.rows());
  ASSERT_TRUE(Pi.cols() == X_full.rows());
  ASSERT_TRUE(X_full.cols() == y.rows());

  // One contact active
  estimator_rpy.set_contacts(0, 1, 0, 0);
  estimator_rpy.ComputeUpdateParams(ekf_x, ekf_p, q_rpy, y, b, H, N, Pi, X_full,
                                    S, K, delta);

  ASSERT_TRUE(H.rows() == N.rows());
  ASSERT_TRUE(H.cols() == 27);
  ASSERT_TRUE(K.rows() == 27);
  ASSERT_TRUE(K.cols() == Pi.rows());
  ASSERT_TRUE(Pi.cols() == X_full.rows());
  ASSERT_TRUE(X_full.cols() == y.rows());

  // Two contacts active
  estimator_rpy.set_contacts(0, 1, 0, 1);
  estimator_rpy.ComputeUpdateParams(ekf_x, ekf_p, q_rpy, y, b, H, N, Pi, X_full,
                                    S, K, delta);

  ASSERT_TRUE(H.rows() == N.rows());
  ASSERT_TRUE(H.cols() == 27);
  ASSERT_TRUE(K.rows() == 27);
  ASSERT_TRUE(K.cols() == Pi.rows());
  ASSERT_TRUE(Pi.cols() == X_full.rows());
  ASSERT_TRUE(X_full.cols() == y.rows());

  // Three contacts active
  estimator_rpy.set_contacts(1, 1, 0, 1);
  estimator_rpy.ComputeUpdateParams(ekf_x, ekf_p, q_rpy, y, b, H, N, Pi, X_full,
                                    S, K, delta);

  ASSERT_TRUE(H.rows() == N.rows());
  ASSERT_TRUE(H.cols() == 27);
  ASSERT_TRUE(K.rows() == 27);
  ASSERT_TRUE(K.cols() == Pi.rows());
  ASSERT_TRUE(Pi.cols() == X_full.rows());
  ASSERT_TRUE(X_full.cols() == y.rows());

  // Similar tests for the quaternion base
  // All contacts active
  estimator_quaternion.ComputeUpdateParams(ekf_x, ekf_p, q_quaternion, y, b, H,
                                           N, Pi, X_full, S, K, delta);

  ASSERT_TRUE(H.rows() == N.rows());
  ASSERT_TRUE(H.cols() == 27);
  ASSERT_TRUE(K.rows() == 27);
  ASSERT_TRUE(K.cols() == Pi.rows());
  ASSERT_TRUE(Pi.cols() == X_full.rows());
  ASSERT_TRUE(X_full.cols() == y.rows());

  // One contact active
  estimator_quaternion.set_contacts(0, 1, 0, 0);
  estimator_quaternion.ComputeUpdateParams(ekf_x, ekf_p, q_quaternion, y, b, H,
                                           N, Pi, X_full, S, K, delta);

  ASSERT_TRUE(H.rows() == N.rows());
  ASSERT_TRUE(H.cols() == 27);
  ASSERT_TRUE(K.rows() == 27);
  ASSERT_TRUE(K.cols() == Pi.rows());
  ASSERT_TRUE(Pi.cols() == X_full.rows());
  ASSERT_TRUE(X_full.cols() == y.rows());

  // Two contacts active
  estimator_quaternion.set_contacts(0, 1, 0, 1);
  estimator_quaternion.ComputeUpdateParams(ekf_x, ekf_p, q_quaternion, y, b, H,
                                           N, Pi, X_full, S, K, delta);

  ASSERT_TRUE(H.rows() == N.rows());
  ASSERT_TRUE(H.cols() == 27);
  ASSERT_TRUE(K.rows() == 27);
  ASSERT_TRUE(K.cols() == Pi.rows());
  ASSERT_TRUE(Pi.cols() == X_full.rows());
  ASSERT_TRUE(X_full.cols() == y.rows());

  // Three contacts active
  estimator_quaternion.set_contacts(1, 1, 0, 1);
  estimator_quaternion.ComputeUpdateParams(ekf_x, ekf_p, q_quaternion, y, b, H,
                                           N, Pi, X_full, S, K, delta);

  ASSERT_TRUE(H.rows() == N.rows());
  ASSERT_TRUE(H.cols() == 27);
  ASSERT_TRUE(K.rows() == 27);
  ASSERT_TRUE(K.cols() == Pi.rows());
  ASSERT_TRUE(Pi.cols() == X_full.rows());
  ASSERT_TRUE(X_full.cols() == y.rows());
}

TEST_F(CassieRbtStateEstimatorTest, TestUpdateX) {
  MatrixXd X = MatrixXd::Random(9, 9);
  VectorXd ekf_bias = VectorXd::Random(6);
  VectorXd ekf_p = VectorXd::Random(27 * 27);

  CassieRbtStateEstimator estimator_rpy(tree_rpy_, X, ekf_bias, true);
  CassieRbtStateEstimator estimator_quaternion(tree_quaternion_, X, ekf_bias,
                                               true);
  VectorXd ekf_x = estimator_rpy.ComputeEkfX(X);

  VectorXd q_rpy = VectorXd::Random(tree_rpy_.get_num_positions());
  VectorXd q_quaternion =
      VectorXd::Random(tree_quaternion_.get_num_positions());

  // Computing the update parameters
  VectorXd y, b, delta;
  MatrixXd H, N, Pi, X_full, S, K;

  estimator_rpy.ComputeUpdateParams(ekf_x, ekf_p, q_rpy, y, b, H, N, Pi, X_full,
                                    S, K, delta);

  MatrixXd X_updated_rpy =
      estimator_rpy.UpdateX(estimator_rpy.ComputeX(ekf_x), delta);

  // Checking if the udpated state matrix is valid
  ASSERT_TRUE(X_updated_rpy.block(3, 0, 6, 3).isApprox(MatrixXd::Zero(6, 3)));
  ASSERT_TRUE(
      X_updated_rpy.block(3, 3, 6, 6).isApprox(MatrixXd::Identity(6, 6)));

  estimator_quaternion.ComputeUpdateParams(ekf_x, ekf_p, q_quaternion, y, b, H,
                                           N, Pi, X_full, S, K, delta);

  MatrixXd X_updated_quaternion =
      estimator_quaternion.UpdateX(estimator_quaternion.ComputeX(ekf_x), delta);

  // Checking if the udpated state matrix is valid
  ASSERT_TRUE(
      X_updated_quaternion.block(3, 0, 6, 3).isApprox(MatrixXd::Zero(6, 3)));
  ASSERT_TRUE(X_updated_quaternion.block(3, 3, 6, 6)
                  .isApprox(MatrixXd::Identity(6, 6)));
}

}  // namespace
}  // namespace systems
}  // namespace dairlib

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
