#include "examples/Cassie/cassie_rbt_state_estimator.h"
#include <gtest/gtest.h>
#include "attic/multibody/multibody_solvers.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"

namespace dairlib {
namespace systems {
namespace {

using dairlib::multibody::PositionSolver;
using Eigen::VectorXd;

class CassieRbtStateEstimatorTest : public ::testing::Test {};

class ContactEstimationTest : public ::testing::Test {
 protected:
  ContactEstimationTest() {
    buildCassieTree(tree_, "examples/Cassie/urdf/cassie_v2.urdf",
                    drake::multibody::joints::kQuaternion);
    drake::multibody::AddFlatTerrainToWorld(&tree_, 100, 0.2);
    estimator_ = std::make_unique<CassieRbtStateEstimator>(tree_, true);

    output_ = std::make_unique<OutputVector<double>>(tree_.get_num_positions(),
                                                     tree_.get_num_velocities(),
                                                     tree_.get_num_actuators());
  }

  void calculateGroundTruth(int& gtl, int& gtr) {
    KinematicsCache<double> cache = tree_.doKinematics(
        output_->GetPositions(), output_->GetVelocities(), true);
    std::vector<drake::multibody::collision::PointPair<double>> pairs;
    pairs = tree_.ComputeMaximumDepthCollisionPoints(cache, true, false);
    for (const auto& pair : pairs) {
      if (pair.distance < 0.0) {
        if (pair.elementA->get_body()->get_body_index() == 18) {
          gtl += 1;
        }
        if (pair.elementA->get_body()->get_body_index() == 20) {
          gtr += 1;
        }
      }
    }
  }

  RigidBodyTree<double> tree_;
  std::unique_ptr<CassieRbtStateEstimator> estimator_;
  std::unique_ptr<OutputVector<double>> output_;
  double dt_;
  int left_contact_;
  int right_contact_;
};

TEST_F(CassieRbtStateEstimatorTest, solveFourbarLinkageTest) {
  RigidBodyTree<double> tree;
  buildCassieTree(tree);
  CassieRbtStateEstimator estimator(tree, false);

  // Example position
  VectorXd q_init(tree.get_num_positions());
  q_init << -0.084017, 0.0826151, -0.00120735, 0.00217829, 0.366012, 0.365803,
      -0.6305, -0.630502, 0.00205363, 0.00205356, 0.838878, 0.838882, 0, 0,
      0.205351, 0.20456;

  // Get the angles analytically
  double calc_left_heel_spring, calc_right_heel_spring;
  estimator.solveFourbarLinkage(q_init, &calc_left_heel_spring,
                                &calc_right_heel_spring);

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

  EXPECT_TRUE((calc_left_heel_spring - nlp_left_heel_spring) < 1e-10);
  EXPECT_TRUE((calc_right_heel_spring - nlp_right_heel_spring) < 1e-10);
  EXPECT_TRUE((calc_left_heel_spring - nlp_left_heel_spring) > -1e-10);
  EXPECT_TRUE((calc_right_heel_spring - nlp_right_heel_spring) > -1e-10);
}

// Double support contact estimation test
// Checks if the contactEstimation returns the correct contacts for a
// configuration of the robot in double stance.
// This case is chosen in such a way that the stance can be estimated using just
// springs.
TEST_F(ContactEstimationTest, DoubleSupportContactEstimationTest) {
  VectorXd q(tree_.get_num_positions());
  VectorXd v(tree_.get_num_velocities());
  VectorXd u(tree_.get_num_actuators());
  VectorXd alpha_imu(3);

  q << 0.00836164, -0.000249535, 1.03223, 0.990065, 0.000339553, 0.00444831,
      0.00085048, -0.000810813, 6.8811e-05, 0.00177426, -0.00514383, 0.447568,
      0.44727, -1.01775, -1.01819, -0.044873, -0.0450231, 1.29924, 1.30006,
      0.00780166, 0.00757446, -1.56023, -1.56018;
  v << -0.00112262, -0.130482, -0.00168999, -0.0162583, 0.000174138, 0.036476,
      -0.00928683, 0.0104115, 0.0705929, -0.0664504, -2.19116, -2.20063,
      7.09707, 7.12947, -5.46722, -5.49114, 1.1875, 1.1884, 0, 0, -5.8359,
      -5.83556;
  u << 0.969754, -1.05396, 0.188614, -0.196147, -7.71725, -7.51014, 54.0345,
      54.2956, -17.5288, -17.5305;
  alpha_imu << 4.44637, -0.015849, -6.28362;

  output_->SetPositions(q);
  output_->SetVelocities(v);
  output_->SetEfforts(u);
  output_->SetIMUAccelerations(alpha_imu);

  dt_ = 0.001;
  left_contact_ = 0;
  right_contact_ = 0;

  auto context = estimator_->CreateDefaultContext();

  estimator_->contactEstimation(*output_, dt_,
      &context->get_mutable_discrete_state(), &left_contact_, &right_contact_);

  int gtl = 0;
  int gtr = 0;
  calculateGroundTruth(gtl, gtr);

  EXPECT_TRUE(gtl && left_contact_)
      << "Left contact error during double support.";
  EXPECT_TRUE(gtr && right_contact_)
      << "Right contact error during double support.";
}

// Left support contact estimation test
// Checks if the contactEstimation returns the correct contacts for a
// configuration of the robot in left stance.
TEST_F(ContactEstimationTest, LeftSupportContactEstimationTest) {
  VectorXd q(tree_.get_num_positions());
  VectorXd v(tree_.get_num_velocities());
  VectorXd u(tree_.get_num_actuators());
  VectorXd alpha_imu(3);

  q << 0.344537, -0.148108, 1.00902, 0.989849, -0.000815987, -0.017933,
      -0.0111588, -0.0357916, -0.0422061, -0.0068692, -0.0355008, 0.274222,
      0.644396, -1.00482, -1.50496, -0.0745786, -0.000565784, 1.36746, 1.73074,
      -0.043625, -0.000678207, -1.45868, -0.936994;
  v << -0.110601, -0.0521661, -0.00286609, 0.910837, -0.0174017, -0.00158473,
      0.124156, 0.8427, 0.0224065, 0.0678774, -1.22403, 2.89698, 0.32455,
      2.21075, -0.03638, 0.247213, -0.333968, -2.51737, 0, 0, 1.36041, -4.312;
  u << 43.4069, -3.11852, -0.18289, -0.315572, -7.86163, -0.12787, 78.4677,
      2.16671, -3.37293, -4.57146;
  alpha_imu << 0.807881, -0.889675, -9.63094;

  output_->SetPositions(q);
  output_->SetVelocities(v);
  output_->SetEfforts(u);
  output_->SetIMUAccelerations(alpha_imu);

  dt_ = 0.001;
  left_contact_ = 0;
  right_contact_ = 0;

  auto context = estimator_->CreateDefaultContext();

  estimator_->contactEstimation(*output_, dt_,
      &context->get_mutable_discrete_state(), &left_contact_, &right_contact_);

  int gtl = 0;
  int gtr = 0;
  calculateGroundTruth(gtl, gtr);

  EXPECT_TRUE(gtl && left_contact_)
      << "Left contact error during left support.";
  EXPECT_TRUE(!gtr && !right_contact_)
      << "Right contact error during left support.";
}

// Right support contact estimation test
// Checks if the contactEstimation returns the correct contacts for a
// configuration of the robot in right stance.
TEST_F(ContactEstimationTest, RightSupportContactEstimationTest) {
  VectorXd q(tree_.get_num_positions());
  VectorXd v(tree_.get_num_velocities());
  VectorXd u(tree_.get_num_actuators());
  VectorXd alpha_imu(3);

  q << 0.01949, -0.157343, 1.00405, 0.98987, -0.00811052, -0.00949625, 0.015811,
      0.123047, -0.0753356, 0.0288855, -0.0330248, 0.832632, 0.0262067,
      -1.52869, -0.882942, -0.00078387, -0.0740736, 1.74919, 1.23608,
      0.00556074, -0.0305797, -1.49203, -1.2012;
  v << 0.156632, -0.0502397, 0.101071, 0.232441, -0.296125, -0.0559459,
      -0.663525, 0.116557, -0.0264677, -0.107556, 2.18153, -0.0230963, -1.65117,
      -1.02961, -0.0386449, 0.735924, 1.75789, -0.0410481, 0, 0, -1.46269,
      0.482573;
  u << 1.17905, -39.036, -0.698845, 10.992, 4.76689, -7.84931, -0.399509, 71.98,
      -1.56492, -9.37228;
  alpha_imu << 2.06368, 2.54482, -9.64308;

  output_->SetPositions(q);
  output_->SetVelocities(v);
  output_->SetEfforts(u);
  output_->SetIMUAccelerations(alpha_imu);

  dt_ = 0.001;
  left_contact_ = 0;
  right_contact_ = 0;

  auto context = estimator_->CreateDefaultContext();

  estimator_->contactEstimation(*output_, dt_,
      &context->get_mutable_discrete_state(), &left_contact_, &right_contact_);

  int gtl = 0;
  int gtr = 0;
  calculateGroundTruth(gtl, gtr);

  EXPECT_TRUE(!gtl && !left_contact_)
      << "Left contact error during right support.";
  EXPECT_TRUE(gtr && right_contact_)
      << "Right contact error during right support.";
}
}  // namespace
}  // namespace systems
}  // namespace dairlib

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
