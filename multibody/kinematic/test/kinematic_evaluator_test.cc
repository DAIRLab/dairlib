#include "multibody/kinematic/kinematic_evaluator.h"

#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "common/find_resource.h"
#include "multibody/kinematic/distance_evaluator.h"
#include "multibody/kinematic/world_point_evaluator.h"

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace dairlib {
namespace multibody {
namespace {

using drake::CompareMatrices;
using drake::geometry::SceneGraph;
using drake::multibody::Body;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

class KinematicEvaluatorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    plant_ = std::make_unique<MultibodyPlant<double>>(0.0);
    auto scene_graph = std::make_unique<SceneGraph<double>>();
    Parser parser(plant_.get(), scene_graph.get());
    std::string full_name =
        dairlib::FindResourceOrThrow("examples/PlanarWalker/PlanarWalker.urdf");

    parser.AddModelFromFile(full_name);

    plant_->WeldFrames(plant_->world_frame(), plant_->GetFrameByName("base"),
                       drake::math::RigidTransform<double>());

    plant_->Finalize();
  }

  std::unique_ptr<MultibodyPlant<double>> plant_;
};

TEST_F(KinematicEvaluatorTest,
       WorldPointEvaluatorFullRotationSpecificationTest) {
  const double tolerance = 1e-10;

  Vector3d pt_A({0, 0, -.5});
  const auto& frame = plant_->GetFrameByName("right_lower_leg");

  // Default evaluator: identity ground frame, zero offset
  std::vector<int> active_directions = {2};
  auto evaluator = WorldPointEvaluator<double>(
      *plant_, pt_A, frame, Eigen::Matrix3d::Identity(), Vector3d::Zero(),
      active_directions);

  auto context = plant_->CreateDefaultContext();

  // Test q = 0, legs straight down
  VectorXd q = VectorXd::Zero(plant_->num_positions());
  plant_->SetPositions(context.get(), q);
  VectorXd v = Eigen::VectorXd::Constant(plant_->num_velocities(), 1);
  plant_->SetVelocities(context.get(), v);

  auto phi = evaluator.EvalFull(*context);
  EXPECT_TRUE(CompareMatrices(Vector3d({0, 0, -1}), phi, tolerance));

  auto phi_active = evaluator.EvalActive(*context);

  VectorXd phi_active_expected(1);
  phi_active_expected << -1;
  EXPECT_TRUE(CompareMatrices(phi_active_expected, phi_active, tolerance));

  auto J = evaluator.EvalFullJacobian(*context);
  MatrixXd J_expected(3, 6);
  J_expected.row(0) << 1, 0, -1, 0, -1, -0.5;
  J_expected.row(1) << 0, 0, 0, 0, 0, 0;
  J_expected.row(2) << 0, 1, 0, 0, 0, 0;
  EXPECT_TRUE(CompareMatrices(J, J_expected, tolerance));

  auto J_active = evaluator.EvalActiveJacobian(*context);
  MatrixXd J_active_expected(1, 6);
  J_active_expected << 0, 1, 0, 0, 0, 0;
  EXPECT_TRUE(CompareMatrices(J_active, J_active_expected, tolerance));

  auto phidot = evaluator.EvalFullTimeDerivative(*context);
  VectorXd phidot_expected(3);
  phidot_expected << -1.5, 0, 1;
  EXPECT_TRUE(CompareMatrices(phidot, phidot_expected, tolerance));

  auto phidot_active = evaluator.EvalActiveTimeDerivative(*context);
  VectorXd phidot_active_expected(1);
  phidot_active_expected << 1;
  EXPECT_TRUE(
      CompareMatrices(phidot_active, phidot_active_expected, tolerance));

  auto Jdotv = evaluator.EvalFullJacobianDotTimesV(*context);
  VectorXd Jdotv_expected(3);
  Jdotv_expected << 0, 0, 6.5;
  EXPECT_TRUE(CompareMatrices(Jdotv, Jdotv_expected, tolerance));

  auto Jdotv_active = evaluator.EvalActiveJacobianDotTimesV(*context);
  VectorXd Jdotv_active_expected(1);
  Jdotv_active_expected << 6.5;
  EXPECT_TRUE(CompareMatrices(Jdotv_active, Jdotv_active_expected, tolerance));

  // Non-default evaluator: non-vertical normal, non-zero offset
  // Performing minimal tests
  auto new_evaluator = WorldPointEvaluator<double>(
      *plant_, pt_A, frame, Vector3d({1, 0, 0}), Vector3d({1, 2, 3}), true);

  auto new_phi = new_evaluator.EvalFull(*context);
  EXPECT_TRUE(CompareMatrices(Vector3d({4, -2, -1}), new_phi, tolerance));
  auto new_phi_active = new_evaluator.EvalActive(*context);
  EXPECT_TRUE(
      CompareMatrices(Vector3d({4, -2, -1}), new_phi_active, tolerance));
}

TEST_F(KinematicEvaluatorTest,
       WorldPointEvaluatorGroundNormalSpecificationTest) {
  const double tolerance = 1e-10;

  Vector3d pt_A({0, 0, -.5});
  const auto& frame = plant_->GetFrameByName("right_lower_leg");

  // Default evaluator: vertical normal, zero offset
  auto evaluator = WorldPointEvaluator<double>(
      *plant_, pt_A, frame, Vector3d({0, 0, 1}), Vector3d::Zero(), false);

  auto context = plant_->CreateDefaultContext();

  // Test q = 0, legs straight down
  VectorXd q = VectorXd::Zero(plant_->num_positions());
  plant_->SetPositions(context.get(), q);
  VectorXd v = Eigen::VectorXd::Constant(plant_->num_velocities(), 1);
  plant_->SetVelocities(context.get(), v);

  // We construct WorldPointEvaluator's rotation matrix by the normal vector of
  // ground plane, so we don't have control over the tangent directions, so we
  // don't compare x and y component (the first two rows) of phi, J, phidot,
  // JdotV.
  auto phi = evaluator.EvalFull(*context);
  EXPECT_TRUE(
      CompareMatrices(Vector3d({0, 0, -1}).row(2), phi.row(2), tolerance));

  auto phi_active = evaluator.EvalActive(*context);

  VectorXd phi_active_expected(1);
  phi_active_expected << -1;
  EXPECT_TRUE(CompareMatrices(phi_active_expected, phi_active, tolerance));

  auto J = evaluator.EvalFullJacobian(*context);
  MatrixXd J_expected(3, 6);
  J_expected.row(0) << 1, 0, -1, 0, -1, -0.5;
  J_expected.row(1) << 0, 0, 0, 0, 0, 0;
  J_expected.row(2) << 0, 1, 0, 0, 0, 0;
  EXPECT_TRUE(CompareMatrices(J.row(2), J_expected.row(2), tolerance));

  auto J_active = evaluator.EvalActiveJacobian(*context);
  MatrixXd J_active_expected(1, 6);
  J_active_expected << 0, 1, 0, 0, 0, 0;
  EXPECT_TRUE(CompareMatrices(J_active, J_active_expected, tolerance));

  auto phidot = evaluator.EvalFullTimeDerivative(*context);
  VectorXd phidot_expected(3);
  phidot_expected << -1.5, 0, 1;
  EXPECT_TRUE(
      CompareMatrices(phidot.row(2), phidot_expected.row(2), tolerance));

  auto phidot_active = evaluator.EvalActiveTimeDerivative(*context);
  VectorXd phidot_active_expected(1);
  phidot_active_expected << 1;
  EXPECT_TRUE(
      CompareMatrices(phidot_active, phidot_active_expected, tolerance));

  auto Jdotv = evaluator.EvalFullJacobianDotTimesV(*context);
  VectorXd Jdotv_expected(3);
  Jdotv_expected << 0, 0, 6.5;
  EXPECT_TRUE(CompareMatrices(Jdotv.row(2), Jdotv_expected.row(2), tolerance));

  auto Jdotv_active = evaluator.EvalActiveJacobianDotTimesV(*context);
  VectorXd Jdotv_active_expected(1);
  Jdotv_active_expected << 6.5;
  EXPECT_TRUE(CompareMatrices(Jdotv_active, Jdotv_active_expected, tolerance));

  // Non-default evaluator: non-vertical normal, non-zero offset
  // Performing minimal tests
  auto new_evaluator = WorldPointEvaluator<double>(
      *plant_, pt_A, frame, Vector3d({1, 0, 0}), Vector3d({1, 2, 3}), true);

  auto new_phi = new_evaluator.EvalFull(*context);
  EXPECT_TRUE(
      CompareMatrices(Vector3d({4, -2, -1}).row(2), new_phi.row(2), tolerance));
  auto new_phi_active = new_evaluator.EvalActive(*context);
  EXPECT_TRUE(
      CompareMatrices(Vector3d({4, -2, -1}), new_phi_active, tolerance));
}

TEST_F(KinematicEvaluatorTest, DistanceEvaluatorTest) {
  const double tolerance = 1e-6;

  Vector3d pt_A({0, 0, -.5});
  const auto& frame_A = plant_->GetFrameByName("right_lower_leg");
  Vector3d pt_B({0, 0, -.5});
  const auto& frame_B = plant_->GetFrameByName("left_lower_leg");

  // Default evaluator: vertical normal, zero offset
  auto evaluator =
      DistanceEvaluator<double>(*plant_, pt_A, frame_A, pt_B, frame_B, .5);

  auto context = plant_->CreateDefaultContext();

  // Test q = pi/2, legs straight down
  VectorXd q = VectorXd::Zero(plant_->num_positions());
  q(4) = M_PI / 2.0;
  plant_->SetPositions(context.get(), q);
  VectorXd v = Eigen::VectorXd::Random(plant_->num_velocities());
  plant_->SetVelocities(context.get(), v);

  auto phi = evaluator.EvalFull(*context);
  VectorXd phi_expected(1);
  phi_expected << sqrt(2) - 0.5;
  EXPECT_TRUE(CompareMatrices(phi_expected, phi, tolerance));

  auto J = evaluator.EvalFullJacobian(*context);
  auto Jdotv = evaluator.EvalFullJacobianDotTimesV(*context);

  // Compare J using numerical approximation in v direction
  double dt = 1e-8;
  plant_->SetPositions(context.get(), q + dt * v);
  auto phi_perturbed = evaluator.EvalFull(*context);
  EXPECT_TRUE(CompareMatrices(J * v, (phi_perturbed - phi) / dt, dt * 100));

  // Compare Jdotv using numerical approximation
  auto J_perturbed = evaluator.EvalFullJacobian(*context);
  auto Jdot_approx = (J_perturbed - J) / dt;
  EXPECT_TRUE(CompareMatrices(Jdotv, Jdot_approx * v, dt * 100));
}

}  // namespace
}  // namespace multibody
}  // namespace dairlib

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
