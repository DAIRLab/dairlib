#include <memory>
#include <utility>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/parsing/parser.h"

#include "common/find_resource.h"
#include "multibody/kinematic/kinematic_evaluator.h"
#include "multibody/kinematic/planar_ground_evaluator.h"

namespace dairlib {
namespace multibody {
namespace {

using drake::CompareMatrices;
using drake::multibody::MultibodyPlant;
using drake::geometry::SceneGraph;
using drake::multibody::Body;
using drake::multibody::Parser;

using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;

class KinematicEvaluatortEst : public ::testing::Test {
 protected:
  void SetUp() override {
    plant_ = std::make_unique<MultibodyPlant<double>>(0.0);
    auto scene_graph = std::make_unique<SceneGraph<double>>();
    Parser parser(plant_.get(), scene_graph.get());
    std::string full_name =
        dairlib::FindResourceOrThrow("examples/PlanarWalker/PlanarWalker.urdf");

    parser.AddModelFromFile(full_name);

    plant_->WeldFrames(
        plant_->world_frame(), plant_->GetFrameByName("base"),
        drake::math::RigidTransform<double>());

    plant_->Finalize();
  }

  std::unique_ptr<MultibodyPlant<double>> plant_;
};

TEST_F(KinematicEvaluatortEst, PlanarGroundEvaluatorTest) {
  const double tolerance = 1e-10;



  Vector3d pt_A({0, 0, -.5});
  const auto& frame = plant_->GetFrameByName("right_lower_leg");

  // Default constraint: vertical normal, zero offset
  auto constraint = PlanarGroundEvaluator<double>(*plant_, pt_A, frame,
      Vector3d({0, 0, 1}), Vector3d::Zero(), false);

  auto context = plant_->CreateDefaultContext();

  // Test q = 0, legs straight down
  VectorXd q(plant_->num_positions());
  plant_->SetPositions(context.get(), q);
  VectorXd v = Eigen::VectorXd::Constant(plant_->num_velocities(), 1);
  plant_->SetVelocities(context.get(), v);

  auto phi = constraint.EvalFull(*context);
  EXPECT_TRUE(CompareMatrices(Vector3d({0, 0, -1}), phi, tolerance));

  auto phi_active = constraint.EvalActive(*context);

  VectorXd phi_active_expected(1);
  phi_active_expected << -1;
  EXPECT_TRUE(CompareMatrices(phi_active_expected, phi_active, tolerance));

  auto J = constraint.EvalFullJacobian(*context);
  MatrixXd J_expected(3, 6);
  J_expected.row(0) << 0, 0, 0, 0, 0, 0;
  J_expected.row(1) << 1, 0, -1, -1, 0, -.5;
  J_expected.row(2) << 0, 1, 0, 0, 0, 0;
  EXPECT_TRUE(CompareMatrices(J, J_expected, tolerance));

  auto J_active = constraint.EvalActiveJacobian(*context);
  MatrixXd J_active_expected(1, 6);
  J_active_expected << 0, 1, 0, 0, 0, 0;
  EXPECT_TRUE(CompareMatrices(J_active, J_active_expected, tolerance));

  auto phidot = constraint.EvalFullTimeDerivative(*context);
  VectorXd phidot_expected(3);
  phidot_expected << 0, -1.5, 1;
  EXPECT_TRUE(CompareMatrices(phidot, phidot_expected, tolerance));

  auto phidot_active = constraint.EvalActiveTimeDerivative(*context);
  VectorXd phidot_active_expected(1);
  phidot_active_expected << 1;
  EXPECT_TRUE(CompareMatrices(phidot_active, phidot_active_expected,
      tolerance));

  auto Jdotv = constraint.EvalFullJacobianDotTimesV(*context);
  VectorXd Jdotv_expected(3);
  Jdotv_expected << 0, 0, 6.5;
  EXPECT_TRUE(CompareMatrices(Jdotv, Jdotv_expected, tolerance));

  auto Jdotv_active = constraint.EvalActiveJacobianDotTimesV(*context);
  VectorXd Jdotv_active_expected(1);
  Jdotv_active_expected << 6.5;
  EXPECT_TRUE(CompareMatrices(Jdotv_active, Jdotv_active_expected, tolerance));

  // Non-default constraint: non-vertical normal, non-zero offset
  // Performing minmal tests
  auto new_constraint = PlanarGroundEvaluator<double>(*plant_, pt_A,
      frame, Vector3d({1, 0, 0}), Vector3d({1, 2, 3}), true);

  auto new_phi = new_constraint.EvalFull(*context);
  EXPECT_TRUE(CompareMatrices(Vector3d({-4, 2, -1}), new_phi, tolerance));
  auto new_phi_active = new_constraint.EvalActive(*context);
  EXPECT_TRUE(CompareMatrices(Vector3d({-4, 2, -1}), new_phi_active,
      tolerance));  
}


// // Contact Jacobian test
// TEST_F(ContactToolkitTest, ContactJacobianTest) {
//   VectorX<double> x_double = x0_;
//   VectorX<AutoDiffXd> x_autodiff = initializeAutoDiff(x0_);
//   MatrixX<double> jac_double =
//       contact_toolkit_->CalcContactJacobian(x_double);
//   MatrixX<AutoDiffXd> jac_autodiff =
//       contact_toolkit_autodiff_->CalcContactJacobian(x_autodiff);

//   // Checking dimensions of the jacobian
//   // Each contact has three directional jacobian componenets - the normal and
//   // two surface tangents.
//   ASSERT_EQ(jac_double.rows(), contact_toolkit_->get_num_contacts() * 3);
//   ASSERT_EQ(jac_double.cols(), plant_->num_positions());

//   ASSERT_EQ(jac_autodiff.rows(),
//             contact_toolkit_autodiff_->get_num_contacts() * 3);
//   ASSERT_EQ(jac_autodiff.cols(), plant_->num_positions());
// }

// // MVDot test
// TEST_F(ContactToolkitTest, MVDotTest) {
//   VectorX<double> x_double = x0_;
//   VectorX<double> u_double = VectorXd::Random(plant_->num_actuators());
//   VectorX<double> lambda_double =
//       VectorXd::Random(tree_.getNumPositionConstraints() +
//                        contact_toolkit_->get_num_contacts() * 3);
//   VectorX<double> mvdot_double =
//       contact_toolkit_->CalcMVDot(x_double, u_double, lambda_double);

//   VectorX<AutoDiffXd> x_autodiff = initializeAutoDiff(x_double);
//   VectorX<AutoDiffXd> u_autodiff = initializeAutoDiff(u_double);
//   VectorX<AutoDiffXd> lambda_autodiff = initializeAutoDiff(lambda_double);
//   VectorX<AutoDiffXd> mvdot_autodiff = contact_toolkit_autodiff_->CalcMVDot(
//       x_autodiff, u_autodiff, lambda_autodiff);

//   // Verifying the dimensions
//   ASSERT_EQ(mvdot_double.rows(), plant_->num_velocities());
//   ASSERT_EQ(mvdot_autodiff.rows(), plant_->num_velocities());

//   // Verifying that both templates return the same value
//   ASSERT_TRUE(mvdot_double.isApprox(DiscardGradient(mvdot_autodiff)));
// }

// // Time derivatives test
// TEST_F(ContactToolkitTest, TimeDerivativesTest) {
//   VectorX<double> x_double = x0_;
//   VectorX<double> u_double = VectorXd::Random(plant_->num_actuators());
//   VectorX<double> lambda_double =
//       VectorXd::Random(tree_.getNumPositionConstraints() +
//                        contact_toolkit_->get_num_contacts() * 3);
//   VectorX<double> xdot_double = contact_toolkit_->CalcTimeDerivatives(
//       x_double, u_double, lambda_double);

//   VectorX<AutoDiffXd> x_autodiff = initializeAutoDiff(x_double);
//   VectorX<AutoDiffXd> u_autodiff = initializeAutoDiff(u_double);
//   VectorX<AutoDiffXd> lambda_autodiff = initializeAutoDiff(lambda_double);
//   VectorX<AutoDiffXd> xdot_autodiff =
//       contact_toolkit_autodiff_->CalcTimeDerivatives(x_autodiff, u_autodiff,
//                                                      lambda_autodiff);

//   // Verifying the dimensions
//   ASSERT_EQ(xdot_double.rows(),
//             plant_->num_positions() + plant_->num_velocities());
//   ASSERT_EQ(xdot_autodiff.rows(),
//             plant_->num_velocities() + plant_->num_velocities());

//   // Verifying that both templates return the same value
//   ASSERT_TRUE(xdot_double.isApprox(DiscardGradient(xdot_autodiff)));
// }

}  // namespace
}  // namespace systems
}  // namespace dairlib

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
