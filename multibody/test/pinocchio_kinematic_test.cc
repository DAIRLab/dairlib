#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "common/find_resource.h"
#include "multibody/multibody_utils.h"
#include "multibody/pinocchio_plant.h"

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsing/parser.h"

namespace dairlib {
namespace multibody {
namespace {

using drake::AutoDiffXd;
using drake::math::ExtractGradient;
using drake::math::ExtractValue;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using Eigen::VectorXd;

/// Comparisons against MultibodyPlant

class PinocchioKinematicTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Building a floating-base plant
    std::string full_name = FindResourceOrThrow(
        "examples/Cassie/urdf/cassie_fixed_springs_conservative.urdf");

    plant_ = std::make_unique<MultibodyPlant<double>>(dt_);
    pin_plant_ = std::make_unique<dairlib::multibody::PinocchioPlant<double>>(
        dt_, full_name);
    Parser parser(plant_.get());
    Parser pin_parser((MultibodyPlant<double>*)pin_plant_.get());
    parser.AddModelFromFile(full_name);
    pin_parser.AddModelFromFile(full_name);
    plant_->Finalize();
    pin_plant_->Finalize();
    pin_plant_->FinalizePlant();

    plant_ad_ = drake::systems::System<double>::ToAutoDiffXd(*plant_);
    pin_plant_ad_ =
        std::make_unique<dairlib::multibody::PinocchioPlant<AutoDiffXd>>(
            *plant_, full_name);
    pin_plant_ad_->FinalizePlant();

    context_ = plant_->CreateDefaultContext();
    pin_context_ = pin_plant_->CreateDefaultContext();
    context_ad_ = plant_ad_->CreateDefaultContext();
    pin_context_ad_ = pin_plant_ad_->CreateDefaultContext();

    foot_frame_ = &plant_->GetBodyByName("toe_left").body_frame();
    pin_foot_frame_ = &pin_plant_->GetBodyByName("toe_left").body_frame();
    foot_frame_ad_ = &plant_ad_->GetBodyByName("toe_left").body_frame();
    pin_foot_frame_ad_ = &pin_plant_ad_->GetBodyByName("toe_left").body_frame();
    world_ = &plant_->world_frame();
    pin_world_ = &pin_plant_->world_frame();
    world_ad_ = &plant_ad_->world_frame();
    pin_world_ad_ = &pin_plant_ad_->world_frame();
    toe_front_ = Eigen::Vector3d(-0.0457, 0.112, 0);
    toe_front_ad_ =
        drake::math::InitializeAutoDiff(Eigen::Vector3d(-0.0457, 0.112, 0));

    x_ = VectorXd::Zero(plant_->num_positions() + plant_->num_velocities());
    x_(0) = 0.56270512;
    x_(1) = -0.32191005;
    x_(2) = 0.13835005;
    x_(3) = 0.74872968;
    x_.tail(12) = VectorXd::Random(12);
    x_ad_ = drake::math::InitializeAutoDiff(x_);
  }

  double tol = 1e-5;
  double dt_ = 1e-3;
  std::unique_ptr<MultibodyPlant<double>> plant_{};
  std::unique_ptr<PinocchioPlant<double>> pin_plant_{};
  Eigen::Vector3d toe_front_;
  drake::Vector3<AutoDiffXd> toe_front_ad_;
  const drake::multibody::BodyFrame<double>* foot_frame_;
  const drake::multibody::BodyFrame<double>* pin_foot_frame_;
  const drake::multibody::BodyFrame<AutoDiffXd>* foot_frame_ad_;
  const drake::multibody::BodyFrame<AutoDiffXd>* pin_foot_frame_ad_;
  const drake::multibody::Frame<double>* world_;
  const drake::multibody::Frame<double>* pin_world_;
  const drake::multibody::Frame<AutoDiffXd>* world_ad_;
  const drake::multibody::Frame<AutoDiffXd>* pin_world_ad_;
  std::unique_ptr<MultibodyPlant<AutoDiffXd>> plant_ad_{};
  std::unique_ptr<PinocchioPlant<AutoDiffXd>> pin_plant_ad_{};
  std::unique_ptr<drake::systems::Context<double>> context_;
  std::unique_ptr<drake::systems::Context<AutoDiffXd>> context_ad_;
  std::unique_ptr<drake::systems::Context<double>> pin_context_;
  std::unique_ptr<drake::systems::Context<AutoDiffXd>> pin_context_ad_;

  VectorXd x_;
  drake::VectorX<AutoDiffXd> x_ad_;
};

TEST_F(PinocchioKinematicTest, TestMassMatrixDouble) {
  int nv = plant_->num_velocities();
  plant_->SetPositionsAndVelocities(context_.get(), x_);
  pin_plant_->SetPositionsAndVelocities(pin_context_.get(), x_);
  Eigen::MatrixXd M(nv, nv);
  Eigen::MatrixXd pin_M(nv, nv);

  plant_->CalcMassMatrix(*context_, &M);

  pin_plant_->CalcMassMatrix(*pin_context_, &pin_M);

  EXPECT_TRUE((M - pin_M).norm() < tol);
}

TEST_F(PinocchioKinematicTest, TestCenterOfMassDouble) {
  plant_->SetPositionsAndVelocities(context_.get(), x_);
  pin_plant_->SetPositionsAndVelocities(pin_context_.get(), x_);
  Eigen::Vector3d com = plant_->CalcCenterOfMassPositionInWorld(*context_);
  Eigen::Vector3d pin_com =
      pin_plant_->CalcCenterOfMassPositionInWorld(*pin_context_);

  EXPECT_TRUE((com - pin_com).norm() < tol);
}

TEST_F(PinocchioKinematicTest, TestCenterOfMassAD) {
  plant_ad_->SetPositionsAndVelocities(context_ad_.get(), x_ad_);
  pin_plant_ad_->SetPositionsAndVelocities(pin_context_ad_.get(), x_ad_);
  drake::Vector3<AutoDiffXd> com =
      plant_ad_->CalcCenterOfMassPositionInWorld(*context_ad_);
  drake::Vector3<AutoDiffXd> pin_com =
      pin_plant_ad_->CalcCenterOfMassPositionInWorld(*pin_context_ad_);

  EXPECT_TRUE((ExtractValue(com) - ExtractValue(pin_com)).norm() < tol);
  EXPECT_TRUE(ExtractGradient(com).rows() == ExtractGradient(pin_com).rows());
  EXPECT_TRUE(ExtractGradient(com).cols() == ExtractGradient(pin_com).cols());
  EXPECT_TRUE((ExtractGradient(com) - ExtractGradient(pin_com)).norm() < tol);
}

TEST_F(PinocchioKinematicTest, TestCentroidalMomentumDouble) {
  plant_->SetPositionsAndVelocities(context_.get(), x_);
  pin_plant_->SetPositionsAndVelocities(pin_context_.get(), x_);
  Eigen::Vector3d com = plant_->CalcCenterOfMassPositionInWorld(*context_);
  Eigen::Vector3d pin_com =
      pin_plant_->CalcCenterOfMassPositionInWorld(*pin_context_);
  auto spatial_momentum =
      plant_->CalcSpatialMomentumInWorldAboutPoint(*context_, com);
  auto pin_spatial_momentum =
      pin_plant_->CalcSpatialMomentumInWorldAboutPoint(*pin_context_, com);

  EXPECT_TRUE((com - pin_com).norm() < tol);
  EXPECT_TRUE(
      (spatial_momentum.translational() - pin_spatial_momentum.translational())
          .norm() < tol);
  EXPECT_TRUE(
      (spatial_momentum.rotational() - pin_spatial_momentum.rotational())
          .norm() < tol);
}

TEST_F(PinocchioKinematicTest, TestCentroidalMomentumAD) {
  plant_ad_->SetPositionsAndVelocities(context_ad_.get(), x_ad_);
  pin_plant_ad_->SetPositionsAndVelocities(pin_context_ad_.get(), x_ad_);
  drake::Vector3<AutoDiffXd> com =
      plant_ad_->CalcCenterOfMassPositionInWorld(*context_ad_);
  drake::Vector3<AutoDiffXd> pin_com =
      pin_plant_ad_->CalcCenterOfMassPositionInWorld(*pin_context_ad_);
  auto spatial_momentum =
      plant_ad_->CalcSpatialMomentumInWorldAboutPoint(*context_ad_, com);
  auto pin_spatial_momentum =
      pin_plant_ad_->CalcSpatialMomentumInWorldAboutPoint(*pin_context_ad_,
                                                          pin_com);

  EXPECT_TRUE((ExtractValue(spatial_momentum.translational()) -
               ExtractValue(pin_spatial_momentum.translational()))
                  .norm() < tol);
  EXPECT_TRUE((ExtractValue(spatial_momentum.rotational()) -
               ExtractValue(pin_spatial_momentum.rotational()))
                  .norm() < tol);
  EXPECT_TRUE((ExtractGradient(spatial_momentum.translational()) -
               ExtractGradient(pin_spatial_momentum.translational()))
                  .norm() < tol);
  EXPECT_TRUE((ExtractGradient(spatial_momentum.translational()) -
               ExtractGradient(pin_spatial_momentum.translational()))
                  .norm() < tol);
  EXPECT_TRUE((ExtractGradient(spatial_momentum.rotational()) -
               ExtractGradient(pin_spatial_momentum.rotational()))
                  .norm() < tol);
  EXPECT_TRUE((ExtractGradient(spatial_momentum.rotational()) -
               ExtractGradient(pin_spatial_momentum.rotational()))
                  .norm() < tol);
}

TEST_F(PinocchioKinematicTest, TestCenterOfMassVel) {
  Eigen::Vector3d com_vel =
      plant_->CalcCenterOfMassTranslationalVelocityInWorld(*context_);
  Eigen::Vector3d pin_com_vel =
      pin_plant_->CalcCenterOfMassTranslationalVelocityInWorld(*pin_context_);

  EXPECT_TRUE((com_vel - pin_com_vel).norm() < tol);
}

TEST_F(PinocchioKinematicTest, TestCalcPointsPositionDouble) {
  plant_->SetPositionsAndVelocities(context_.get(), x_);
  pin_plant_->SetPositionsAndVelocities(pin_context_.get(), x_);
  Eigen::Vector3d foot_pos;
  Eigen::Vector3d pin_foot_pos;
  plant_->CalcPointsPositions(*context_, *foot_frame_, toe_front_, *world_,
                              &foot_pos);
  pin_plant_->CalcPointsPositions(*pin_context_, *pin_foot_frame_, toe_front_,
                                  *pin_world_, &pin_foot_pos);
  EXPECT_TRUE((foot_pos - pin_foot_pos).norm() < tol);
}

TEST_F(PinocchioKinematicTest, TestCalcPointsPositionAD) {
  plant_ad_->SetPositionsAndVelocities(context_ad_.get(), x_ad_);
  pin_plant_ad_->SetPositionsAndVelocities(pin_context_ad_.get(), x_ad_);
  drake::Vector3<AutoDiffXd> foot_pos;
  drake::Vector3<AutoDiffXd> pin_foot_pos;
  plant_ad_->CalcPointsPositions(*context_ad_, *foot_frame_ad_, toe_front_ad_,
                                 *world_ad_, &foot_pos);
  pin_plant_ad_->CalcPointsPositions(*pin_context_ad_, *pin_foot_frame_ad_,
                                     toe_front_ad_, *pin_world_ad_,
                                     &pin_foot_pos);
  std::cout << ExtractGradient(foot_pos).leftCols(18) << std::endl;
  std::cout << ExtractGradient(pin_foot_pos) << std::endl;
  std::cout << ExtractGradient(foot_pos).rows() << std::endl;
  std::cout << ExtractGradient(foot_pos).cols() << std::endl;
  std::cout << ExtractGradient(pin_foot_pos).rows() << std::endl;
  std::cout << ExtractGradient(pin_foot_pos).cols() << std::endl;
  EXPECT_TRUE((ExtractValue(foot_pos) - ExtractValue(pin_foot_pos)).norm() <
              tol);
  EXPECT_TRUE(
      (ExtractGradient(foot_pos) - ExtractGradient(pin_foot_pos)).norm() < tol);
}

}  // namespace
}  // namespace multibody
}  // namespace dairlib

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
