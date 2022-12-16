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
    std::cout << "Constructing double plants" << std::endl;

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

    std::cout << "Finished constructing double plants" << std::endl;

    plant_ad_ = drake::systems::System<double>::ToAutoDiffXd(*plant_);
    //    plant_ad_->Finalize();
    pin_plant_ad_ =
        std::make_unique<dairlib::multibody::PinocchioPlant<AutoDiffXd>>(
            *plant_, full_name);
    pin_plant_ad_->FinalizePlant();

    context_ = plant_->CreateDefaultContext();
    pin_context_ = pin_plant_->CreateDefaultContext();
    context_ad_ = plant_ad_->CreateDefaultContext();
    pin_context_ad_ = pin_plant_ad_->CreateDefaultContext();

    x_ = VectorXd::Zero(plant_->num_positions() + plant_->num_velocities());
    x_(0) = 0.467;
    x_(1) = 0.003;
    x_(2) = 0.839;
    x_(3) = -0.280;
    x_(4) = 1;
    x_(10) = 1;
    x_ad_ = drake::math::InitializeAutoDiff(x_);
  }

  double tol = 1e-5;
  double dt_ = 1e-3;
  std::unique_ptr<MultibodyPlant<double>> plant_{};
  std::unique_ptr<PinocchioPlant<double>> pin_plant_{};
  std::unique_ptr<MultibodyPlant<AutoDiffXd>> plant_ad_{};
  std::unique_ptr<PinocchioPlant<AutoDiffXd>> pin_plant_ad_{};
  std::unique_ptr<drake::systems::Context<double>> context_;
  std::unique_ptr<drake::systems::Context<AutoDiffXd>> context_ad_;
  std::unique_ptr<drake::systems::Context<double>> pin_context_;
  std::unique_ptr<drake::systems::Context<AutoDiffXd>> pin_context_ad_;

  VectorXd x_;
  drake::VectorX<AutoDiffXd> x_ad_;
};

// template <>
// bool PinocchioPlant<double>::TestInverseDynamics(
//    const drake::systems::Context<double>& context, const VectorXd&
//    known_vdot, const drake::multibody::MultibodyForces<double>&
//    external_forces, double tol) const {
//  auto f = MultibodyPlant<double>::CalcInverseDynamics(context, known_vdot,
//                                                       external_forces);
//  auto pin_f = CalcInverseDynamics(context, known_vdot, external_forces);
//
//  return (f - pin_f).norm() < tol;
//}
//
// template <>
// bool PinocchioPlant<double>::TestMassMatrix(const Context<double>& context,
//                                            double tol) const {
//  int nv = num_velocities();
//
//  MatrixXd M(nv, nv);
//  MatrixXd pin_M(nv, nv);
//
//  MultibodyPlant<double>::CalcMassMatrix(context, &M);
//
//  CalcMassMatrix(context, &pin_M);
//
//  return (M - pin_M).norm() < tol;
//}

TEST_F(PinocchioKinematicTest, TestCenterOfMassDouble) {
  plant_->SetPositionsAndVelocities(context_.get(), x_);
  pin_plant_->SetPositionsAndVelocities(pin_context_.get(), x_);
  Eigen::Vector3d com = plant_->CalcCenterOfMassPositionInWorld(*context_);
  Eigen::Vector3d pin_com =
      pin_plant_->CalcCenterOfMassPositionInWorld(*pin_context_);

  std::cout << "com = " << com.transpose() << std::endl;
  std::cout << "pin_com = " << pin_com.transpose() << std::endl;

  EXPECT_TRUE((com - pin_com).norm() < tol);
}

TEST_F(PinocchioKinematicTest, TestCenterOfMassAD) {
  plant_ad_->SetPositionsAndVelocities(context_ad_.get(), x_ad_);
  pin_plant_ad_->SetPositionsAndVelocities(pin_context_ad_.get(), x_ad_);
  drake::Vector3<AutoDiffXd> com =
      plant_ad_->CalcCenterOfMassPositionInWorld(*context_ad_);
  drake::Vector3<AutoDiffXd> pin_com =
      pin_plant_ad_->CalcCenterOfMassPositionInWorld(*pin_context_ad_);
  std::cout << "com value = " << ExtractValue(com).transpose() << std::endl;
  std::cout << "pin_com value = " << ExtractValue(pin_com).transpose()
            << std::endl;
  std::cout << "com gradient = " << ExtractGradient(com).transpose()
            << std::endl;
  std::cout << "pin_com gradient = " << ExtractGradient(pin_com).transpose()
            << std::endl;

  EXPECT_TRUE((ExtractValue(com) - ExtractValue(pin_com)).norm() < tol);
  EXPECT_TRUE(ExtractGradient(com).rows() == ExtractGradient(pin_com).rows());
  EXPECT_TRUE(ExtractGradient(com).cols() == ExtractGradient(pin_com).cols());
  EXPECT_TRUE((ExtractGradient(com) - ExtractGradient(pin_com)).norm() < tol);
}

//
// template <>
// bool PinocchioPlant<double>::TestCenterOfMassVel(const Context<double>&
// context,
//                                                 double tol) const {
//  Eigen::Vector3d com_vel =
//      MultibodyPlant<double>::CalcCenterOfMassTranslationalVelocityInWorld(
//          context);
//  Eigen::Vector3d pin_com_vel =
//      CalcCenterOfMassTranslationalVelocityInWorld(context);
//  std::cout << "com_vel = " << com_vel.transpose() << std::endl;
//  std::cout << "pin_com_vel = " << pin_com_vel.transpose() << std::endl;
//
//  return (com_vel - pin_com_vel).norm() < tol;
//}
//
// template <>
// bool PinocchioPlant<double>::TestCenterOfMassJ(const Context<double>&
// context,
//                                               double tol) const {
//  int nv = num_velocities();
//
//  MatrixXd J(3, nv);
//  MatrixXd pin_J(3, nv);
//
//  MultibodyPlant<double>::CalcJacobianCenterOfMassTranslationalVelocity(
//      context, JacobianWrtVariable::kV, this->world_frame(),
//      this->world_frame(), &J);
//
//  CalcJacobianCenterOfMassTranslationalVelocity(
//      context, JacobianWrtVariable::kV, this->world_frame(),
//      this->world_frame(), &pin_J);
//  return (J - pin_J).norm() < tol;
//}
//
// template <>
// bool PinocchioPlant<AutoDiffXd>::TestInverseDynamics(
//    const drake::systems::Context<AutoDiffXd>& context,
//    const drake::VectorX<AutoDiffXd>& known_vdot,
//    const drake::multibody::MultibodyForces<AutoDiffXd>& external_forces,
//    double tol) const {
//  throw std::domain_error(
//      "TestInverseDynamics not implemented with AutoDiffXd");
//}
//
// template <>
// bool PinocchioPlant<AutoDiffXd>::TestMassMatrix(
//    const Context<AutoDiffXd>& context, double tol) const {
//  throw std::domain_error("TestMassMatrix not implemented with AutoDiffXd");
//}
//
//
// template <>
// bool PinocchioPlant<AutoDiffXd>::TestCenterOfMassVel(
//    const Context<AutoDiffXd>& context, double tol) const {
//  throw std::domain_error(
//      "CalcCenterOfMassPositionInWorld not implemented with AutoDiffXd");
//}
//
// template <>
// bool PinocchioPlant<AutoDiffXd>::TestCenterOfMassJ(
//    const Context<AutoDiffXd>& context, double tol) const {
//  throw std::domain_error("TestCenterOfMassJ not implemented with
//  AutoDiffXd");
//}

}  // namespace
}  // namespace multibody
}  // namespace dairlib

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
