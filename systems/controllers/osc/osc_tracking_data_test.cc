#include <memory>
#include <utility>

#include <gtest/gtest.h>
#include "examples/Cassie/cassie_utils.h"
#include "systems/controllers/osc/osc_tracking_data.h"
#include "systems/controllers/osc/osc_user_defined_pos.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace dairlib {
namespace systems {
namespace {

using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::System;
using drake::systems::SystemOutput;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::make_unique;
using std::unique_ptr;

class ComPos final : public controllers::OscUserDefinedPos {
 public:
  ComPos(const drake::multibody::MultibodyPlant<double>& plant)
      : plant_(plant), context_(plant.CreateDefaultContext()) {}

  // Users define their own position class in the derived class.
  Eigen::VectorXd Position(const Eigen::VectorXd& q) const final {
    plant_.SetPositions(context_.get(), q);
    return plant_.CalcCenterOfMassPosition(*context_);
//    VectorXd ret(1);
//    ret << 0.5 * q(0) * q(0);
////    ret << 0.5 * q(1) * q(1);
////    ret << 0.5 * q(0) * q(0) +  0.5 * q(0) * q(1);
////    ret << 0.5 * q(4) * q(5) +  0.5 * q(5) * q(5) +  0.5 * q(0) * q(0) +  0.5 * q(0) * q(1);
//    return ret;
  }

 private:
  const drake::multibody::MultibodyPlant<double>& plant_;
  std::unique_ptr<drake::systems::Context<double>> context_;
};
class LeftToePos final : public controllers::OscUserDefinedPos {
 public:
  LeftToePos(const drake::multibody::MultibodyPlant<double>& plant)
      : plant_(plant), context_(plant.CreateDefaultContext()) {}

  // Users define their own position class in the derived class.
  Eigen::VectorXd Position(const Eigen::VectorXd& q) const final {
    plant_.SetPositions(context_.get(), q);
    auto left_toe = LeftToe(plant_);
    VectorXd ret(3);
    plant_.CalcPointsPositions(*context_, left_toe.second, left_toe.first,
                               plant_.world_frame(), &ret);
    return ret;
  }

 private:
  const drake::multibody::MultibodyPlant<double>& plant_;
  std::unique_ptr<drake::systems::Context<double>> context_;
};

class TrackingDataTest : public ::testing::Test {
 protected:
  TrackingDataTest()
      : plant_w_spring_(drake::multibody::MultibodyPlant<double>(1e-3)),
        plant_wo_spring_(drake::multibody::MultibodyPlant<double>(1e-3)) {
    addCassieMultibody(&plant_w_spring_, nullptr, true /*floating base*/,
                       "examples/Cassie/urdf/cassie_v2.urdf",
                       true /*spring model*/, false /*loop closure*/);
    plant_w_spring_.Finalize();
    addCassieMultibody(&plant_wo_spring_, nullptr, true /*floating base*/,
                       "examples/Cassie/urdf/cassie_fixed_springs.urdf",
                       false /*spring model*/, false /*loop closure*/);
    plant_wo_spring_.Finalize();

    context_wo_spring_ = plant_wo_spring_.CreateDefaultContext();

    n_q_ = plant_wo_spring_.num_positions();
    n_v_ = plant_wo_spring_.num_velocities();

    x_samples_ = std::vector<VectorXd>(1, VectorXd::Zero(n_q_ + n_v_));
    VectorXd q(n_q_);
    VectorXd v(n_v_);
    VectorXd x(n_q_ + n_v_);
    q << 0.990065, 0.000339553, 0.00444831, 0.00085048, 0.00836164,
        -0.000249535, 1.03223, -0.000810813, 6.8811e-05, 0.00177426,
        -0.00514383, 0.447568, 0.44727, -1.01775, -1.01819, 1.29924, 1.30006,
        -1.56023, -1.56018;
    v << 0.156632, -0.0502397, 0.101071, 0.232441, -0.296125, -0.0559459,
        -0.663525, 0.116557, -0.0264677, -0.107556, 2.18153, -0.0230963,
        -1.65117, -1.02961, 1.75789, -0.0410481, -1.46269, 0.482573;
    x << q, v;
    x_samples_[0] = x;
//    q << 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
//    v << 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
//    x << q, v;
//    x_samples_[1] = x;
//    q << 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
//    v << 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
//    x << q, v;
//    x_samples_[2] = x;
  }

  drake::multibody::MultibodyPlant<double> plant_w_spring_;
  drake::multibody::MultibodyPlant<double> plant_wo_spring_;
  std::unique_ptr<drake::systems::Context<double>> context_wo_spring_;
  std::vector<VectorXd> x_samples_;
  int n_q_;
  int n_v_;
};

TEST_F(TrackingDataTest, AbstractTrackingData1) {

  // TODO: You can use Drake's API for Pevlis origin to see if it gives the same answer?

  ComPos user_defined_com_pos(plant_wo_spring_);
  int n_r = user_defined_com_pos
      .Position(VectorXd::Ones(plant_wo_spring_.num_positions()))
      .size();
  controllers::AbstractTrackingData abstract_tracking_data(
      "COM", n_r, MatrixXd::Zero(n_r, n_r), MatrixXd::Zero(n_r, n_r),
      MatrixXd::Zero(n_r, n_r), &plant_w_spring_, &plant_wo_spring_,
      &user_defined_com_pos);

  for (auto x : x_samples_) {
    std::cout << "\n********************************************************\n";
    std::cout << "x = " << x.transpose() << std::endl;

    plant_wo_spring_.SetPositionsAndVelocities(context_wo_spring_.get(), x);

    MatrixXd J_drake(3, n_v_);
    plant_wo_spring_.CalcJacobianCenterOfMassTranslationalVelocity(
        *context_wo_spring_, drake::multibody::JacobianWrtVariable::kV,
        plant_wo_spring_.world_frame(), plant_wo_spring_.world_frame(),
        &J_drake);
    VectorXd JdotV_drake =
        plant_wo_spring_.CalcBiasCenterOfMassTranslationalAcceleration(
            *context_wo_spring_, drake::multibody::JacobianWrtVariable::kV,
            plant_wo_spring_.world_frame(), plant_wo_spring_.world_frame());

    controllers::OscTrackingData* tracking_data = &abstract_tracking_data;
    tracking_data->UpdateJAndJdotVForUnitTest(x, *context_wo_spring_);

    MatrixXd J_user = tracking_data->GetJ();
    VectorXd JdotV_user = tracking_data->GetJdotTimesV();

    std::cout << "(J_drake - J_user).norm() = " << (J_drake - J_user).norm() << std::endl;
    std::cout << "\nv_drake = " << (J_drake * x.tail(n_v_)).transpose() << std::endl;
    std::cout << "v_user = " << (J_user * x.tail(n_v_)).transpose() << std::endl;
    std::cout << "\nJdotV_drake = " << JdotV_drake.transpose() << std::endl;
    std::cout << "JdotV_user = " << JdotV_user.transpose() << std::endl;
    std::cout << "(JdotV_drake - JdotV_user) = " << (JdotV_drake - JdotV_user).transpose() << std::endl;
    std::cout << "(JdotV_drake - JdotV_user).norm() = " << (JdotV_drake - JdotV_user).norm() << std::endl;

    EXPECT_TRUE((J_drake - J_user).norm() < 1e-7);
    EXPECT_TRUE((JdotV_drake - JdotV_user).norm() < 1e-7);
  }
}


TEST_F(TrackingDataTest, AbstractTrackingData2) {
  // Toe
  auto left_toe = LeftToe(plant_wo_spring_);

  LeftToePos user_defined_com_pos(plant_wo_spring_);
  int n_r = user_defined_com_pos
      .Position(VectorXd::Ones(plant_wo_spring_.num_positions()))
      .size();
  controllers::AbstractTrackingData abstract_tracking_data(
      "left_toe", n_r, MatrixXd::Zero(n_r, n_r), MatrixXd::Zero(n_r, n_r),
      MatrixXd::Zero(n_r, n_r), &plant_w_spring_, &plant_wo_spring_,
      &user_defined_com_pos);

  for (auto x : x_samples_) {
    std::cout << "\n********************************************************\n";
    std::cout << "x = " << x.transpose() << std::endl;

    plant_wo_spring_.SetPositionsAndVelocities(context_wo_spring_.get(), x);

    MatrixXd J_drake(3, n_v_);
    plant_wo_spring_.CalcJacobianTranslationalVelocity(
        *context_wo_spring_, drake::multibody::JacobianWrtVariable::kV,
        left_toe.second, left_toe.first, plant_wo_spring_.world_frame(),
        plant_wo_spring_.world_frame(), &J_drake);
    VectorXd JdotV_drake =
        plant_wo_spring_
            .CalcBiasSpatialAcceleration(
                *context_wo_spring_, drake::multibody::JacobianWrtVariable::kV,
                left_toe.second, left_toe.first, plant_wo_spring_.world_frame(),
                plant_wo_spring_.world_frame())
            .translational();

    controllers::OscTrackingData* tracking_data = &abstract_tracking_data;
    tracking_data->UpdateJAndJdotVForUnitTest(x, *context_wo_spring_);

    MatrixXd J_user = tracking_data->GetJ();
    VectorXd JdotV_user = tracking_data->GetJdotTimesV();

std::cout << "(J_drake - J_user).norm() = " << (J_drake - J_user).norm() << std::endl;
    std::cout << "\nv_drake = " << (J_drake * x.tail(n_v_)).transpose() << std::endl;
    std::cout << "v_user = " << (J_user * x.tail(n_v_)).transpose() << std::endl;
    std::cout << "\nJdotV_drake = " << JdotV_drake.transpose() << std::endl;
    std::cout << "JdotV_user = " << JdotV_user.transpose() << std::endl;
std::cout << "(JdotV_drake - JdotV_user) = " << (JdotV_drake - JdotV_user).transpose() << std::endl;
std::cout << "(JdotV_drake - JdotV_user).norm() = " << (JdotV_drake - JdotV_user).norm() << std::endl;

    EXPECT_TRUE((J_drake - J_user).norm() < 1e-7);
    EXPECT_TRUE((JdotV_drake - JdotV_user).norm() < 1e-7);
  }
}

}  // namespace
}  // namespace systems
}  // namespace dairlib

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
