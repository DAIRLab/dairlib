#include <memory>

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

  Eigen::VectorXd Position(const Eigen::VectorXd& q) const final {
    plant_.SetPositions(context_.get(), q);
    return plant_.CalcCenterOfMassPosition(*context_);
  }

 private:
  const drake::multibody::MultibodyPlant<double>& plant_;
  std::unique_ptr<drake::systems::Context<double>> context_;
};
class LeftToePos final : public controllers::OscUserDefinedPos {
 public:
  LeftToePos(const drake::multibody::MultibodyPlant<double>& plant)
      : plant_(plant), context_(plant.CreateDefaultContext()) {}

  Eigen::VectorXd Position(const Eigen::VectorXd& q) const final {
    plant_.SetPositions(context_.get(), q);
    auto left_toe = LeftToeFront(plant_);
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

    x_samples_ = std::vector<VectorXd>(2, VectorXd::Zero(n_q_ + n_v_));
    VectorXd q(n_q_);
    VectorXd v(n_v_);
    VectorXd x(n_q_ + n_v_);
    // Double support phase of Cassie walking
    q << 0.990065, 0.000339553, 0.00444831, 0.00085048, 0.00836164,
        -0.000249535, 1.03223, -0.000810813, 6.8811e-05, 0.00177426,
        -0.00514383, 0.447568, 0.44727, -1.01775, -1.01819, 1.29924, 1.30006,
        -1.56023, -1.56018;
    v << 0.156632, -0.0502397, 0.101071, 0.232441, -0.296125, -0.0559459,
        -0.663525, 0.116557, -0.0264677, -0.107556, 2.18153, -0.0230963,
        -1.65117, -1.02961, 1.75789, -0.0410481, -1.46269, 0.482573;
    x << q, v;
    x_samples_[0] = x;
    // Left single support phase of Cassie walking
    q << 0.989849, -0.000815987, -0.017933, -0.0111588, 0.344537, -0.148108,
        1.00902, -0.0357916, -0.0422061, -0.0068692, -0.0355008, 0.274222,
        0.644396, -1.00482, -1.50496, 1.36746, 1.73074, -1.45868, -0.936994;
    v << -0.110601, -0.0521661, -0.00286609, 0.910837, -0.0174017, -0.00158473,
        0.124156, 0.8427, 0.0224065, 0.0678774, -1.22403, 2.89698, 0.32455,
        2.21075, -0.333968, -2.51737, 1.36041, -4.312;
    x << q, v;
    x_samples_[1] = x;
  }

  drake::multibody::MultibodyPlant<double> plant_w_spring_;
  drake::multibody::MultibodyPlant<double> plant_wo_spring_;
  std::unique_ptr<drake::systems::Context<double>> context_wo_spring_;
  std::vector<VectorXd> x_samples_;
  int n_q_;
  int n_v_;
};

TEST_F(TrackingDataTest, AbstractTrackingData1) {
  ComPos user_defined_com_pos(plant_wo_spring_);
  controllers::AbstractTrackingData abstract_tracking_data(
      "COM", 3, MatrixXd::Zero(3, 3), MatrixXd::Zero(3, 3),
      MatrixXd::Zero(3, 3), &plant_w_spring_, &plant_wo_spring_,
      &user_defined_com_pos);

  for (auto x : x_samples_) {
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

    EXPECT_TRUE((J_drake - J_user).norm() < 5e-5);
    EXPECT_TRUE((JdotV_drake - JdotV_user).norm() < 5e-5);
  }
}


TEST_F(TrackingDataTest, AbstractTrackingData2) {
  // Toe
  auto left_toe = LeftToeFront(plant_wo_spring_);

  LeftToePos user_defined_com_pos(plant_wo_spring_);
  controllers::AbstractTrackingData abstract_tracking_data(
      "left_toe", 3, MatrixXd::Zero(3, 3), MatrixXd::Zero(3, 3),
      MatrixXd::Zero(3, 3), &plant_w_spring_, &plant_wo_spring_,
      &user_defined_com_pos);

  for (auto x : x_samples_) {
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

    EXPECT_TRUE((J_drake - J_user).norm() < 5e-5);
    EXPECT_TRUE((JdotV_drake - JdotV_user).norm() < 5e-5);
  }
}

}  // namespace
}  // namespace systems
}  // namespace dairlib

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
