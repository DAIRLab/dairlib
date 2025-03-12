#include <gtest/gtest.h>

#include "systems/controllers/footstep_planning/alip_utils.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace dairlib::systems::controllers::alip_utils {

using Eigen::Vector2d;
using Eigen::Vector4d;
using Eigen::Matrix4d;
using Eigen::Matrix;

namespace {

class AlipGaitTest : public ::testing::Test {
 protected:
  void SetUp() override {
    test_gait = AlipGaitParams{
        0.85,
        32.0,
        0.3,
        0.1,
        0.3,
        Vector2d::UnitX(),
        Stance::kLeft,
        ResetDiscretization::kZOH
    };
    std::tie(x0, x1) = MakePeriodicAlipGait(test_gait);
  }
  Vector4d x0;
  Vector4d x1;
  AlipGaitParams test_gait;
};

TEST_F(AlipGaitTest, P2Projection) {
  Matrix4d Pi0;
  Matrix4d Pi1;
  Matrix<double, 4, 2> d0;
  Matrix<double, 4, 2> d1;
  MakeProjectionToP2Orbit(test_gait, Pi0, Pi1, d0, d1);
  Eigen::Vector4d t0 = Pi0 * (x0 - d0 * test_gait.desired_velocity);
  Eigen::Vector4d t1 = Pi0 * (x1 - d1 * test_gait.desired_velocity);

  EXPECT_TRUE(drake::CompareMatrices(t0, Vector4d::Zero(), 1e-10));
  EXPECT_TRUE(drake::CompareMatrices(t1, Vector4d::Zero(), 1e-10));
}

}
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}