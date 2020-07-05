#include <memory>
#include <gtest/gtest.h>

#include "examples/Cassie/cassie_utils.h"
#include "examples/goldilocks_models/reduced_order_models.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace dairlib {
namespace goldilocks_models {
namespace {

using drake::VectorX;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::System;
using drake::systems::SystemOutput;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::make_unique;
using std::unique_ptr;

class MonomialFeatureTest : public ::testing::Test {
 protected:
  MonomialFeatureTest() {}
};

TEST_F(MonomialFeatureTest, FeatureLengthTest) {
  EXPECT_TRUE(MonomialFeatures(2, 2).length() == 6);
  // 1 + 4 + (4Choose2 + 4) = 1 + 4 + 10 = 15
  EXPECT_TRUE(MonomialFeatures(2, 4).length() == 15);
  // 1 + 6 + (6Choose2 + 6) = 1 + 6 + 21 = 28
  EXPECT_TRUE(MonomialFeatures(2, 6).length() == 28);
  // 1 + 8 + (8Choose2 + 8) = 1 + 8 + 36 = 45
  EXPECT_TRUE(MonomialFeatures(2, 8).length() == 45);
}

TEST_F(MonomialFeatureTest, SecondOrder) {
  /*
    order = 2 and n_q = 3

    Features =
      row index : symbolic term
      0: ()
      1: (0)
      2: (0, 0)
      3: (0, 1)
      4: (0, 2)
      5: (1)
      6: (1, 1)
      7: (1, 2)
      8: (2)
      9: (2, 2)
    First order partial derivatives =
      Key ==> Term
      1, (0) ==> 1, ()
      2, (0) ==> 2, (0)
      3, (0) ==> 1, (1)
      3, (1) ==> 1, (0)
      4, (0) ==> 1, (2)
      4, (2) ==> 1, (0)
      5, (1) ==> 1, ()
      6, (1) ==> 2, (1)
      7, (1) ==> 1, (2)
      7, (2) ==> 1, (1)
      8, (2) ==> 1, ()
      9, (2) ==> 2, (2)
    Second order partial derivatives =
      Key ==> Term
      2, (0, 0) ==> 2, ()
      3, (0, 1) ==> 2, ()
      4, (0, 2) ==> 2, ()
      6, (1, 1) ==> 2, ()
      7, (1, 2) ==> 2, ()
      9, (2, 2) ==> 2, ()
   */

  MonomialFeatures features(2, 3);
  EXPECT_TRUE(features.length() == 10);

  VectorX<double> q(3);
  q << 0, 1, 0.5;
  VectorX<double> qdot(3);
  qdot << 0.3, 0, 0;

  VectorX<double> expected_feature(10);
  expected_feature << 1, 0, 0, 0, 0, 1, 1, 0.5, 0.5, 0.25;
  EXPECT_TRUE((features.Eval(q) - expected_feature).norm() == 0);

  VectorX<double> expected_JV(10);
  expected_JV << 0, 0.3, 0, 0.3, 0.15, 0, 0, 0, 0, 0;
  EXPECT_TRUE((features.EvalJV(q, qdot) - expected_JV).norm() == 0);

  VectorX<double> expected_JdotV(10);
  expected_JdotV << 0, 0, 0.18, 0, 0, 0, 0, 0, 0, 0;
  EXPECT_TRUE((features.EvalJdotV(q, qdot) - expected_JdotV).norm() == 0);

  //  features.PrintSymbolicFeatures();
  //  features.PrintSymbolicPartialDerivatives(1);
  //  features.PrintSymbolicPartialDerivatives(2);
  //  cout << "==================\n";
  //  cout << features.Eval(q) << endl;
  //  cout << "==================\n";
  //  cout << features.EvalJV(q, qdot) << endl;
  //  cout << "==================\n";
  //  cout << features.EvalJdotV(q, qdot) << endl;
}

TEST_F(MonomialFeatureTest, HighOrder) {
  /*
    order = 3 and n_q = 2

    Features =
      row index : symbolic term
      0: ()
      1: (0)
      2: (0, 0)
      3: (0, 0, 0)
      4: (0, 0, 1)
      5: (0, 1)
      6: (0, 1, 1)
      7: (1)
      8: (1, 1)
      9: (1, 1, 1)
    First order partial derivatives =
      Key ==> Term
      1, (0) ==> 1, ()
      2, (0) ==> 2, (0)
      3, (0) ==> 3, (0, 0)
      4, (0) ==> 2, (0, 1)
      4, (1) ==> 1, (0, 0)
      5, (0) ==> 1, (1)
      5, (1) ==> 1, (0)
      6, (0) ==> 1, (1, 1)
      6, (1) ==> 2, (0, 1)
      7, (1) ==> 1, ()
      8, (1) ==> 2, (1)
      9, (1) ==> 3, (1, 1)
    Second order partial derivatives =
      Key ==> Term
      2, (0, 0) ==> 2, ()
      3, (0, 0) ==> 6, (0)
      4, (0, 0) ==> 2, (1)
      4, (0, 1) ==> 4, (0)
      5, (0, 1) ==> 2, ()
      6, (0, 1) ==> 4, (1)
      6, (1, 1) ==> 2, (0)
      8, (1, 1) ==> 2, ()
      9, (1, 1) ==> 6, (1)
   */

  MonomialFeatures features(3, 2);
  EXPECT_TRUE(features.length() == 10);

  VectorX<double> q(2);
  q << 0, 0.5;
  VectorX<double> qdot(2);
  qdot << 0.3, 0;

  VectorX<double> expected_feature(10);
  expected_feature << 1, 0, 0, 0, 0, 0, 0, 0.5, 0.25, 0.125;
  EXPECT_TRUE((features.Eval(q) - expected_feature).norm() == 0);

  VectorX<double> expected_JV(10);
  expected_JV << 0, 0.3, 0, 0, 0, 0.15, 0.075, 0, 0, 0;
  EXPECT_TRUE((features.EvalJV(q, qdot) - expected_JV).norm() == 0);

  VectorX<double> expected_JdotV(10);
  expected_JdotV << 0, 0, 0.18, 0, 0.09, 0, 0, 0, 0, 0;
  EXPECT_TRUE((features.EvalJdotV(q, qdot) - expected_JdotV).norm() == 0);
}

TEST_F(MonomialFeatureTest, SkipIndices) {
  /*
    order = 2, n_q = 3 and skip the second element of q

    Features =
      row index : symbolic term
      0: ()
      1: (0)
      2: (0, 0)
      3: (0, 2)
      4: (2)
      5: (2, 2)
    First order partial derivatives =
      Key ==> Term
      1, (0) ==> 1, ()
      2, (0) ==> 2, (0)
      3, (0) ==> 1, (2)
      3, (2) ==> 1, (0)
      4, (2) ==> 1, ()
      5, (2) ==> 2, (2)
    Second order partial derivatives =
      Key ==> Term
      2, (0, 0) ==> 2, ()
      3, (0, 2) ==> 2, ()
      5, (2, 2) ==> 2, ()
   */
  MonomialFeatures features(2, 3, {1});

  EXPECT_TRUE(features.length() == 6);

  VectorX<double> q(3);
  q << 0, 1, 0.5;
  VectorX<double> qdot(3);
  qdot << 0.3, 0, 0;

  VectorX<double> expected_feature(10);
  expected_feature << 1, 0, 0, 0, 0.5, 0.25;
  EXPECT_TRUE((features.Eval(q) - expected_feature).norm() == 0);

  VectorX<double> expected_JV(10);
  expected_JV << 0, 0.3, 0, 0.15, 0, 0;
  EXPECT_TRUE((features.EvalJV(q, qdot) - expected_JV).norm() == 0);

  VectorX<double> expected_JdotV(10);
  expected_JdotV << 0, 0, 0.18, 0, 0, 0;
  EXPECT_TRUE((features.EvalJdotV(q, qdot) - expected_JdotV).norm() == 0);
}

}  // namespace
}  // namespace goldilocks_models
}  // namespace dairlib

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
