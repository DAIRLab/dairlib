#include "systems/framework/output_vector.h"

#include <memory>
#include <utility>

#include <gtest/gtest.h>

namespace dairlib {
namespace systems {
namespace {

using drake::systems::BasicVector;
using std::make_unique;

class OutputVectorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    int nq = 5;
    int nv = 4;
    int nu = 2;
    q_.resize(nq);
    q_ << 2.0, -1.5, 1.0, 3.14, 2.18;
    v_.resize(nv);
    v_ << -.7, 10.6, 0.0, -1.0;
    x_.resize(nq+nv);
    x_ << q_, v_;
    effort_.resize(nu);
    effort_ << -2.5, 3.7;

    vector_ = make_unique<OutputVector<double>>(q_, v_, effort_);
  }

  Eigen::VectorXd x_;
  Eigen::VectorXd q_;
  Eigen::VectorXd v_;
  Eigen::VectorXd effort_;
  std::unique_ptr<OutputVector<double>> vector_;
};

TEST_F(OutputVectorTest, ValueChecks) {
  ASSERT_EQ(q_, vector_->GetPositions());

  ASSERT_EQ(v_, vector_->GetVelocities());

  ASSERT_EQ(x_, vector_->GetState());

  ASSERT_EQ(q_, vector_->GetMutablePositions());

  ASSERT_EQ(v_, vector_->GetMutableVelocities());

  ASSERT_EQ(x_, vector_->GetMutableState());
}

// TEST_F(OutputVectorTest, NonMutableCheck) {
//   auto data = vector_->CopyVectorNoTimestamp();
//   ASSERT_EQ(input_value_, data);
//   data(0) = 1;
//   ASSERT_EQ(input_value_, vector_->get_data());
// }

TEST_F(OutputVectorTest, MutableCheck) {
  auto data = vector_->GetMutableState();
  data(0) = 1;
  ASSERT_EQ(1, vector_->GetState()(0));
  data(0) = x_(0);

  auto data2 = vector_->GetMutablePositions();
  data2(2) = -5;
  ASSERT_EQ(-5, vector_->GetPositions()(2));
  data2(2) = q_(2);

  auto data3 = vector_->GetMutableVelocities();
  data3(2) = -5;
  ASSERT_EQ(-5, vector_->GetVelocities()(2));
  data3(2) = v_(2);
}



}  // namespace
}  // namespace systems
}  // namespace dairlib


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}