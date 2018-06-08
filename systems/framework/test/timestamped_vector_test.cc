#include "systems/framework/timestamped_vector.h"

#include <memory>
#include <utility>

#include <gtest/gtest.h>

namespace dairlib {
namespace systems {
namespace {

using drake::systems::BasicVector;
using std::make_unique;

class TimestampedVectorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const int size = 5;
    time_ = .1;
    input_value_.resize(size);
    input_value_ << 2.0, -1.5, 1.0, 3.14, 2.18;
    vector_ = make_unique<TimestampedVector<double>>(input_value_);
    vector_->set_timestamp(time_);
  }

  double time_;
  Eigen::VectorXd input_value_;
  std::unique_ptr<TimestampedVector<double>> vector_;
};

TEST_F(TimestampedVectorTest, ValueChecks) {

  ASSERT_EQ(time_, vector_->get_timestamp());

  ASSERT_EQ(input_value_, vector_->get_data());

  ASSERT_EQ(input_value_, vector_->get_mutable_data());
}

TEST_F(TimestampedVectorTest, NonMutableCheck) {
  auto data = vector_->CopyVectorNoTimestamp();
  ASSERT_EQ(input_value_, data);
  data(0) = 1;
  ASSERT_EQ(input_value_, vector_->get_data());
}

TEST_F(TimestampedVectorTest, MutableCheck) {

  auto data = vector_->get_mutable_data();
  data(0) = 1;
  ASSERT_EQ(1, vector_->GetAtIndex(0));

  data(0) = input_value_(0);
}

TEST_F(TimestampedVectorTest, SetCheck) {
  auto vector2 = make_unique<TimestampedVector<double>>(input_value_.size());
  vector2->SetDataVector(input_value_);

  ASSERT_EQ(vector_->get_data(), vector2->get_data());
}

}  // namespace
}  // namespace systems
}  // namespace dairlib


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}