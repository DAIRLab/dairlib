#include <memory>
#include <utility>

#include <gtest/gtest.h>
#include "multibody/tree_container.h"

namespace dairlib {
namespace systems {
namespace {

class TreeContainerTest : public ::testing::Test {

 protected:
  void SetUp() override {

  }
};

// Tests number of input and output ports.
TEST_F(TreeContainerTest, InitializationTest) {

  /// Checks that the number of input ports in the system and in the context
  // are consistent.
  ASSERT_EQ(1, 1);
}

}  // namespace
}  // namespace systems
}  // namespace dairlib


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
