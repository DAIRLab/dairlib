#include <memory>
#include <utility>

#include <gtest/gtest.h>
//#include "examples/Cassie/cassie_utils.h"
#include "multibody/rbt_utils.h"
#include "multibody/contact_toolkit.h"

namespace dairlib {
namespace systems {
namespace {

class ContactToolkitTest : public ::testing::Test {

 protected:
  void SetUp() override {

  }

  RigidBodyTree<double> tree_;
};

// Tests number of input and output ports.
TEST_F(ContactToolkitTest, InitializationTest) {

  ASSERT_EQ(1, 1);
}

}  // namespace
}  // namespace systems
}  // namespace dairlib


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
