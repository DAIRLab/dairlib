#include <gtest/gtest.h>
#include "multibody/tree_container.h"

namespace dairlib {
namespace multibody {
namespace {

class TreeContainerTest : public ::testing::Test {

  protected:
    void SetUp() override {}

};

} // namespace
} // multibody
} // dairlib

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
