#include <gtest/gtest.h>

#include "geometry/convex_foothold_receiver.h"

namespace dairlib::geometry{
namespace {

 class ConvexFootholdTest : public ::testing::Test {
  protected:
   void SetUp() override {

   }
   std::vector<ConvexFoothold> footholds_;
 };
}

TEST_F(ConvexFootholdTest, AddVertexTest) {

}
}

int main(int argc, char*argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}