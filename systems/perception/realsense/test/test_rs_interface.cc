#include "systems/perception/realsense/single_rs_interface.h"

#include <gtest/gtest.h>


namespace rs2_systems {

GTEST_TEST(RSInterfaceTest, Test) {
  auto interface = SingleRSInterface();
  interface.Start();
  interface.Stop();
}

}