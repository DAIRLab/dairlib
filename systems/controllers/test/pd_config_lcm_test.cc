#include "systems/controllers/pd_config_lcm.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"

#include <memory>
#include <utility>

#include <gtest/gtest.h>

namespace dairlib {
namespace systems {
namespace {

using drake::systems::BasicVector;
using std::make_unique;

class PDConfigLCMTest : public ::testing::Test {
};

TEST_F(PDConfigLCMTest, FixedBaseTest) {
  RigidBodyTree<double> tree;
  drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
    "examples/Cassie/urdf/cassie.urdf",
    drake::multibody::joints::kFixed, &tree);

  PDConfigReceiver config(tree);
}

TEST_F(PDConfigLCMTest, RPYTest) {
  RigidBodyTree<double> tree;
  drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
    "examples/Cassie/urdf/cassie.urdf",
    drake::multibody::joints::kRollPitchYaw, &tree);

  PDConfigReceiver config(tree);
}

TEST_F(PDConfigLCMTest, QuaternionTest) {
  RigidBodyTree<double> tree;
  drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
    "examples/Cassie/urdf/cassie.urdf",
    drake::multibody::joints::kQuaternion, &tree);

  PDConfigReceiver config(tree);
}


}  // namespace
}  // namespace systems
}  // namespace dairlib


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}