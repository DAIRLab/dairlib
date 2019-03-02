#include "systems/sensors/sim_cassie_sensor_aggregator.h"
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

class SimCassieSensorAggregatorTest : public ::testing::Test {
};

TEST_F(SimCassieSensorAggregatorTest, FixedBaseTest) {
  RigidBodyTree<double> tree;
  drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
    "examples/Cassie/urdf/cassie.urdf",
    drake::multibody::joints::kFixed, &tree);

  SimCassieSensorAggregator aggregator(tree);
}

TEST_F(SimCassieSensorAggregatorTest, RPYTest) {
  RigidBodyTree<double> tree;
  drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
    "examples/Cassie/urdf/cassie.urdf",
    drake::multibody::joints::kRollPitchYaw, &tree);

  SimCassieSensorAggregator aggregator(tree);
}

TEST_F(SimCassieSensorAggregatorTest, QuaternionTest) {
  RigidBodyTree<double> tree;
  drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
    "examples/Cassie/urdf/cassie.urdf",
    drake::multibody::joints::kQuaternion, &tree);

  SimCassieSensorAggregator aggregator(tree);
}


}  // namespace
}  // namespace systems
}  // namespace dairlib


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}