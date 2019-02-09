#include <gtest/gtest.h>
#include <memory>
#include <utility>

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsing/parser.h"
#include "multibody/mbt_utils.h"
#include "common/find_resource.h"

namespace dairlib {
namespace multibody {
namespace {

using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;

class MultibodyUtilsTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Building a floating-base plant
    drake::geometry::SceneGraph<double> scene_graph;
    std::string full_name = FindResourceOrThrow(
        "examples/Cassie/urdf/cassie_v2.urdf");
    Parser parser(&plant_, &scene_graph);
    parser.AddModelFromFile(full_name);
    plant_.Finalize();
  }

  MultibodyPlant<double> plant_;
};

// Test that maps can be constructed without errrors caught by the functions
// themselves
TEST_F(MultibodyUtilsTest, StateAndActuatorMappingTest) {
  auto positions_map = makeNameToPositionsMap(plant_);

  auto velocities_map = makeNameToVelocitiesMap(plant_);

  auto actuators_map = makeNameToActuatorsMap(plant_);
}

}  // namespace
}  // namespace multibody
}  // namespace dairlib


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}