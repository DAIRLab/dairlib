#include <gtest/gtest.h>
#include <memory>
#include <utility>

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsing/parser.h"
#include "multibody/multibody_utils.h"
#include "common/find_resource.h"

namespace dairlib {
namespace multibody {
namespace {

using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using Eigen::VectorXd;

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

TEST_F(MultibodyUtilsTest, ContextTest) {
  VectorXd x = VectorXd::Zero(plant_.num_positions() + plant_.num_velocities());
  x(5) = 1;
  x(10) = -1.5;

  VectorXd u = VectorXd::Zero(plant_.num_actuators());
  u(0) = 2;
  u(3) = -3.1;

  auto context = createContext(plant_, x, u);

  VectorXd x_context = plant_.GetPositionsAndVelocities(*context);
  VectorXd u_context = getInput(plant_, *context);

  EXPECT_EQ(x, x_context);
  EXPECT_EQ(u, u_context);
}

}  // namespace
}  // namespace multibody
}  // namespace dairlib


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
