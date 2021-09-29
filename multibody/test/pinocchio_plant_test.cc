#include <gtest/gtest.h>
#include <memory>
#include <utility>

#include "common/find_resource.h"
#include "multibody/pinocchio_plant.h"

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsing/parser.h"

namespace dairlib {
namespace multibody {
namespace {

using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using Eigen::VectorXd;
using Eigen::MatrixXd;

class PinocchioPlantTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Building floating-base
    std::string urdf =
        FindResourceOrThrow("examples/Cassie/urdf/cassie_v2.urdf");
  
    plant_ = std::make_unique<PinocchioPlant<double>>(0, urdf);

    drake::geometry::SceneGraph<double> scene_graph;
    Parser parser(plant_.get(), &scene_graph);
    parser.AddModelFromFile(urdf);
    plant_->WeldFrames(plant_->world_frame(),
                           plant_->GetFrameByName("pelvis"));
    plant_->Finalize();
  }

  std::unique_ptr<PinocchioPlant<double>> plant_{};
};

// Test mass matrix
TEST_F(PinocchioPlantTest, MassMatrixTest) {
  int nq = plant_->num_positions();
  int nv = plant_->num_velocities();

  VectorXd x = VectorXd::Random(nq + nv);

  auto context = plant_->CreateDefaultContext();
  plant_->SetPositionsAndVelocities(context.get(), x);
  EXPECT_TRUE(plant_->TestMassMatrix(*context, 1e-9));
}


}  // namespace
}  // namespace multibody
}  // namespace dairlib


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
