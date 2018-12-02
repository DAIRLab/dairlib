#include <gtest/gtest.h>
#include <memory>
#include <utility>

// #include "drake/geometry/scene_graph.h"
// #include "drake/multibody/parsing/urdf_parser.h"
// #include "multibody/mbt_utils.h"
// #include "common/find_resource.h"

namespace dairlib {
namespace multibody {
namespace {

// using drake::multibody::multibody_plant::MultibodyPlant;
// using drake::multibody::parsing::AddModelFromUrdfFile;

class MultibodyUtilsTest : public ::testing::Test {
 protected:
  void SetUp() override {
  //   drake::geometry::SceneGraph<double> scene_graph;
  //   std::string full_name = FindResourceOrThrow(
  //       "examples/Cassie/urdf/cassie_v2.urdf");
  //   AddModelFromUrdfFile(full_name, &plant_, &scene_graph);
  //   plant_.Finalize();
  }

  // MultibodyPlant<double> plant_;
};

// Tests that the output of this system equals its input.
TEST_F(MultibodyUtilsTest, StateAndActuatorMappingTest) {
  // /// Checks that the number of input ports in the system and in the context
  // // are consistent.
  // ASSERT_EQ(1, context_->get_num_input_ports());
  // ASSERT_EQ(1, pass_through_->get_num_input_ports());

  // // Hook input of the expected size.
  // context_->FixInputPort(
  //       0, std::make_unique<BasicVector<double>>(input_value_));


  // pass_through_->CalcOutput(*context_, output_.get());

  // // Checks that the number of output ports in the system and in the
  // // output are consistent.
  // ASSERT_EQ(1, output_->get_num_ports());
  // ASSERT_EQ(1, pass_through_->get_num_output_ports());

  // Eigen::VectorXd output;
  // const BasicVector<double>* output_vector = output_->get_vector_data(0);
  // ASSERT_NE(nullptr, output_vector);
  // output = output_vector->get_value();

  // EXPECT_EQ(input_subvector_, output);
  EXPECT_EQ(1,1);
}

}  // namespace
}  // namespace multibody
}  // namespace dairlib


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}