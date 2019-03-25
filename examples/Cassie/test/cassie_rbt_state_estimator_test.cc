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
using Eigen::VectorXd;

class CassieRbtStateEstimatorTest : public ::testing::Test {
};

TEST_F(CassieRbtStateEstimatorTest, FixedBaseTest) {
  RigidBodyTree<double> tree;
  drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
    "examples/Cassie/urdf/cassie.urdf",
    drake::multibody::joints::kFixed, &tree);

  CassieRbtStateEstimator estimator(tree);
  VectorXd q_init;
  double left_heel_spring, right_heel_spring;
  solveFourbarLinkage(q_init, left_heel_spring, right_heel_spring)
}

TEST_F(CassieRbtStateEstimatorTest, RPYTest) {
  RigidBodyTree<double> tree;
  drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
    "examples/Cassie/urdf/cassie.urdf",
    drake::multibody::joints::kRollPitchYaw, &tree);

  CassieRbtStateEstimator estimator(tree);
  solveFourbarLinkage(VectorXd q_init,
    double & left_heel_spring,double & right_heel_spring)
}

TEST_F(CassieRbtStateEstimatorTest, QuaternionTest) {
  RigidBodyTree<double> tree;
  drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
    "examples/Cassie/urdf/cassie.urdf",
    drake::multibody::joints::kQuaternion, &tree);

  CassieRbtStateEstimator estimator(tree);
  solveFourbarLinkage(VectorXd q_init,
    double & left_heel_spring,double & right_heel_spring)
}


}  // namespace
}  // namespace systems
}  // namespace dairlib


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}