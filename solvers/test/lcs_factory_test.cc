#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "multibody/multibody_utils.h"
#include "solvers/lcs_factory.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/parsing/parser.h"

namespace dairlib {
namespace solvers {
namespace {

using drake::CompareMatrices;
using drake::math::RigidTransform;
using drake::multibody::MultibodyPlant;
using drake::systems::DiagramBuilder;
using drake::multibody::Parser;
using Eigen::VectorXd;
using Eigen::MatrixXd;

class LcsFactoryTest : public ::testing::Test {
};

TEST_F(LcsFactoryTest, Test) {
  // Build a MultibodyPlant with a sphere-plane contact
  DiagramBuilder<double> builder;
  auto [plant, scene_graph] =
      drake::multibody::AddMultibodyPlantSceneGraph(&builder, 0.0);
  Parser parser(&plant);
  parser.AddModelFromFile("examples/franka_trajectory_following/robot_properties_fingers/urdf/sphere.urdf");
  parser.AddModelFromFile("examples/franka_trajectory_following/robot_properties_fingers/urdf/franka_box.urdf");
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_link0"),
                   RigidTransform<double>::Identity());

  plant.Finalize();
  auto diagram = builder.Build();

  std::unique_ptr<MultibodyPlant<drake::AutoDiffXd>> plant_ad =
    drake::systems::System<double>::ToAutoDiffXd(plant);

  // State to linearize abou
  VectorXd q = VectorXd::Zero(plant.num_positions());
  VectorXd v = VectorXd::Zero(plant.num_velocities());
  VectorXd u = VectorXd::Zero(plant.num_actuators());

  auto q_map = multibody::makeNameToPositionsMap(plant);
  auto v_map = multibody::makeNameToVelocitiesMap(plant);

  // initialize ball
  q[q_map["base_qw"]] = 1;
  q[q_map["base_qx"]] = 0;
  q[q_map["base_qy"]] = 0;
  q[q_map["base_qz"]] = 0;
  q[q_map["base_x"]] = 0;
  q[q_map["base_y"]] = 0;
  q[q_map["base_z"]] = .0315 - .0031; //surface of the box is at -0.0031 m


  // Create and set context
  auto diagram_context = diagram->CreateDefaultContext();
  auto& context = diagram->GetMutableSubsystemContext(plant, diagram_context.get());
  // auto context = plant.CreateDefaultContext();
  auto context_ad = plant_ad->CreateDefaultContext();

  plant.SetPositions(&context, q);
  plant.SetVelocities(&context, v);
  multibody::SetInputsIfNew<double>(plant, u, &context);

  VectorXd xu(plant.num_positions() + plant.num_velocities() +
      plant.num_actuators());
  xu << q, v, u;
  auto xu_ad = drake::math::InitializeAutoDiff(xu);

  plant_ad->SetPositionsAndVelocities(context_ad.get(),
      xu_ad.head(plant.num_positions() + plant.num_velocities()));
  multibody::SetInputsIfNew<drake::AutoDiffXd>(
      *plant_ad, xu_ad.tail(plant.num_actuators()), context_ad.get());


  drake::geometry::GeometryId sphere_geom =
    plant.GetCollisionGeometriesForBody(plant.GetBodyByName("sphere"))[0];
  drake::geometry::GeometryId ground_geom =
    plant.GetCollisionGeometriesForBody(plant.GetBodyByName("box"))[0];


  std::vector<drake::SortedPair<drake::geometry::GeometryId>> contact_pairs;
  contact_pairs.push_back(drake::SortedPair(sphere_geom, ground_geom));

  double mu = 1;
  double dt = .1;
  int num_friction_directions = 2;
  int N = 5;

  auto linearization = LCSFactory::LinearizePlantToLCS(plant, context,
      *plant_ad, *context_ad, contact_pairs, num_friction_directions, mu, dt,
      N);
  auto lcs = linearization.first;

  // Check the sphere dynamics
  //  1) check only gravity in velocity dynamics w/o contact
  EXPECT_TRUE(drake::CompareMatrices(lcs.A_[0].block(21, 21, 6, 6),
                                     MatrixXd::Identity(6, 6)));
  EXPECT_EQ(lcs.d_[0](26), -9.81*dt);

  // Fix sticking contact between sphere and ball
  std::set<int> active_lambda_inds({1,2,3,4,5});
  std::set<int> inactive_lambda_inds({0});
  auto lcs_fixed = solvers::LCSFactory::FixSomeModes(lcs,
      active_lambda_inds, inactive_lambda_inds);

  // Check sphere dynamics with fixed contact
  // zdot = 0
  double tol = 1e-8;
  EXPECT_TRUE(drake::CompareMatrices(lcs_fixed.A_[0].block(26, 21, 1, 6),
                                     MatrixXd::Zero(1, 6), tol));
  EXPECT_NEAR(lcs_fixed.d_[0](26), 0, tol);

  // Check that initial velocities that are rolling are fixed points w.r.t. A
  VectorXd x_test = VectorXd::Ones(lcs_fixed.A_[0].rows());
  x_test(24) = 3; //xdot
  x_test(22) = 3/.0315; //w_y
  x_test(25) = 5; //ydot
  x_test(21) = -5/.0315; //w_x
  x_test(26) = 0; //zdot
  VectorXd x_next = lcs_fixed.A_[0] * x_test;
  EXPECT_TRUE(drake::CompareMatrices(x_next.tail(6), x_test.tail(6), tol));
}

}
}
}
