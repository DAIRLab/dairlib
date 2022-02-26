#include "multibody/multibody_utils.h"
#include "multibody/geom_geom_collider.h"
#include "multibody/kinematic/kinematic_evaluator_set.h"
#include "common/find_resource.h"

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/diagram_builder.h"


namespace dairlib {
namespace solvers {
namespace {

using drake::AutoDiffXd;
using drake::geometry::GeometryId;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using Eigen::VectorXd;


void LcsTest() {
  drake::systems::DiagramBuilder<double> builder;

  auto [plant, scene_graph] =
      drake::multibody::AddMultibodyPlantSceneGraph(&builder, 0.0);

  Parser parser(&plant, &scene_graph);

  parser.package_map().Add("robot_properties_fingers",
                           "examples/trifinger/robot_properties_fingers");
  parser.AddModelFromFile(FindResourceOrThrow("examples/trifinger/"
      "robot_properties_fingers/urdf/trifinger_minimal_collision.urdf"));
  parser.AddModelFromFile(FindResourceOrThrow(
      "examples/trifinger/robot_properties_fingers/cube/cube_v2.urdf"));

  auto X_WI = drake::math::RigidTransform<double>::Identity();
  plant.WeldFrames(plant.world_frame(),
                    plant.GetFrameByName("base_link"), X_WI);
  plant.Finalize();

  auto diagram = builder.Build();


  const std::vector<GeometryId>& finger_lower_link_0_geoms =
      plant.GetCollisionGeometriesForBody(plant.GetBodyByName(
          "finger_lower_link_0"));
  const std::vector<GeometryId>& finger_lower_link_120_geoms =
      plant.GetCollisionGeometriesForBody(plant.GetBodyByName(
          "finger_lower_link_120"));
  const std::vector<GeometryId>& finger_lower_link_240_geoms =
      plant.GetCollisionGeometriesForBody(plant.GetBodyByName(
          "finger_lower_link_240"));
  const std::vector<GeometryId>& cube_geoms =
      plant.GetCollisionGeometriesForBody(plant.GetBodyByName(
          "cube"));

  // Each body here only has one geometry
  auto geom_A = finger_lower_link_0_geoms[0];
  auto geom_B = finger_lower_link_120_geoms[0];
  auto geom_C = finger_lower_link_240_geoms[0];

  multibody::GeomGeomCollider collider_A_cube(plant, geom_A, cube_geoms[0], 2);
  multibody::GeomGeomCollider collider_B_cube(plant, geom_B, cube_geoms[0], 2);
  multibody::GeomGeomCollider collider_C_cube(plant, geom_C, cube_geoms[0], 2);

  auto diagram_context = diagram->CreateDefaultContext();
  auto& context = diagram->GetMutableSubsystemContext(plant,
                                                      diagram_context.get());

  VectorXd q = VectorXd::Zero(plant.num_positions());
  auto q_map = multibody::makeNameToPositionsMap(plant);

  q(q_map.at("finger_base_to_upper_joint_0")) = 0;
  q(q_map.at("finger_upper_to_middle_joint_0")) = -1;
  q(q_map.at("finger_middle_to_lower_joint_0")) = -1.5;
  q(q_map.at("finger_base_to_upper_joint_0")) = 0;
  q(q_map.at("finger_upper_to_middle_joint_120")) = -1;
  q(q_map.at("finger_middle_to_lower_joint_120")) = -1.5;
  q(q_map.at("finger_base_to_upper_joint_240")) = 0;
  q(q_map.at("finger_upper_to_middle_joint_240")) = -1;
  q(q_map.at("finger_middle_to_lower_joint_240")) = -1.5;
  q(q_map.at("base_qw")) = 1;
  q(q_map.at("base_qx")) = 0;
  q(q_map.at("base_qz")) = 0;
  q(q_map.at("base_x")) = 0;
  q(q_map.at("base_y")) = 0;
  q(q_map.at("base_z")) = .05;

  plant.SetPositions(&context, q);

  VectorXd v = VectorXd::Zero(plant.num_velocities());
  auto v_map = multibody::makeNameToVelocitiesMap(plant);

  VectorXd u = VectorXd::Zero(plant.num_actuators());

  plant.SetVelocities(&context, v);

  Eigen::MatrixXd J_A_cube(9, plant.num_positions());
  collider_A_cube.Eval(context, &J_A_cube);
  std::cout << J_A_cube << std::endl << std::endl;

  // Build the LCS
  // First, we'll use an AutoDiff version of the plant for non-contact terms

  auto plant_ad = drake::systems::System<double>::ToAutoDiffXd(plant);
  multibody::KinematicEvaluatorSet<AutoDiffXd> evaluator(*plant_ad);

  // stacked [x;u] as autodiff
  VectorXd xu(q.size() + v.size() + u.size());
  xu << q, v, u;
  auto xu_ad = drake::math::InitializeAutoDiff(xu);
}

}  // namespace
}  // namespace solvers
}  // namespace dairlib


int main(int argc, char **argv) {
  dairlib::solvers::LcsTest();
}
