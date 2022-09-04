#include "multibody/multibody_utils.h"
#include "multibody/geom_geom_collider.h"
#include "common/find_resource.h"
#include <iostream>

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/diagram_builder.h"

namespace dairlib {
namespace multibody {
namespace {

using drake::SortedPair;
using drake::geometry::GeometryId;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using Eigen::VectorXd;


void GeomGeomColliderTest() {
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
  const std::vector<GeometryId>& cube_geoms =
      plant.GetCollisionGeometriesForBody(plant.GetBodyByName(
          "cube"));

  // Each body here only has one geometry
  auto geom_A = finger_lower_link_0_geoms[0];
  auto geom_B = finger_lower_link_120_geoms[0];

  GeomGeomCollider collider_A_B(plant, SortedPair(geom_A, geom_B));
  GeomGeomCollider collider_A_cube(plant, SortedPair(geom_A, cube_geoms[0]));

  auto diagram_context = diagram->CreateDefaultContext();
  auto& context = diagram->GetMutableSubsystemContext(plant,
                                                      diagram_context.get());

  VectorXd q = VectorXd::Zero(plant.num_positions());
  auto q_map = MakeNameToPositionsMap(plant);

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

  std::cout << "A-cube, standard basis" << std::endl;
  auto [phi_A_cube, J_A_cube] = collider_A_cube.Eval(context);
  std::cout << J_A_cube << std::endl << std::endl;

  std::cout << "A-B, 4-direction polytope" << std::endl;
  auto [phi_A_B, J_A_B] = collider_A_B.EvalPolytope(context, 4);
  std::cout << J_A_B << std::endl << std::endl;

  std::cout << "A-B, planar" << std::endl;
  auto [phi_A_B_planar, J_A_B_planar] = collider_A_B.EvalPlanar(context,
      Eigen::Vector3d(0, 1, 0));
  std::cout << J_A_B_planar << std::endl << std::endl;
}

}  // namespace
}  // namespace multibody
}  // namespace dairlib


int main(int argc, char **argv) {
  dairlib::multibody::GeomGeomColliderTest();
}
