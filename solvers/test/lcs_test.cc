#include "common/find_resource.h"
#include "multibody/multibody_utils.h"
#include "multibody/geom_geom_collider.h"
#include "multibody/kinematic/kinematic_evaluator_set.h"
#include "solvers/lcs_factory.h"

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/diagram_builder.h"


namespace dairlib {
namespace solvers {
namespace {

using drake::AutoDiffXd;
using drake::SortedPair;
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

  std::vector<SortedPair<GeometryId>> contact_geoms;
  contact_geoms.push_back(SortedPair(finger_lower_link_0_geoms[0],
                                     cube_geoms[0]));
  contact_geoms.push_back(SortedPair(finger_lower_link_120_geoms[0],
                                     cube_geoms[0]));
  contact_geoms.push_back(SortedPair(finger_lower_link_240_geoms[0],
                                     cube_geoms[0]));

  auto diagram_context = diagram->CreateDefaultContext();
  auto& context = diagram->GetMutableSubsystemContext(plant,
                                                      diagram_context.get());

  ///
  /// Set state and input for linearization
  ///
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
  multibody::SetInputsIfNew<double>(plant, u, &context);

  // Build the LCS
  // First, we'll use an AutoDiff version of the plant for non-contact terms

  auto plant_ad = drake::systems::System<double>::ToAutoDiffXd(plant);

  // stacked [x;u] as autodiff
  // Now, all gradients will be w.r.t. [x;u]
  VectorXd xu(q.size() + v.size() + u.size());
  xu << q, v, u;
  auto xu_ad = drake::math::InitializeAutoDiff(xu);

  auto context_ad = plant_ad->CreateDefaultContext();
  plant_ad->SetPositionsAndVelocities(context_ad.get(),
      xu_ad.head(plant.num_positions() + plant.num_velocities()));
  multibody::SetInputsIfNew<AutoDiffXd>(*plant_ad,
                                        xu_ad.tail(plant.num_actuators()),
                                        context_ad.get());

  int num_friction_directions = 2;
  double mu = .8;
  LCSFactory::LinearizePlantToLCS(plant, context, *plant_ad, *context_ad,
          contact_geoms, num_friction_directions, mu);

}

}  // namespace
}  // namespace solvers
}  // namespace dairlib


int main(int argc, char **argv) {
  dairlib::solvers::LcsTest();
}
