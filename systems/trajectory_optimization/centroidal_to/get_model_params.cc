#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/context.h>
#include <drake/multibody/math/spatial_algebra.h>
#include <drake/multibody/parsing/parser.h>
#include "common/find_resource.h"

using drake::multibody::MultibodyPlant;
using drake::multibody::RotationalInertia;
using drake::systems::Context;
using drake::systems::DiagramBuilder;
using drake::geometry::SceneGraph;
using drake::multibody::SpatialInertia;
using drake::multibody::Parser;


RotationalInertia<double> CalcLinkInertiaAboutPlantCom(
    const MultibodyPlant<double>& plant,
    const Context<double>& context,
    const std::string link_name) {
  // Get plant center of mass
  Eigen::Vector3d CoM = plant.CalcCenterOfMassPositionInWorld(context);
  // Find inertia of link in link's own frame
  SpatialInertia<double> I = plant.GetBodyByName(link_name).CalcSpatialInertiaInBodyFrame(context);
  // Rotate inertia to world frame
  I.ReExpressInPlace(
      plant.CalcRelativeRotationMatrix(context,
                                       plant.GetBodyByName(link_name).body_frame(),
                                       plant.world_frame()));

  // Shift inertia to find about CoM
  I.ShiftInPlace(CoM - plant.GetBodyByName(link_name).EvalPoseInWorld(context).translation());
  std::cout << I.CalcRotationalInertia() << std::endl;
  return I.CalcRotationalInertia();
}


int main(int argc, char** argv) {
  std::string urdf = dairlib::FindResourceOrThrow(
      "examples/PlanarWalker/PlanarWalkerWithTorso.urdf");


  DiagramBuilder<double> builder;
  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");


  MultibodyPlant<double>& plant = *builder.AddSystem<MultibodyPlant>(0.0);
  Parser parser(&plant);
  parser.AddModelFromFile(urdf);
  plant.WeldFrames(
      plant.world_frame(), plant.GetFrameByName("base"),
      drake::math::RigidTransform<double>());

  plant.Finalize();

  auto context = plant.CreateDefaultContext();
  Eigen::VectorXd state = Eigen::VectorXd::Zero(plant.num_positions() +
                                                plant.num_velocities());

  plant.SetPositionsAndVelocities(context.get(), state);

  std::vector<std::string> links = {"torso_mass", "left_upper_leg_mass",
                                    "left_lower_leg_mass",
                                    "right_upper_leg_mass",
                                    "right_lower_leg_mass"};

  RotationalInertia I = CalcLinkInertiaAboutPlantCom(plant, *context, links[0]);
  double mass = plant.GetBodyByName(links[0]).get_mass(*context);

  for (int i = 1; i < links.size(); i ++) {
    mass += plant.GetBodyByName(links[i]).get_mass(*context);
    I += CalcLinkInertiaAboutPlantCom(plant, *context, links[i]);
  }

  std::cout << I << std::endl;
  std::cout << mass <<std::endl;

  return 0;
}
