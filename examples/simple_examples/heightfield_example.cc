#include <gflags/gflags.h>

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/scene_graph.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"

namespace dairlib {

DEFINE_double(realtime_rate, 1.0, "target realtime rate for simulator");

using drake::systems::DiagramBuilder;
using drake::multibody::MultibodyPlant;
using drake::multibody::SpatialInertia;
using drake::multibody::UnitInertia;
using drake::math::RigidTransform;
using drake::geometry::SceneGraph;
using drake::geometry::Sphere;
using drake::geometry::HeightField;
using drake::geometry::DrakeVisualizer;
using drake::multibody::CoulombFriction;

using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using Eigen::MatrixXd;

int do_main() {
  // Plant/System initialization
  DiagramBuilder<double> builder;
  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  const double time_step = 0.001;
  MultibodyPlant<double>& plant = *builder.AddSystem<MultibodyPlant>(time_step);
  plant.AddRigidBody("sphere", SpatialInertia<double>(
      1.0, Vector3d::Zero(), UnitInertia<double>(1.0, 1.0, 1.0)));

  plant.RegisterCollisionGeometry(
      plant.GetBodyByName("sphere"), RigidTransform<double>::Identity(),
      Sphere(0.1), "sphere_collide", CoulombFriction<double>(0.8, 0.8));
  plant.RegisterVisualGeometry(
      plant.GetBodyByName("sphere"), RigidTransform<double>::Identity(),
      Sphere(0.1), "sphere_visual", Vector4d(1.0, 0.0, 0.0, 1.0));

  plant.RegisterCollisionGeometry(
      plant.world_body(), RigidTransform<double>::Identity(),
      HeightField(MatrixXd::Zero(4, 4), 10.0, 10.0, 10.0),
      "hfield_collide", CoulombFriction<double>(0.8, 0.8));
  plant.RegisterVisualGeometry(
      plant.world_body(), RigidTransform<double>::Identity(),
      HeightField(MatrixXd::Zero(4, 4), 10.0, 10.0, 10.0),
      "hfield_visual", Vector4d(1.0, 1.0, 1.0, 1.0));
  plant.Finalize();

  DrakeVisualizer<double>::AddToBuilder(&builder, scene_graph);

  auto diagram = builder.Build();
  auto diagram_context = diagram->CreateDefaultContext();
  auto& plant_context = diagram->GetMutableSubsystemContext(
      plant, diagram_context.get());


  VectorXd q_init;
  q_init << 1, 0, 0, 0, 0, 0, 1;
  plant.SetPositions(&plant_context,q_init);
  drake::systems::Simulator<double>
      simulator(*diagram, std::move(diagram_context));
  simulator.set_target_realtime_rate(1.0);
  simulator.Initialize();
  simulator.AdvanceTo(4.0);

}
}

int main(int argc, char* argv[]) {
  return dairlib::do_main();
}