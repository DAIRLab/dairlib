#include <iostream>
#include <memory>
#include <utility>

#include <gflags/gflags.h>

#include "common/find_resource.h"
#include "deformable_common.h"
#include "parallel_gripper_controller.h"

#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/fem/deformable_body_config.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/multibody/plant/deformable_model.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

DEFINE_double(simulation_time, 12.0, "Desired duration of the simulation [s].");
DEFINE_double(realtime_rate, 1.0, "Desired real time rate.");
DEFINE_double(time_step, 5e-3,
              "Discrete time step for the system [s]. Must be positive.");
DEFINE_string(contact_approximation, "lagged",
              "Type of convex contact approximation. See "
              "multibody::DiscreteContactApproximation for details. Options "
              "are: 'sap', 'lagged', and 'similar'.");

using dairlib::ParallelGripperController;
using drake::geometry::AddContactMaterial;
using drake::geometry::Capsule;
using drake::geometry::IllustrationProperties;
using drake::geometry::ProximityProperties;
using drake::geometry::Sphere;
using drake::math::RigidTransformd;
using drake::multibody::AddMultibodyPlant;
using drake::multibody::CoulombFriction;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::MultibodyPlantConfig;
using drake::multibody::Parser;
using drake::multibody::PrismaticJoint;
using drake::multibody::RigidBody;
using drake::multibody::SpatialInertia;
using drake::systems::Context;
using Eigen::Vector3d;
using Eigen::Vector4d;

namespace dairlib {

/* Adds a parallel gripper to the given MultibodyPlant. Returns the
 ModelInstanceIndex of the gripper model. */
ModelInstanceIndex AddParallelGripper(MultibodyPlant<double>* plant) {
  // TODO(xuchenhan-tri): Consider using a schunk gripper from the manipulation
  // station instead.
  Parser parser(plant);
  ModelInstanceIndex model_instance =
      parser.AddModels(dairlib::FindResourceOrThrow(
          "examples/belt_assembly/urdf/simple_gripper.sdf"))[0];
  /* Get joints so that we can set initial conditions. */
  PrismaticJoint<double>& left_slider =
      plant->GetMutableJointByName<PrismaticJoint>("left_slider");
  PrismaticJoint<double>& right_slider =
      plant->GetMutableJointByName<PrismaticJoint>("right_slider");
  /* Initialize the gripper in an "open" position. */
  const double kInitialWidth = 0.085;
  left_slider.set_default_translation(-kInitialWidth / 2.0);
  right_slider.set_default_translation(kInitialWidth / 2.0);

  return model_instance;
}

int do_main() {
  drake::systems::DiagramBuilder<double> builder;

  MultibodyPlantConfig plant_config;
  plant_config.time_step = FLAGS_time_step;
  plant_config.discrete_contact_approximation = FLAGS_contact_approximation;

  auto [plant, scene_graph] = AddMultibodyPlant(plant_config, &builder);
  dairlib::RegisterRigidGround(&plant);

  /* Minimum required proximity properties for rigid bodies to interact with
   deformable bodies.
   1. A valid Coulomb friction coefficient, and
   2. A resolution hint. (Rigid bodies need to be tessellated so that collision
   queries can be performed against deformable geometries.) The value dictates
   how fine the mesh used to represent the rigid collision geometry is. */
  ProximityProperties rigid_proximity_props;
  /* Set the friction coefficient close to that of rubber against rubber. */
  const CoulombFriction<double> surface_friction(1.15, 1.15);
  AddContactMaterial({}, {}, surface_friction, &rigid_proximity_props);
  rigid_proximity_props.AddProperty(drake::geometry::internal::kHydroGroup,
                                    drake::geometry::internal::kRezHint, 0.01);

  /* Add a parallel gripper */
  ModelInstanceIndex gripper_instance = AddParallelGripper(&plant);

  /* Set up a deformable torus. */
  Parser parser(&plant);
  parser.AddModels(dairlib::FindResourceOrThrow(
      "examples/belt_assembly/urdf/round_belt.sdf"));

  parser.AddModels(dairlib::FindResourceOrThrow(
      "examples/belt_assembly/urdf/large_pulley.sdf"));
  drake::math::RigidTransform<double> X_WI =
      drake::math::RigidTransform<double>(
          drake::math::RollPitchYaw<double>(0, 1.57, 0),
          drake::Vector3<double>(0, 0, 0.05));
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("large_pulley"),
                   X_WI);

  /* All rigid and deformable models have been added. Finalize the plant. */
  plant.Finalize();

  /* Add a visualizer that emits LCM messages for visualization. */
  drake::geometry::DrakeVisualizerParams params;
  params.show_hydroelastic = true;
  auto& drake_visualizer = drake::geometry::DrakeVisualizerd::AddToBuilder(
      &builder, scene_graph, nullptr, params);

  /* Set the width between the fingers for open and closed states as well as
   the height to which the gripper lifts the deformable torus. */
  const double kOpenWidth = 0.028;
  const double kClosedWidth = -0.01;
  const double kLiftedHeight = 0.1;
  const auto& control = *builder.AddSystem<ParallelGripperController>(
      kOpenWidth, kClosedWidth, kLiftedHeight);
  builder.Connect(control.get_output_port(),
                  plant.get_desired_state_input_port(gripper_instance));

  // Add contact results visualizer
  auto contact_results_visualizer =
      drake::multibody::ConnectContactResultsToDrakeVisualizer(
          &builder, plant, scene_graph, nullptr);

  auto diagram = builder.Build();
  std::unique_ptr<Context<double>> diagram_context =
      diagram->CreateDefaultContext();

  /* Build the simulator and run! */
  drake::systems::Simulator<double> simulator(*diagram,
                                              std::move(diagram_context));
  simulator.Initialize();
  simulator.set_target_realtime_rate(FLAGS_realtime_rate);
  simulator.AdvanceTo(FLAGS_simulation_time);
  std::cout << "Real-time rate: " << simulator.get_actual_realtime_rate()
            << std::endl;
  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "This is a demo used to showcase deformable body simulations in Drake. "
      "A parallel (or suction) gripper grasps a deformable torus on the "
      "ground, lifts it up, and then drops it back on the ground. "
      "Launch meldis before running this example. "
      "Refer to README for instructions on meldis as well as optional flags.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return dairlib::do_main();
}
