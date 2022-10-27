#include <gflags/gflags.h>

#include "dairlib/lcmt_robot_output.hpp"
#include "multibody/com_pose_system.h"
#include "multibody/multibody_utils.h"
#include "multibody/visualization_utils.h"
#include "systems/primitives/subvector_pass_through.h"
#include "systems/robot_lcm_systems.h"

#include "drake/geometry/drake_visualizer.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/rendering/multibody_position_to_geometry_pose.h"
#include <drake/multibody/parsing/parser.h>
#include "systems/system_utils.h"
#include "drake/common/yaml/yaml_io.h"
#include "examples/franka_trajectory_following/c3_parameters.h"

DEFINE_string(channel, "FRANKA_OUTPUT",
              "LCM channel for receiving state. "
              "Use FRANKA_OUTPUT to get state from simulator, and "
              "use FRANKA_ROS_OUTPUT to get state from state estimator");

namespace dairlib {

using dairlib::systems::RobotOutputReceiver;
using dairlib::systems::SubvectorPassThrough;
using drake::geometry::DrakeVisualizer;
using drake::geometry::SceneGraph;
using drake::geometry::Sphere;
using drake::math::RigidTransformd;
using drake::multibody::MultibodyPlant;
using drake::multibody::RigidBody;
using drake::multibody::SpatialInertia;
using drake::multibody::UnitInertia;
using drake::systems::Simulator;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::rendering::MultibodyPositionToGeometryPose;
using std::cout;
using std::endl;

using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::Parser;
using drake::math::RigidTransform;
using drake::systems::DiagramBuilder;

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  C3Parameters param = drake::yaml::LoadYamlFile<C3Parameters>(
    "examples/franka_trajectory_following/parameters.yaml");

  drake::systems::DiagramBuilder<double> builder;

  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  MultibodyPlant<double> plant(0.0);

  Parser parser(&plant, &scene_graph);
  parser.AddModelFromFile("examples/franka_trajectory_following/robot_properties_fingers/urdf/franka_box.urdf");
  parser.AddModelFromFile("examples/franka_trajectory_following/robot_properties_fingers/urdf/sphere.urdf");
  parser.AddModelFromFile("examples/franka_trajectory_following/robot_properties_fingers/urdf/sphere2.urdf");

  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_link0"), X_WI);

  plant.Finalize();

  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>();

  // Create state receiver.
  auto state_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_robot_output>(
          FLAGS_channel, lcm));
  auto state_receiver = builder.AddSystem<RobotOutputReceiver>(plant);
  builder.Connect(*state_sub, *state_receiver);

  auto passthrough = builder.AddSystem<SubvectorPassThrough>(
      state_receiver->get_output_port(0).size(), 0, plant.num_positions());
  builder.Connect(*state_receiver, *passthrough);

  auto to_pose =
      builder.AddSystem<MultibodyPositionToGeometryPose<double>>(plant);
  builder.Connect(*passthrough, *to_pose);
  builder.Connect(
      to_pose->get_output_port(),
      scene_graph.get_source_pose_port(plant.get_source_id().value()));
  DrakeVisualizer<double>::AddToBuilder(&builder, scene_graph);

  // state_receiver->set_publish_period(1.0/30.0);  // framerate

  auto diagram = builder.Build();
  // DrawAndSaveDiagramGraph(*diagram, "examples/franka_trajectory_following/diagram_visualizer");

  auto context = diagram->CreateDefaultContext();

  /// Use the simulator to drive at a fixed rate
  /// If set_publish_every_time_step is true, this publishes twice
  /// Set realtime rate. Otherwise, runs as fast as possible
  auto stepper =
      std::make_unique<Simulator<double>>(*diagram, std::move(context));
  stepper->set_publish_every_time_step(false);
  stepper->set_publish_at_initialization(false);
  stepper->set_target_realtime_rate(param.realtime_rate); // may need to change this to param.real_time_rate?
  stepper->Initialize();

  drake::log()->info("visualizer started");

  stepper->AdvanceTo(std::numeric_limits<double>::infinity());

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::do_main(argc, argv); }
