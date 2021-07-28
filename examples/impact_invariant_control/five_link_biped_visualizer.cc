#include <gflags/gflags.h>

#include "common/find_resource.h"
#include "dairlib/lcmt_robot_output.hpp"
#include "multibody/com_pose_system.h"
#include "multibody/multibody_utils.h"
#include "systems/primitives/subvector_pass_through.h"
#include "systems/robot_lcm_systems.h"

#include "drake/geometry/drake_visualizer.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/rendering/multibody_position_to_geometry_pose.h"

namespace dairlib {

DEFINE_string(channel, "RABBIT_STATE", "LCM channel for receiving state. ");
DEFINE_bool(com, true, "Visualize the COM as a sphere");
DEFINE_bool(
    com_ground, true,
    "If com=true, sets whether the COM should be shown on the ground (z=0)"
    " or at the correct height.");

using dairlib::systems::RobotOutputReceiver;
using dairlib::systems::SubvectorPassThrough;
using drake::geometry::SceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::Simulator;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::rendering::MultibodyPositionToGeometryPose;
using drake::geometry::DrakeVisualizer;

using drake::geometry::Sphere;
using drake::math::RigidTransformd;
using drake::multibody::RigidBody;
using drake::multibody::SpatialInertia;
using drake::multibody::UnitInertia;
using drake::systems::rendering::MultibodyPositionToGeometryPose;

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  drake::systems::DiagramBuilder<double> builder;

  // MultibodyPlant<double> plant;
  std::string full_name = FindResourceOrThrow(
      "examples/impact_invariant_control/five_link_biped.urdf");
  MultibodyPlant<double> plant(0.0);
  SceneGraph<double>* scene_graph = builder.AddSystem<SceneGraph>();
  scene_graph->set_name("scene_graph");
  Parser parser(&plant, scene_graph);
  parser.AddModelFromFile(full_name);
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"),
                   drake::math::RigidTransform<double>());
  plant.mutable_gravity_field().set_gravity_vector(-9.81 *
                                                   Eigen::Vector3d::UnitZ());
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
      scene_graph->get_source_pose_port(plant.get_source_id().value()));

  // *******Add COM visualization**********
  auto ball_plant = std::make_unique<MultibodyPlant<double>>(0.0);
  if (FLAGS_com) {
    double radius = .02;
    UnitInertia<double> G_Bcm = UnitInertia<double>::SolidSphere(radius);
    SpatialInertia<double> M_Bcm(1, Eigen::Vector3d::Zero(), G_Bcm);

    const RigidBody<double>& ball = ball_plant->AddRigidBody("Ball", M_Bcm);

    ball_plant->RegisterAsSourceForSceneGraph(scene_graph);
    // Add visual for the COM.
    const Eigen::Vector4d orange(1.0, 0.55, 0.0, 1.0);
    const RigidTransformd X_BS = RigidTransformd::Identity();
    ball_plant->RegisterVisualGeometry(ball, X_BS, Sphere(radius), "visual",
                                       orange);
    ball_plant->Finalize();

    // connect
    auto q_passthrough = builder.AddSystem<SubvectorPassThrough>(
        state_receiver->get_output_port(0).size(), 0, plant.num_positions());
    builder.Connect(state_receiver->get_output_port(0),
                    q_passthrough->get_input_port());
    auto q_to_com_system = builder.AddSystem<multibody::ComPoseSystem>(plant);

    auto ball_to_pose =
        builder.AddSystem<MultibodyPositionToGeometryPose<double>>(*ball_plant);
    builder.Connect(*q_passthrough, *q_to_com_system);
    if (FLAGS_com_ground) {
      builder.Connect(q_to_com_system->get_xy_com_output_port(),
                      ball_to_pose->get_input_port());
    } else {
      builder.Connect(q_to_com_system->get_com_output_port(),
                      ball_to_pose->get_input_port());
    }
    builder.Connect(
        ball_to_pose->get_output_port(),
        scene_graph->get_source_pose_port(ball_plant->get_source_id().value()));
  }

  DrakeVisualizer<double>::AddToBuilder(&builder, *scene_graph);

  auto diagram = builder.Build();

  auto context = diagram->CreateDefaultContext();

  /// Use the simulator to drive at a fixed rate
  /// If set_publish_every_time_step is true, this publishes twice
  /// Set realtime rate. Otherwise, runs as fast as possible
  auto stepper =
      std::make_unique<Simulator<double>>(*diagram, std::move(context));
  stepper->set_publish_every_time_step(false);
  stepper->set_publish_at_initialization(false);
  stepper->set_target_realtime_rate(1.0);
  stepper->Initialize();

  drake::log()->info("visualizer started");

  stepper->AdvanceTo(std::numeric_limits<double>::infinity());

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::do_main(argc, argv); }
