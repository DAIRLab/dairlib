#include <cmath>
#include <iostream>
#include <limits>

#include <drake/common/yaml/yaml_io.h>
#include <drake/geometry/drake_visualizer.h>
#include <drake/geometry/meshcat_visualizer.h>
#include <drake/lcm/drake_lcm.h>
#include <drake/math/rigid_transform.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/tree/revolute_joint.h>
#include <drake/multibody/tree/revolute_spring.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/lcm/lcm_interface_system.h>
#include <drake/systems/lcm/lcm_publisher_system.h>
#include <drake/systems/lcm/lcm_subscriber_system.h>
#include <gflags/gflags.h>

#include "common/find_resource.h"
#include "parameter_headers/lcm_channel_params.h"
#include "parameter_headers/sim_params.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"

#include "drake/multibody/plant/deformable_model.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/common/text_logging.h"

namespace dairlib {
namespace examples {
namespace magna {

static constexpr const char* kFrankaModel =
    "package://drake_models/franka_description/urdf/"
    "panda_arm.urdf";
static constexpr const char* kFrankaHand =
    "package://drake_models/franka_description/urdf/"
    "panda_hand_with_long_fingers.urdf";

using dairlib::systems::AddActuationRecieverAndStateSenderLcm;
using drake::math::RigidTransform;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::DeformableBody;
using drake::multibody::DeformableModel;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::Parser;
using drake::multibody::RevoluteJoint;
using drake::multibody::RevoluteSpring;
using drake::systems::DiagramBuilder;
using Eigen::VectorXd;

DEFINE_string(lcm_url, "udpm://239.255.76.67:7667?ttl=0",
              "LCM URL with IP, port, and TTL settings");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  MagnaSimParams sim_params = drake::yaml::LoadYamlFile<MagnaSimParams>(
      "examples/magna/parameters/sim_params.yaml");
  MagnaLcmChannels lcm_channel_params =
      drake::yaml::LoadYamlFile<MagnaLcmChannels>(
          "examples/magna/parameters/lcm_channels.yaml");
  // Build the simulation plant.
  DiagramBuilder<double> builder;
  auto [plant, scene_graph] =
      AddMultibodyPlantSceneGraph(&builder, sim_params.dt);

  Parser parser(&plant, &scene_graph);
  parser.SetAutoRenaming(true);

  // Add the task scene model which includes the table and the franka mount.
  // Note: the task scene model has no collision geometry; only the pulley
  // models include it.
  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  ModelInstanceIndex scene_index = parser.AddModels(
      dairlib::FindResourceOrThrow("examples/magna/urdf/common/scene.urdf"))[0];
  plant.WeldFrames(plant.world_frame(),
                   plant.GetFrameByName("scene", scene_index), X_WI);
  // Add pulley models
  ModelInstanceIndex task_board_index = parser.AddModels(
      dairlib::FindResourceOrThrow("examples/magna/urdf/timing_belt_task/"
                                   "timing_belt_task_board.sdf"))[0];
  RigidTransform<double> task_board_pose =
      RigidTransform<double>(drake::math::RollPitchYaw<double>(0, 0, 1.57079),
                             drake::Vector3<double>(0.67826, -0.192, 0.00543));
  plant.WeldFrames(plant.world_frame(),
                   plant.GetFrameByName("board", task_board_index),
                   task_board_pose);

  // Add franka arm model
  ModelInstanceIndex franka_index = parser.AddModelsFromUrl(kFrankaModel)[0];
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_link0"),
                   X_WI);
  // Add franka hand model and attach it to the franka arm
  RigidTransform<double> franka_hand_pose_wrt_panda_link8 =
      RigidTransform<double>(
          drake::math::RollPitchYaw<double>(0, 0, -0.785398163397),
          drake::Vector3<double>(0.0, 0.0, 0.0));
  ModelInstanceIndex franka_hand_index =
      parser.AddModelsFromUrl(kFrankaHand)[0];
  plant.WeldFrames(plant.GetFrameByName("panda_link8"),
                   plant.GetFrameByName("panda_hand", franka_hand_index),
                   franka_hand_pose_wrt_panda_link8);

  // Add timing belt model
  ModelInstanceIndex timing_belt_index = parser.AddModels(
      dairlib::FindResourceOrThrow("examples/magna/urdf/timing_belt_task/"
                                   "timing_belt.urdf"))[0];

  // Add weld constraint to create the closed kinematic chain
  plant.AddWeldConstraint(
      plant.GetBodyByName(sim_params.timing_belt_start_body_name), X_WI,
      plant.GetBodyByName(sim_params.timing_belt_end_body_name), X_WI);
  int num_timing_belt_elements = 67;
  for (int i = 0; i < num_timing_belt_elements - 1; i++) {
    std::string roll_joint_name = "joint_" + std::to_string(i) + "_roll";
    std::string yaw_joint_name = "joint_" + std::to_string(i) + "_yaw";
    plant.AddForceElement<RevoluteSpring>(
        plant.GetJointByName<RevoluteJoint>(roll_joint_name, timing_belt_index),
        0.0, sim_params.twisting_stiffness);
    plant.AddForceElement<RevoluteSpring>(
        plant.GetJointByName<RevoluteJoint>(yaw_joint_name, timing_belt_index),
        0.0, sim_params.bending_stiffness);
  }
  plant.Finalize();

  drake::lcm::DrakeLcm drake_lcm(FLAGS_lcm_url);
  auto lcm =
      builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>(&drake_lcm);
  AddActuationRecieverAndStateSenderLcm(
      &builder, plant, lcm, lcm_channel_params.franka_input_channel,
      lcm_channel_params.franka_state_channel, sim_params.franka_publish_rate,
      franka_index, sim_params.publish_efforts, sim_params.actuator_delay);

  AddActuationRecieverAndStateSenderLcm(
      &builder, plant, lcm, lcm_channel_params.franka_hand_input_channel,
      lcm_channel_params.franka_hand_state_channel,
      sim_params.franka_publish_rate, franka_hand_index,
      sim_params.publish_efforts, sim_params.actuator_delay);

  auto timing_belt_state_sender = builder.AddSystem<systems::ObjectStateSender>(
      plant, false, timing_belt_index);
  auto timing_belt_state_pub = builder.AddSystem(
      drake::systems::lcm::LcmPublisherSystem::Make<dairlib::lcmt_object_state>(
          lcm_channel_params.timing_belt_state_channel, lcm,
          1.0 / sim_params.franka_publish_rate));

  builder.Connect(plant.get_state_output_port(timing_belt_index),
                  timing_belt_state_sender->get_input_port_state());
  builder.Connect(timing_belt_state_sender->get_output_port(),
                  timing_belt_state_pub->get_input_port());

  int nq = plant.num_positions();
  int nv = plant.num_velocities();

  auto diagram = builder.Build();
  diagram->set_name("timing_belt_sim");
  dairlib::DrawAndSaveDiagramGraph(*diagram);

  // Create and configure the simulator
  drake::systems::Simulator<double> simulator(*diagram);

  simulator.set_publish_every_time_step(false);
  simulator.set_publish_at_initialization(false);
  simulator.set_target_realtime_rate(sim_params.realtime_rate);

  auto& plant_context = diagram->GetMutableSubsystemContext(
      plant, &simulator.get_mutable_context());

  // Set the initial state of the plant
  VectorXd q = VectorXd::Zero(nq);
  q.head(plant.num_positions(franka_index)) = sim_params.q_init_franka;
  q.segment(plant.num_positions(franka_index), 2) =
      sim_params.q_init_franka_hand;

  q.segment(plant.num_positions(franka_index) + 2, 7) =
      sim_params.q_init_base_timing_belt;

  plant.SetPositions(&plant_context, q);
  VectorXd v = VectorXd::Zero(nv);
  plant.SetVelocities(&plant_context, v);

  // Initialize and run the simulation indefinitely
  simulator.Initialize();
  drake::log()->info("Simulation started");
  simulator.AdvanceTo(std::numeric_limits<double>::infinity());
  return 0;
}
}  // namespace magna
}  // namespace examples
}  // namespace dairlib
int main(int argc, char* argv[]) {
  return dairlib::examples::magna::DoMain(argc, argv);
}
