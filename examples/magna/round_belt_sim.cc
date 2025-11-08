#include <cmath>
#include <iostream>
#include <limits>

#include <drake/common/yaml/yaml_io.h>
#include <drake/geometry/drake_visualizer.h>
#include <drake/geometry/meshcat_visualizer.h>
#include <drake/lcm/drake_lcm.h>
#include <drake/math/rigid_transform.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/lcm/lcm_interface_system.h>
#include <drake/systems/lcm/lcm_publisher_system.h>
#include <gflags/gflags.h>

#include "common/find_resource.h"
#include "parameter_headers/lcm_channel_params.h"
#include "parameter_headers/sim_params.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"

#include "drake/multibody/plant/deformable_model.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"

namespace dairlib {
namespace magna {

static constexpr const char* kFrankaModel =
    "package://drake_models/franka_description/urdf/"
    "panda_arm_hand_with_long_fingers.urdf";

using dairlib::systems::AddActuationRecieverAndStateSenderLcm;
using drake::math::RigidTransform;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::DeformableBody;
using drake::multibody::DeformableModel;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::Parser;
using drake::systems::DiagramBuilder;
using Eigen::VectorXd;

// Below is a hacky function to find the indices of two end points of the round
// belt. Note: This function only works when the initial round belt pose aligns
// with the world frame, and the order of the vertices won't change after the
// simulation starts.
void FindKeyVertices(const drake::multibody::MultibodyPlant<double>& plant,
                     double y_value = 0.0, double y_tolerance = 1e-3) {
  // Get the reference to the deformable body to extract the vertices'
  // positions.
  const DeformableModel<double>& deformable_model = plant.deformable_model();
  const DeformableBody<double>& deform_body =
      deformable_model.GetBodyByName("round_belt");
  VectorXd reference_positions =
      deformable_model.GetReferencePositions(deform_body.body_id());

  int min_idx = -1, max_idx = -1;
  double min_x = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();

  for (int i = 0; i < reference_positions.size() / 3; ++i) {
    double x = reference_positions(3 * i);
    double y = reference_positions(3 * i + 1);
    if (std::abs(y - y_value) < y_tolerance) {
      if (x < min_x) {
        min_x = x;
        min_idx = i;
      }
      if (x > max_x) {
        max_x = x;
        max_idx = i;
      }
    }
  }

  if (min_idx != -1) {
    std::cout << "Index with minimum x (y=0): " << min_idx << std::endl;
    std::cout << reference_positions.segment(3 * min_idx, 3).transpose()
              << std::endl;
  } else {
    std::cout << "No point found with y=0" << std::endl;
  }
  if (max_idx != -1) {
    std::cout << "Index with maximum x (y=0): " << max_idx << std::endl;
    std::cout << reference_positions.segment(3 * max_idx, 3).transpose()
              << std::endl;
  }
}

DEFINE_string(lcm_url, "udpm://239.255.76.67:7667?ttl=0",
              "LCM URL with IP, port, and TTL settings");
DEFINE_double(timestep, 5e-3, "Desired duration of the simulation [s].");
int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  MagnaSimParams sim_params = drake::yaml::LoadYamlFile<MagnaSimParams>(
      "examples/magna/parameters/sim_params.yaml");
  MagnaLcmChannels lcm_channel_params =
      drake::yaml::LoadYamlFile<MagnaLcmChannels>(
          "examples/magna/parameters/lcm_channels_simulation.yaml");
  // Build the simulation plant.
  DiagramBuilder<double> builder;
  auto [plant, scene_graph] =
      AddMultibodyPlantSceneGraph(&builder, FLAGS_timestep);

  Parser parser(&plant, &scene_graph);
  parser.SetAutoRenaming(true);

  // Add the task scene model which includes the table and the franka mount.
  // Note: the task scene model has no collision geometry; only the pulley
  // models include it.
  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  ModelInstanceIndex round_belt_task_scene_index = parser.AddModels(
      dairlib::FindResourceOrThrow("examples/magna/urdf/round_belt_task/"
                                   "round_belt_task_scene.urdf"))[0];
  plant.WeldFrames(plant.world_frame(),
                   plant.GetFrameByName("round_belt_task_scene",
                                        round_belt_task_scene_index),
                   X_WI);
  // Add pulley models
  ModelInstanceIndex task_board_index = parser.AddModels(
      dairlib::FindResourceOrThrow("examples/magna/urdf/round_belt_task/"
                                   "round_belt_task_board.sdf"))[0];
  RigidTransform<double> task_board_pose =
      RigidTransform<double>(drake::math::RollPitchYaw<double>(0, 0, 1.57079),
                             drake::Vector3<double>(0.68585, -0.192, 0.00543));
  plant.WeldFrames(plant.world_frame(),
                   plant.GetFrameByName("board", task_board_index),
                   task_board_pose);

  // Add franka arm model
  ModelInstanceIndex franka_index = parser.AddModelsFromUrl(kFrankaModel)[0];
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_link0"),
                   X_WI);

  // Add round belt model
  ModelInstanceIndex round_belt_index = parser.AddModels(
      dairlib::FindResourceOrThrow("examples/magna/urdf/round_belt_task/"
                                   "round_belt.sdf"))[0];
  plant.Finalize();

  drake::lcm::DrakeLcm drake_lcm(FLAGS_lcm_url);
  auto lcm =
      builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>(&drake_lcm);
  AddActuationRecieverAndStateSenderLcm(
      &builder, plant, lcm, lcm_channel_params.franka_input_channel,
      lcm_channel_params.franka_state_channel, sim_params.franka_publish_rate,
      franka_index, sim_params.publish_efforts, sim_params.actuator_delay);

  /* Add a visualizer that emits LCM messages for visualization. */
  drake::geometry::DrakeVisualizerParams params;
  params.publish_period = 1.0 / sim_params.object_publish_rate;
  params.show_hydroelastic = true;
  auto& drake_visualizer = drake::geometry::DrakeVisualizerd::AddToBuilder(
      &builder, scene_graph, nullptr, params);

  int nq = plant.num_positions();
  int nv = plant.num_velocities();

  auto diagram = builder.Build();
  diagram->set_name("round_belt_sim");
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
}  // namespace dairlib
int main(int argc, char* argv[]) { return dairlib::magna::DoMain(argc, argv); }
