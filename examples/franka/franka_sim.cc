#include <math.h>

#include <vector>

#include <drake/common/find_resource.h>
#include <drake/common/yaml/yaml_io.h>
#include <drake/geometry/drake_visualizer.h>
#include <drake/geometry/meshcat_visualizer.h>
#include <drake/geometry/meshcat_visualizer_params.h>
#include <drake/lcm/drake_lcm.h>
#include <drake/math/rigid_transform.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/lcm/lcm_interface_system.h>
#include <drake/systems/lcm/lcm_publisher_system.h>
#include <drake/systems/lcm/lcm_subscriber_system.h>
#include <drake/systems/primitives/multiplexer.h>
#include <drake/systems/primitives/vector_log_sink.h>
#include <drake/visualization/visualization_config_functions.h>
#include <gflags/gflags.h>

#include "common/eigen_utils.h"
#include "common/find_resource.h"
#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "examples/franka/parameters/franka_lcm_channels.h"
#include "examples/franka/parameters/franka_sim_params.h"
#include "multibody/multibody_utils.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"

namespace dairlib {

using dairlib::systems::SubvectorPassThrough;
using drake::geometry::GeometrySet;
using drake::geometry::SceneGraph;
using drake::math::RigidTransform;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::Context;
using drake::systems::DiagramBuilder;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::trajectories::PiecewisePolynomial;
using multibody::MakeNameToPositionsMap;
using multibody::MakeNameToVelocitiesMap;
using systems::AddActuationRecieverAndStateSenderLcm;

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

DEFINE_string(lcm_channels,
              "examples/franka/parameters/lcm_channels_simulation.yaml",
              "Filepath containing lcm channels");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  // load parameters
  FrankaSimParams sim_params = drake::yaml::LoadYamlFile<FrankaSimParams>(
      "examples/franka/parameters/franka_sim_params.yaml");
  FrankaLcmChannels lcm_channel_params =
      drake::yaml::LoadYamlFile<FrankaLcmChannels>(FLAGS_lcm_channels);

  // load urdf and sphere
  DiagramBuilder<double> builder;
  double sim_dt = sim_params.dt;
  //  double output_dt = sim_params.dt;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, sim_dt);

  Parser parser(&plant);
  drake::multibody::ModelInstanceIndex franka_index =
      parser.AddModels(drake::FindResourceOrThrow(sim_params.franka_model))[0];
  drake::multibody::ModelInstanceIndex end_effector_index =
      parser.AddModels(FindResourceOrThrow(sim_params.end_effector_model))[0];
  drake::multibody::ModelInstanceIndex tray_index =
      parser.AddModels(FindResourceOrThrow(sim_params.tray_model))[0];
  drake::multibody::ModelInstanceIndex box_index =
      parser.AddModels(FindResourceOrThrow(sim_params.box_model))[0];
  multibody::AddFlatTerrain(&plant, &scene_graph, 1.0, 1.0);

  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  Vector3d franka_origin = Eigen::VectorXd::Zero(3);
  Vector3d tool_attachment_frame =
      StdVectorToVectorXd(sim_params.tool_attachment_frame);


  RigidTransform<double> R_X_W = RigidTransform<double>(
      drake::math::RotationMatrix<double>(), franka_origin);
  RigidTransform<double> T_EE_W = RigidTransform<double>(
      drake::math::RotationMatrix<double>(), tool_attachment_frame);
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_link0"),
                   R_X_W);
  plant.WeldFrames(plant.GetFrameByName("panda_link7"),
                   plant.GetFrameByName("plate", end_effector_index), T_EE_W);

  const drake::geometry::GeometrySet& paddle_geom_set =
      plant.CollectRegisteredGeometries({
          &plant.GetBodyByName("panda_link4"),
          &plant.GetBodyByName("panda_link5"),
          &plant.GetBodyByName("panda_link6"),
          &plant.GetBodyByName("panda_link7"),
          &plant.GetBodyByName("panda_link8"),
      });
  auto tray_collision_set = GeometrySet(
      plant.GetCollisionGeometriesForBody(plant.GetBodyByName("tray")));
  plant.ExcludeCollisionGeometriesWithCollisionFilterGroupPair(
      {"paddle", paddle_geom_set}, {"tray", tray_collision_set});

  plant.Finalize();

  /* -------------------------------------------------------------------------------------------*/

  drake::lcm::DrakeLcm drake_lcm;
  auto lcm =
      builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>(&drake_lcm);
  AddActuationRecieverAndStateSenderLcm(
      &builder, plant, lcm, lcm_channel_params.franka_input_channel,
      lcm_channel_params.franka_state_channel, sim_params.publish_rate,
      franka_index, sim_params.publish_efforts, sim_params.actuator_delay);
  auto tray_state_sender =
      builder.AddSystem<systems::ObjectStateSender>(plant, tray_index);
  auto tray_state_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_object_state>(
          lcm_channel_params.tray_state_channel, lcm,
          1.0 / sim_params.publish_rate));
  auto box_state_sender =
      builder.AddSystem<systems::ObjectStateSender>(plant, box_index);
  auto box_state_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_object_state>(
          lcm_channel_params.box_state_channel, lcm,
          1.0 / sim_params.publish_rate));

  builder.Connect(plant.get_state_output_port(tray_index),
                  tray_state_sender->get_input_port_state());
  builder.Connect(plant.get_state_output_port(box_index),
                  box_state_sender->get_input_port_state());
  builder.Connect(tray_state_sender->get_output_port(),
                  tray_state_pub->get_input_port());
  builder.Connect(box_state_sender->get_output_port(),
                  box_state_pub->get_input_port());

  int nq = plant.num_positions();
  int nv = plant.num_velocities();
  int nu = plant.num_actuators();

  if (sim_params.visualize) {
    drake::visualization::AddDefaultVisualization(&builder);
  }

  auto diagram = builder.Build();

  drake::systems::Simulator<double> simulator(*diagram);

  simulator.set_publish_every_time_step(false);
  simulator.set_publish_at_initialization(false);
  simulator.set_target_realtime_rate(sim_params.realtime_rate);

  auto& plant_context = diagram->GetMutableSubsystemContext(
      plant, &simulator.get_mutable_context());

  VectorXd q = VectorXd::Zero(nq);
  std::map<std::string, int> q_map = MakeNameToPositionsMap(plant);

  // initialize EE close to {0.5, 0, 0.12}[m] in task space
  q[q_map["panda_joint1"]] = sim_params.q_init_franka[0];
  q[q_map["panda_joint2"]] = sim_params.q_init_franka[1];
  q[q_map["panda_joint3"]] = sim_params.q_init_franka[2];
  q[q_map["panda_joint4"]] = sim_params.q_init_franka[3];
  q[q_map["panda_joint5"]] = sim_params.q_init_franka[4];
  q[q_map["panda_joint6"]] = sim_params.q_init_franka[5];
  q[q_map["panda_joint7"]] = sim_params.q_init_franka[6];

  q[q_map.at("tray_qw")] = sim_params.q_init_plate[0];
  q[q_map.at("tray_qx")] = sim_params.q_init_plate[1];
  q[q_map.at("tray_qy")] = sim_params.q_init_plate[2];
  q[q_map.at("tray_qz")] = sim_params.q_init_plate[3];
  q[q_map.at("tray_x")] = sim_params.q_init_plate[4];
  q[q_map.at("tray_y")] = sim_params.q_init_plate[5];
  q[q_map.at("tray_z")] = sim_params.q_init_plate[6];

  q[q_map["box_qw"]] = sim_params.q_init_box[0];
  q[q_map["box_qx"]] = sim_params.q_init_box[1];
  q[q_map["box_qy"]] = sim_params.q_init_box[2];
  q[q_map["box_qz"]] = sim_params.q_init_box[3];
  q[q_map["box_x"]] = sim_params.q_init_box[4];
  q[q_map["box_y"]] = sim_params.q_init_box[5];
  q[q_map["box_z"]] = sim_params.q_init_box[6];

  plant.SetPositions(&plant_context, q);

  VectorXd v = VectorXd::Zero(nv);
  plant.SetVelocities(&plant_context, v);

  simulator.Initialize();
  simulator.AdvanceTo(std::numeric_limits<double>::infinity());

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { dairlib::DoMain(argc, argv); }
