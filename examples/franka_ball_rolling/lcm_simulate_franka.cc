#include <cmath>
#include <vector>

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
#include <drake/systems/primitives/multiplexer.h>
#include <drake/systems/primitives/vector_log_sink.h>
#include <drake/visualization/visualization_config_functions.h>
#include <gflags/gflags.h>

#include "common/eigen_utils.h"
#include "examples/franka_ball_rolling/parameters/lcm_channels_params.h"
#include "examples/franka_ball_rolling/parameters/simulate_franka_params.h"
#include "multibody/multibody_utils.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"

namespace dairlib {

using drake::geometry::SceneGraph;
using drake::math::RigidTransform;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::Context;
using drake::systems::DiagramBuilder;
using drake::systems::VectorLogSink;
using drake::systems::lcm::LcmPublisherSystem;
using multibody::MakeNameToPositionsMap;
using multibody::MakeNameToVelocitiesMap;
using systems::AddActuationRecieverAndStateSenderLcm;

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

int DoMain(int argc, char* argv[]) {
  /* -------------------------------------------------------------------------------------------*/
  // load parameters
  SimulateFrankaParams sim_param = drake::yaml::LoadYamlFile<
      SimulateFrankaParams>(
      "examples/franka_ball_rolling/parameters/simulate_franka_params.yaml");
  BallRollingLcmChannels lcm_channel_param = drake::yaml::LoadYamlFile<
      BallRollingLcmChannels>(
      "examples/franka_ball_rolling/parameters/lcm_channels_sim_params.yaml");

  // set plant, simulation step and publish time rates
  DiagramBuilder<double> builder;
  double sim_dt = sim_param.sim_dt;
  double publish_dt = sim_param.publish_dt;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, sim_dt);

  // load urdf models
  Parser parser(&plant);
  parser.SetAutoRenaming(true);
  drake::multibody::ModelInstanceIndex franka_index =
      parser.AddModels(sim_param.franka_model)[0];
  drake::multibody::ModelInstanceIndex ground_index =
      parser.AddModels(sim_param.ground_model)[0];
  drake::multibody::ModelInstanceIndex end_effector_index =
      parser.AddModels(sim_param.end_effector_model)[0];
  drake::multibody::ModelInstanceIndex ball_index =
      parser.AddModels(sim_param.ball_model)[0];

  // visually give a offset stage model, not important
  parser.AddModels(sim_param.offset_model);

  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  RigidTransform<double> X_F_EE =
      RigidTransform<double>(sim_param.tool_attachment_frame);
  RigidTransform<double> X_F_G =
      RigidTransform<double>(sim_param.ground_offset_frame);

  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_link0"),
                   X_WI);
  plant.WeldFrames(
      plant.GetFrameByName("panda_link7"),
      plant.GetFrameByName("end_effector_base", end_effector_index), X_F_EE);
  plant.WeldFrames(plant.GetFrameByName("panda_link0"),
                   plant.GetFrameByName("visual_table_offset"), X_WI);
  plant.WeldFrames(plant.GetFrameByName("panda_link0"),
                   plant.GetFrameByName("ground", ground_index), X_F_G);

  plant.Finalize();

  /* -------------------------------------------------------------------------------------------*/
  drake::lcm::DrakeLcm drake_lcm;
  auto lcm =
      builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>(&drake_lcm);

  //  auto passthrough = AddActuationRecieverAndStateSenderLcm(
  //          &builder, plant, lcm, "FRANKA_INPUT", "FRANKA_OUTPUT",
  //          1 / publish_dt, true, 0.0);

  auto passthrough = AddActuationRecieverAndStateSenderLcm(
      &builder, plant, lcm, lcm_channel_param.franka_input_channel,
      lcm_channel_param.franka_output_channel, 1 / publish_dt, franka_index,
      true, 0.0);

  auto ball_state_sender =
      builder.AddSystem<systems::ObjectStateSender>(plant, ball_index);
  auto ball_state_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_object_state>(
          lcm_channel_param.true_ball_state_channel, lcm, 1.0 / 80));

  builder.Connect(plant.get_state_output_port(ball_index),
                  ball_state_sender->get_input_port_state());
  builder.Connect(ball_state_sender->get_output_port(),
                  ball_state_pub->get_input_port());

  int nq = plant.num_positions();
  int nv = plant.num_velocities();
  int nu = plant.num_actuators();
  auto logger =
      builder.AddSystem<VectorLogSink<double>>(nq + nv + nu, publish_dt);

  // default visualizer that shows everything
  if (sim_param.visualize_defualt_drake) {
    drake::visualization::AddDefaultVisualization(&builder);
  }

  // multiplex state and input for logger
  std::vector<int> input_sizes = {nq + nv, nu};
  auto mux =
      builder.AddSystem<drake::systems::Multiplexer<double>>(input_sizes);

  builder.Connect(plant.get_state_output_port(), mux->get_input_port(0));
  builder.Connect(passthrough->get_output_port(), mux->get_input_port(1));
  builder.Connect(mux->get_output_port(0), logger->get_input_port(0));

  auto diagram = builder.Build();
  diagram->set_name(("Diagram_Simulate_Franka"));
  DrawAndSaveDiagramGraph(
      *diagram, "examples/franka_ball_rolling/diagram_lcm_simulate_franka");

  drake::systems::Simulator<double> simulator(*diagram);

  simulator.set_publish_every_time_step(false);
  simulator.set_publish_at_initialization(false);
  simulator.set_target_realtime_rate(sim_param.realtime_rate);

  auto& plant_context = diagram->GetMutableSubsystemContext(
      plant, &simulator.get_mutable_context());

  VectorXd q = VectorXd::Zero(nq);
  std::map<std::string, int> q_map = MakeNameToPositionsMap(plant);

  // TODO:: find a more elegant way to assign these, possibly using model
  // indices initialize franka configurations
  //  q.head(plant.num_positions(franka_index)) = sim_param.q_init_franka;

  q[q_map["panda_joint1"]] = sim_param.q_init_franka(0);
  q[q_map["panda_joint2"]] = sim_param.q_init_franka(1);
  q[q_map["panda_joint3"]] = sim_param.q_init_franka(2);
  q[q_map["panda_joint4"]] = sim_param.q_init_franka(3);
  q[q_map["panda_joint5"]] = sim_param.q_init_franka(4);
  q[q_map["panda_joint6"]] = sim_param.q_init_franka(5);
  q[q_map["panda_joint7"]] = sim_param.q_init_franka(6);

  // initialize ball positions
  double traj_radius = sim_param.traj_radius;
  //  VectorXd q_ball = VectorXd::Zero(plant.num_positions(ball_index));
  q[q_map["sphere_qw"]] = sim_param.q_init_ball(0);
  q[q_map["sphere_qx"]] = sim_param.q_init_ball(1);
  q[q_map["sphere_qy"]] = sim_param.q_init_ball(2);
  q[q_map["sphere_qz"]] = sim_param.q_init_ball(3);
  q[q_map["sphere_x"]] =
      sim_param.x_c + traj_radius * sin(M_PI * sim_param.phase / 180.0);
  q[q_map["sphere_y"]] =
      sim_param.y_c + traj_radius * cos(M_PI * sim_param.phase / 180.0);
  q[q_map["sphere_z"]] =
      sim_param.ball_radius + sim_param.ground_offset_frame(2);

  plant.SetPositions(&plant_context, q);

  VectorXd v = VectorXd::Zero(nv);
  plant.SetVelocities(&plant_context, v);

  simulator.Initialize();
  simulator.AdvanceTo(std::numeric_limits<double>::infinity());

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { dairlib::DoMain(argc, argv); }
