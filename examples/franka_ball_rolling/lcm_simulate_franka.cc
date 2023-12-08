#include <math.h>

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
#include <drake/systems/lcm/lcm_subscriber_system.h>
#include <drake/systems/primitives/multiplexer.h>
#include <drake/systems/primitives/vector_log_sink.h>
#include <drake/visualization/visualization_config_functions.h>
#include <gflags/gflags.h>

#include "common/eigen_utils.h"
#include "examples/franka_ball_rolling/parameters/simulate_franka_params.h"
#include "multibody/multibody_utils.h"
#include "systems/robot_lcm_systems.h"

namespace dairlib {

using drake::geometry::SceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::math::RigidTransform;
using drake::systems::DiagramBuilder;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::Context;
using drake::multibody::Parser;
using drake::trajectories::PiecewisePolynomial;
using drake::systems::VectorLogSink;
using multibody::MakeNameToPositionsMap;
using multibody::MakeNameToVelocitiesMap;
using systems::AddActuationRecieverAndStateSenderLcm;

using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::MatrixXd;

int DoMain(int argc, char* argv[]){
  // load parameters
  SimulateFrankaParams sim_param = drake::yaml::LoadYamlFile<SimulateFrankaParams>(
    "examples/franka_ball_rolling/parameters/simulate_franka_params.yaml");

  // set simulation step and publish time rates
  DiagramBuilder<double> builder;
  double sim_dt = sim_param.sim_dt;
  double publish_dt = sim_param.publish_dt;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, sim_dt);

  // load urdf models
  Parser parser(&plant);
  parser.AddModelFromFile(sim_param.franka_model);
  parser.AddModelFromFile(sim_param.offset_model);
  parser.AddModelFromFile(sim_param.ground_model);
  parser.AddModelFromFile(sim_param.end_effector_model);
  parser.AddModelFromFile(sim_param.ball_model);
  
  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  RigidTransform<double> X_F_EE = RigidTransform<double>(sim_param.tool_attachment_frame);
  RigidTransform<double> X_F_G = RigidTransform<double>(sim_param.ground_offset_frame);

  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_link0"), X_WI);
  plant.WeldFrames(plant.GetFrameByName("panda_link7"), plant.GetFrameByName("end_effector_base"), X_F_EE);
  plant.WeldFrames(plant.GetFrameByName("panda_link0"), plant.GetFrameByName("visual_table_offset"), X_WI);
  plant.WeldFrames(plant.GetFrameByName("panda_link0"), plant.GetFrameByName("ground"), X_F_G);


  plant.Finalize();
  
  /* -------------------------------------------------------------------------------------------*/

  drake::lcm::DrakeLcm drake_lcm;
  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>(&drake_lcm);
  
  auto passthrough = AddActuationRecieverAndStateSenderLcm(
          &builder, plant, lcm, "FRANKA_INPUT", "FRANKA_OUTPUT",
          1 / publish_dt, true, 0.0);

  /// meshcat visualizer
    drake::geometry::DrakeVisualizer<double>::AddToBuilder(&builder, scene_graph);
    drake::geometry::MeshcatVisualizerParams params;
    params.publish_period = 1.0/30.0;
    auto meshcat = std::make_shared<drake::geometry::Meshcat>();
    auto visualizer = &drake::geometry::MeshcatVisualizer<double>::AddToBuilder(
        &builder, scene_graph, meshcat, std::move(params));

  int nq = plant.num_positions();
  int nv = plant.num_velocities();
  int nu = plant.num_actuators();
  auto logger = builder.AddSystem<VectorLogSink<double>>(nq+nv+nu, publish_dt);

  // default visualizer that shows everything
  if (true) {
        drake::visualization::AddDefaultVisualization(&builder);
  }
  
  // multiplex state and input for logger
  std::vector<int> input_sizes = {nq+nv, nu};
  auto mux = builder.AddSystem<drake::systems::Multiplexer<double>>(input_sizes);

  builder.Connect(plant.get_state_output_port(), mux->get_input_port(0));
  builder.Connect(passthrough->get_output_port(), mux->get_input_port(1));
  builder.Connect(mux->get_output_port(0), logger->get_input_port(0));

  auto diagram = builder.Build();
  // DrawAndSaveDiagramGraph(*diagram, "examples/franka_ball_rolling/diagram_simulate_franka_lcm");

  drake::systems::Simulator<double> simulator(*diagram);
  
  simulator.set_publish_every_time_step(false);
  simulator.set_publish_at_initialization(false);
  simulator.set_target_realtime_rate(sim_param.realtime_rate);

  auto& plant_context = diagram->GetMutableSubsystemContext(
      plant, &simulator.get_mutable_context());
  
  VectorXd q = VectorXd::Zero(nq);
  std::map<std::string, int> q_map = MakeNameToPositionsMap(plant);

  // initialize EE close to {0.5, 0, 0.12}[m] in task space
  q[q_map["panda_joint1"]] = sim_param.q_init_franka(0);
  q[q_map["panda_joint2"]] = sim_param.q_init_franka(1);
  q[q_map["panda_joint3"]] = sim_param.q_init_franka(2);
  q[q_map["panda_joint4"]] = sim_param.q_init_franka(3);
  q[q_map["panda_joint5"]] = sim_param.q_init_franka(4);
  q[q_map["panda_joint6"]] = sim_param.q_init_franka(5);
  q[q_map["panda_joint7"]] = sim_param.q_init_franka(6);

  // initialize ball
  double traj_radius = sim_param.traj_radius;
  q[q_map["base_qw"]] = sim_param.q_init_ball(0);
  q[q_map["base_qx"]] = sim_param.q_init_ball(1);
  q[q_map["base_qy"]] = sim_param.q_init_ball(2);
  q[q_map["base_qz"]] = sim_param.q_init_ball(3);
  q[q_map["base_x"]] = sim_param.x_c + traj_radius * sin(M_PI * sim_param.phase / 180.0);
  q[q_map["base_y"]] = sim_param.y_c + traj_radius * cos(M_PI * sim_param.phase / 180.0);
  q[q_map["base_z"]] = sim_param.ball_radius - sim_param.ground_offset_frame(2);

  plant.SetPositions(&plant_context, q);

  VectorXd v = VectorXd::Zero(nv);
  plant.SetVelocities(&plant_context, v);

  simulator.Initialize();
  simulator.AdvanceTo(std::numeric_limits<double>::infinity());

  return 0;
}

} // namespace dairlib

int main(int argc, char* argv[]) { dairlib::DoMain(argc, argv);}
