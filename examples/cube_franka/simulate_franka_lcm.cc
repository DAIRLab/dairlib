#include <vector>
#include <math.h>

#include "drake/geometry/meshcat_visualizer.h"
#include "drake/geometry/meshcat_visualizer_params.h"

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include <drake/lcm/drake_lcm.h>
#include <drake/multibody/tree/multibody_element.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/common/trajectories/piecewise_polynomial.h>
#include <drake/geometry/drake_visualizer.h>
#include <drake/math/rigid_transform.h>
#include "drake/math/autodiff.h"
#include "drake/common/yaml/yaml_io.h"
#include <drake/systems/primitives/vector_log_sink.h>
#include <drake/systems/primitives/multiplexer.h>

#include "systems/robot_lcm_systems.h"
#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_c3.hpp"
#include "multibody/multibody_utils.h"
#include "systems/system_utils.h"

#include "examples/cube_franka/c3_parameters.h"
#include "systems/robot_lcm_systems.h"
#include "systems/framework/lcm_driven_loop.h"

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
using multibody::makeNameToPositionsMap;
using multibody::makeNameToVelocitiesMap;
using systems::AddActuationRecieverAndStateSenderLcm;

using Eigen::VectorXd;
using Eigen::MatrixXd;

int DoMain(int argc, char* argv[]){
  // load parameters
  C3Parameters param = drake::yaml::LoadYamlFile<C3Parameters>(
    "examples/cube_franka/parameters.yaml");

  // load urdf and sphere
  DiagramBuilder<double> builder;
  double sim_dt = param.sim_dt;
  double output_dt = param.sim_dt;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, sim_dt);

  Parser parser(&plant);
  parser.AddModelFromFile("examples/cube_franka/robot_properties_fingers/urdf/franka_box.urdf");
  // parser.AddModelFromFile("examples/cube_franka/robot_properties_fingers/urdf/sphere.urdf");
  parser.AddModelFromFile("examples/cube_franka/robot_properties_fingers/urdf/cube_v3.sdf");
  
  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_link0"), X_WI);
  plant.Finalize();
  
  /* -------------------------------------------------------------------------------------------*/

  drake::lcm::DrakeLcm drake_lcm;
  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>(&drake_lcm);
  
  auto passthrough = AddActuationRecieverAndStateSenderLcm(
    &builder, plant, lcm, "FRANKA_INPUT", "FRANKA_OUTPUT",
    1/output_dt, true, 0.0);

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
  auto logger = builder.AddSystem<VectorLogSink<double>>(nq+nv+nu, output_dt);
  
  // multiplex state and input for logger
  std::vector<int> input_sizes = {nq+nv, nu};
  auto mux = builder.AddSystem<drake::systems::Multiplexer<double>>(input_sizes);

  builder.Connect(plant.get_state_output_port(), mux->get_input_port(0));
  builder.Connect(passthrough->get_output_port(), mux->get_input_port(1));
  builder.Connect(mux->get_output_port(0), logger->get_input_port(0));

  auto diagram = builder.Build();
  // DrawAndSaveDiagramGraph(*diagram, "examples/franka_trajectory_following/diagram_simulate_franka_lcm");

  drake::systems::Simulator<double> simulator(*diagram);
  
  simulator.set_publish_every_time_step(false);
  simulator.set_publish_at_initialization(false);
  simulator.set_target_realtime_rate(param.realtime_rate);

  auto& plant_context = diagram->GetMutableSubsystemContext(
      plant, &simulator.get_mutable_context());
  
  VectorXd q = VectorXd::Zero(nq);
  std::map<std::string, int> q_map = makeNameToPositionsMap(plant);
  
  // initialize EE close to {0.5, 0, 0.12}[m] in task space
  q[q_map["panda_joint1"]] = param.q_init_franka(0);
  q[q_map["panda_joint2"]] = param.q_init_franka(1);
  q[q_map["panda_joint3"]] = param.q_init_franka(2);
  q[q_map["panda_joint4"]] = param.q_init_franka(3);
  q[q_map["panda_joint5"]] = param.q_init_franka(4);
  q[q_map["panda_joint6"]] = param.q_init_franka(5);
  q[q_map["panda_joint7"]] = param.q_init_franka(6);

  // initialize ball
  double traj_radius = param.traj_radius;
  q[q_map["base_qw"]] = param.q_init_ball(0);
  q[q_map["base_qx"]] = param.q_init_ball(1);
  q[q_map["base_qy"]] = param.q_init_ball(2);
  q[q_map["base_qz"]] = param.q_init_ball(3);
  q[q_map["base_x"]] = param.x_c + traj_radius * sin(M_PI * param.phase / 180.0);
  q[q_map["base_y"]] = param.y_c + traj_radius * cos(M_PI * param.phase / 180.0);
  q[q_map["base_z"]] = param.ball_radius + param.table_offset;

  plant.SetPositions(&plant_context, q);

  VectorXd v = VectorXd::Zero(nv);
  plant.SetVelocities(&plant_context, v);

  simulator.Initialize();
  simulator.AdvanceTo(std::numeric_limits<double>::infinity());

  // do data logging here

  return 0;
}

} // namespace dairlib

int main(int argc, char* argv[]) { dairlib::DoMain(argc, argv);}
