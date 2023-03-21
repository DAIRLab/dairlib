#include <math.h>

#include <vector>

#include <drake/common/find_resource.h>
#include <drake/geometry/drake_visualizer.h>
#include <drake/lcm/drake_lcm.h>
#include <drake/math/rigid_transform.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/systems/primitives/multiplexer.h>
#include <drake/systems/primitives/vector_log_sink.h>

#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "examples/franka/franka_sim_params.h"
#include "multibody/multibody_utils.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"

#include "drake/common/yaml/yaml_io.h"
#include "drake/geometry/meshcat_visualizer.h"
#include "drake/geometry/meshcat_visualizer_params.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

namespace dairlib {

using dairlib::systems::SubvectorPassThrough;
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
using Eigen::VectorXd;
using Eigen::Vector3d;

int DoMain(int argc, char* argv[]) {
  // load parameters
  FrankaSimParams sim_params = drake::yaml::LoadYamlFile<FrankaSimParams>(
      "examples/franka/franka_sim_params.yaml");

  // load urdf and sphere
  DiagramBuilder<double> builder;
  double sim_dt = sim_params.dt;
  double output_dt = sim_params.dt;
  //  auto scene_graph =
  //  builder.AddSystem(std::make_unique<SceneGraph<double>>());
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, sim_dt);

  Parser parser(&plant);
  drake::multibody::ModelInstanceIndex franka_index = parser.AddModelFromFile("examples/franka/urdf/franka_box.urdf");
  drake::multibody::ModelInstanceIndex table_index = parser.AddModelFromFile(drake::FindResourceOrThrow(
      "drake/examples/kuka_iiwa_arm/models/table/"
      "extra_heavy_duty_table_surface_only_collision.sdf"));
  drake::multibody::ModelInstanceIndex tray_index = parser.AddModelFromFile(drake::FindResourceOrThrow(
      "drake/examples/kuka_iiwa_arm/models/objects/open_top_box.urdf"));

  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  Vector3d shift = Eigen::VectorXd::Zero(3);
  shift(2) = 0.7645;
  RigidTransform<double> R_X_W = RigidTransform<double>(drake::math::RotationMatrix<double>(), shift);
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("link"),
                   X_WI);
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_link0"), R_X_W);

  plant.Finalize();

  /* -------------------------------------------------------------------------------------------*/

  drake::lcm::DrakeLcm drake_lcm;
  auto lcm =
      builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>(&drake_lcm);
  auto passthrough = AddActuationRecieverAndStateSenderLcm(
      &builder, plant, lcm, sim_params.controller_channel,
      sim_params.state_channel, sim_params.publish_rate,
      franka_index,
      sim_params.publish_efforts, sim_params.actuator_delay);
  auto tray_state_sender =
      builder.AddSystem<systems::RobotOutputSender>(plant, tray_index);
  auto tray_state_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_output>(
          sim_params.tray_state_channel, lcm, 1.0 / sim_params.publish_rate));

  builder.Connect(plant.get_state_output_port(tray_index),
                  tray_state_sender->get_input_port_state());
  builder.Connect(tray_state_sender->get_output_port(),
                  tray_state_pub->get_input_port());

  int nq = plant.num_positions();
  int nv = plant.num_velocities();
  int nu = plant.num_actuators();

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
  q[q_map["panda_joint3"]] = sim_params.q_init_franka[2];
  q[q_map["panda_joint4"]] = sim_params.q_init_franka[3];
  q[q_map["panda_joint5"]] = sim_params.q_init_franka[4];
  q[q_map["panda_joint6"]] = sim_params.q_init_franka[5];
  q[q_map["panda_joint2"]] = sim_params.q_init_franka[1];
  q[q_map["panda_joint7"]] = sim_params.q_init_franka[6];
  q[q_map["panda_joint1"]] = sim_params.q_init_franka[0];

  q[plant.num_positions() - 7] = 1;
  q[plant.num_positions() - 1] = 2;

  plant.SetPositions(&plant_context, q);

  VectorXd v = VectorXd::Zero(nv);
  plant.SetVelocities(&plant_context, v);

  simulator.Initialize();
  simulator.AdvanceTo(std::numeric_limits<double>::infinity());

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { dairlib::DoMain(argc, argv); }
