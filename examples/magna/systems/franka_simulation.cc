#include <math.h>

#include <string>
#include <tuple>
#include <vector>

#include <Eigen/Dense>
#include <drake/common/find_resource.h>
#include <drake/geometry/scene_graph.h>
#include <drake/lcm/drake_lcm.h>
#include <drake/math/rigid_transform.h>
#include <drake/math/roll_pitch_yaw.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/lcm/lcm_interface_system.h>
#include <drake/systems/primitives/constant_vector_source.h>
#include <drake/systems/primitives/demultiplexer.h>
#include <drake/systems/primitives/multiplexer.h>
#include <drake/systems/primitives/zero_order_hold.h>
#include <gflags/gflags.h>
#include <optional>

#include "common/find_resource.h"
#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "examples/magna/systems/controllers/franka_cartesian_osc_controller_params.h"
#include "examples/magna/systems/franka_common.h"
#include "examples/magna/systems/franka_simulation_utils.h"
#include "multibody/multibody_utils.h"
#include "systems/primitives/subvector_pass_through.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"

#include "drake/lcmt_schunk_wsg_command.hpp"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

using drake::geometry::SceneGraph;
using drake::lcm::DrakeLcm;
using drake::math::RigidTransform;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::BasicVector;
using drake::systems::DiagramBuilder;
using drake::systems::lcm::LcmInterfaceSystem;

using Eigen::Vector3d;
using Eigen::VectorXd;

DEFINE_string(franka_input_channel, "FRANKA_INPUT",
              "LCM channel for receiving Franka input");
DEFINE_string(franka_state_channel, "FRANKA_STATE",
              "LCM channel for sending Franka state");
DEFINE_string(franka_with_hand_state_channel, "FRANKA_WITH_HAND_STATE",
              "LCM channel for sending Franka with hand state");
DEFINE_string(gripper_state_channel, "GRIPPER_STATE",
              "LCM channel for sending Gripper state");
DEFINE_string(gripper_input_channel, "GRIPPER_INPUT",
              "LCM channel for receiving Gripper actuation command");
DEFINE_double(franka_state_publish_rate, 1000.0,
              "Rate (in Hz) at which to publish Franka state over LCM");
DEFINE_double(simulation_dt, 0.0001, "Simulation time step");
DEFINE_double(realtime_rate, 1.0, "Target realtime rate for simulation");
DEFINE_string(lcm_url, "udpm://239.255.76.67:7667?ttl=0",
              "LCM URL with IP, port, and TTL settings");

namespace dairlib {
namespace examples {
namespace magna {
namespace systems {

int RunFrankaSimulation() {
  // load urdf and sphere
  DiagramBuilder<double> builder;
  double sim_dt = FLAGS_simulation_dt;

  // Default Plant containing franka with hand
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, sim_dt);
  auto plant_index =
      AddFrankaToPlant(&plant, &scene_graph, std::nullopt /* no end effector */,
                       true /* with hand */);
  plant.Finalize();

  // Separate plant containing only franka (no hand) for state publishing and
  // input handling
  drake::multibody::MultibodyPlant<double> franka_mbplant(0.0);
  auto franka_index = AddFrankaToPlant(&franka_mbplant, nullptr,
                                       std::nullopt /* no end effector */,
                                       false /* without hand */);
  franka_mbplant.Finalize();

  // Separate plant containing only panda hand for state publishing
  drake::multibody::MultibodyPlant<double> hand_mbplant(0.0);
  auto hand_index = AddFrankaHandToPlant(&hand_mbplant, nullptr);
  hand_mbplant.Finalize();

  /* -------------------------------------------------------------------------------------------*/

  drake::lcm::DrakeLcm drake_lcm(FLAGS_lcm_url);
  auto lcm =
      builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>(&drake_lcm);

  /* -------------------------------------------------------------------------------------------*/
  // Set up Franka simulation
  // Create LCM input for franka actuators
  auto franka_input_sub = builder.AddSystem(
      drake::systems::lcm::LcmSubscriberSystem::Make<dairlib::lcmt_robot_input>(
          FLAGS_franka_input_channel, lcm));
  auto franka_input_receiver =
      builder.AddSystem<dairlib::systems::RobotInputReceiver>(franka_mbplant);
  builder.Connect(*franka_input_sub, *franka_input_receiver);

  // Get only inputs for Franka
  auto franka_passthrough =
      builder.AddSystem<dairlib::systems::SubvectorPassThrough>(
          franka_input_receiver->get_output_port(0).size(), 0,
          franka_mbplant.num_actuators());
  builder.Connect(*franka_input_receiver, *franka_passthrough);

  // Create LCM input for gripper actuators
  auto hand_input_sub = builder.AddSystem(
      drake::systems::lcm::LcmSubscriberSystem::Make<dairlib::lcmt_robot_input>(
          FLAGS_gripper_input_channel, lcm));
  auto hand_input_receiver =
      builder.AddSystem<dairlib::systems::RobotInputReceiver>(hand_mbplant);
  builder.Connect(*hand_input_sub, *hand_input_receiver);

  auto hand_passthrough =
      builder.AddSystem<dairlib::systems::SubvectorPassThrough>(
          hand_input_receiver->get_output_port(0).size(), 0,
          hand_mbplant.num_actuators());
  builder.Connect(*hand_input_receiver, *hand_passthrough);

  /* -------------------------------------------------------------------------------------------*/
  // Create LCM output for state and efforts

  auto plant_state_pub = builder.AddSystem(
      drake::systems::lcm::LcmPublisherSystem::Make<dairlib::lcmt_robot_output>(
          FLAGS_franka_with_hand_state_channel, lcm,
          1.0 / FLAGS_franka_state_publish_rate));
  auto plant_state_sender =
      builder.AddSystem<dairlib::systems::RobotOutputSender>(
          plant, plant_index, true /* include efforts */);
  builder.Connect(plant.get_state_output_port(plant_index),
                  plant_state_sender->get_input_port_state());
  builder.Connect(*plant_state_sender, *plant_state_pub);

  std::vector<int> state_sizes = {
      plant.num_positions() - 2, 2 /* state and effort */,
      plant.num_velocities() - 2, 2 /* state and effort */};
  auto state_demux =
      builder.AddSystem<drake::systems::Demultiplexer<double>>(state_sizes);
  builder.Connect(plant.get_state_output_port(plant_index),
                  state_demux->get_input_port());

  // publish franka states
  std::vector<int> franka_state_sizes = {franka_mbplant.num_positions(),
                                         franka_mbplant.num_velocities()};
  auto franka_state_mux =
      builder.AddSystem<drake::systems::Multiplexer<double>>(
          franka_state_sizes);
  auto franka_state_sender =
      builder.AddSystem<dairlib::systems::RobotOutputSender>(
          franka_mbplant, franka_index, true /* include efforts */);
  builder.Connect(state_demux->get_output_port(0),
                  franka_state_mux->get_input_port(0));
  builder.Connect(state_demux->get_output_port(2),
                  franka_state_mux->get_input_port(1));
  builder.Connect(franka_state_mux->get_output_port(),
                  franka_state_sender->get_input_port_state());
  auto franka_state_pub = builder.AddSystem(
      drake::systems::lcm::LcmPublisherSystem::Make<dairlib::lcmt_robot_output>(
          FLAGS_franka_state_channel, lcm,
          1.0 / FLAGS_franka_state_publish_rate));
  builder.Connect(*franka_state_sender, *franka_state_pub);

  // publish hand states
  auto hand_state_mux = builder.AddSystem<drake::systems::Multiplexer<double>>(
      std::vector<int>{2, 2});
  auto hand_state_sender =
      builder.AddSystem<dairlib::systems::RobotOutputSender>(
          hand_mbplant, hand_index, true /* include efforts */);
  builder.Connect(state_demux->get_output_port(1) /* effort port */,
                  hand_state_mux->get_input_port(0));
  builder.Connect(state_demux->get_output_port(3) /* velocity port */,
                  hand_state_mux->get_input_port(1));
  builder.Connect(hand_state_mux->get_output_port(),
                  hand_state_sender->get_input_port_state());
  auto hand_state_pub = builder.AddSystem(
      drake::systems::lcm::LcmPublisherSystem::Make<dairlib::lcmt_robot_output>(
          FLAGS_gripper_state_channel, lcm,
          1.0 / FLAGS_franka_state_publish_rate));
  builder.Connect(*hand_state_sender, *hand_state_pub);
  /* -------------------------------------------------------------------------------------------*/
  // Setup Actuation with Gravity Compensation
  drake::log()->info("Setting up gravity compensation...");

  std::vector<int> actuation_sizes = {
        plant.get_actuation_input_port().size() - 2, 2 /* gripper actuators
        */};
  auto mux =
      builder.AddSystem<drake::systems::Multiplexer<double>>(actuation_sizes);

  builder.Connect(franka_passthrough->get_output_port(),
                  mux->get_input_port(0));
  builder.Connect(hand_passthrough->get_output_port(),
                  mux->get_input_port(1));

  auto gravity_context = plant.CreateDefaultContext();
  auto gravity_compensator =
      builder.AddSystem<GravityCompensator>(plant, *gravity_context);
  builder.Connect(mux->get_output_port(),
                  gravity_compensator->get_input_port_actuation());
  builder.Connect(plant.get_state_output_port(plant_index),
                  gravity_compensator->get_input_port_state());

  builder.Connect(gravity_compensator->get_output_port_compensated_actuation(),
                  plant.get_actuation_input_port());

  // Connect gravity compensated efforts to state sender for franka arm
  builder.Connect(franka_passthrough->get_output_port(),
                  franka_state_sender->get_input_port_effort());
  builder.Connect(hand_passthrough->get_output_port(),
                  hand_state_sender->get_input_port_effort());
  builder.Connect(gravity_compensator->get_output_port_compensated_actuation(),
                  plant_state_sender->get_input_port_effort());
  /* -------------------------------------------------------------------------------------------*/
  drake::log()->info("Building simulation diagram...");
  int nq = plant.num_positions();
  int nv = plant.num_velocities();

  auto owned_diagram = builder.Build();
  std::shared_ptr<drake::systems::Diagram<double>> shared_diagram_ptr =
      std::move(owned_diagram);
  shared_diagram_ptr->set_name(("franka_simulation_diagram"));
  DrawAndSaveDiagramGraph(*shared_diagram_ptr);

  drake::systems::Simulator<double> simulator(*shared_diagram_ptr);

  simulator.set_publish_every_time_step(false);
  simulator.set_publish_at_initialization(false);
  simulator.set_target_realtime_rate(FLAGS_realtime_rate);

  auto& plant_context = shared_diagram_ptr->GetMutableSubsystemContext(
      plant, &simulator.get_mutable_context());

  VectorXd q = VectorXd::Zero(nq);

  q << 0, M_PI / 8, 0, -3 * M_PI / 4, 0, 7 * M_PI / 8, 0;
  plant.SetPositions(&plant_context, q);

  VectorXd v = VectorXd::Zero(nv);
  plant.SetVelocities(&plant_context, v);

  simulator.Initialize();
  simulator.AdvanceTo(std::numeric_limits<double>::infinity());

  return 0;
}
}  // namespace systems
}  // namespace magna
}  // namespace examples
}  // namespace dairlib

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::set_log_level("trace");
  dairlib::examples::magna::systems::RunFrankaSimulation();
}
