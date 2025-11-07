#include <math.h>

#include <limits>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include <Eigen/Dense>
#include <gflags/gflags.h>

#include "common/find_resource.h"
#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "examples/magna/systems/franka_common.h"
#include "examples/magna/systems/franka_simulation_utils.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"

#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

using drake::geometry::SceneGraph;
using drake::lcm::DrakeLcm;
using drake::math::RigidTransform;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::BasicVector;
using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
using drake::systems::TriggerType;
using drake::systems::TriggerTypeSet;
using drake::systems::lcm::LcmInterfaceSystem;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;

using Eigen::Vector3d;
using Eigen::VectorXd;

using dairlib::systems::LcmDrivenLoop;
using dairlib::systems::RobotCommandSender;
using dairlib::systems::RobotOutputReceiver;

DEFINE_string(gripper_command_channel, "GRIPPER_COMMAND",
              "LCM channel for receiving Gripper command");
DEFINE_string(gripper_state_channel, "GRIPPER_STATE",
              "LCM channel for receiving Gripper state from simulation");
DEFINE_string(
    gripper_input_channel, "GRIPPER_INPUT",
    "LCM channel for sending Gripper actuation command to franka simulation");
DEFINE_double(simulation_dt, 0.0001, "Simulation time step");
DEFINE_double(realtime_rate, 1.0, "Target realtime rate for simulation");
DEFINE_string(lcm_url, "udpm://239.255.76.67:7667?ttl=0",
              "LCM URL with IP, port, and TTL settings");

namespace dairlib {
namespace examples {
namespace magna {
namespace systems {

int RunFrankaHandSimulation() {
  DiagramBuilder<double> builder;

  // Separate plant containing only panda hand for state publishing
  MultibodyPlant<double> hand_mbplant(0.0);
  auto hand_index = AddFrankaHandToPlant(&hand_mbplant, nullptr);
  hand_mbplant.Finalize();

  /* -------------------------------------------------------------------------------------------*/

  drake::lcm::DrakeLcm lcm(FLAGS_lcm_url);

  // Create command state receiver.
  auto hand_state_receiver =
      builder.AddSystem<RobotOutputReceiver>(hand_mbplant, hand_index);

  auto hand_mbplant_context = hand_mbplant.CreateDefaultContext();
  const auto& gripper_actuation_input_port = SimulatePandaHand(
      &builder, hand_mbplant, hand_mbplant_context.get(), &lcm,
      FLAGS_gripper_command_channel,
      hand_state_receiver->get_output_port() /* effort port */,
      "examples/magna/params/franka_osc_qp_settings.yaml");

  auto hand_command_sender =
      builder.AddSystem<RobotCommandSender>(hand_mbplant);
  auto hand_command_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          FLAGS_gripper_input_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));

  builder.Connect(gripper_actuation_input_port,
                  hand_command_sender->get_input_port());
  builder.Connect(hand_command_sender->get_output_port(),
                  hand_command_pub->get_input_port());

  /* -------------------------------------------------------------------------------------------*/
  drake::log()->info("Building simulation diagram...");

  auto owned_diagram = builder.Build();
  std::shared_ptr<drake::systems::Diagram<double>> shared_diagram_ptr =
      std::move(owned_diagram);
  shared_diagram_ptr->set_name("franka_hand_simulation_diagram");
  DrawAndSaveDiagramGraph(*shared_diagram_ptr);

  // Run lcm-driven simulation
  LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm, shared_diagram_ptr, hand_state_receiver,
      FLAGS_gripper_state_channel, true);
  drake::log()->info("Starting LCM-driven loop simulation.");
  loop.Simulate();
  return 0;
}

}  // namespace systems
}  // namespace magna
}  // namespace examples
}  // namespace dairlib

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::set_log_level("trace");
  return dairlib::examples::magna::systems::RunFrankaHandSimulation();
}
