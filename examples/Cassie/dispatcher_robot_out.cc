#include <memory>

#include <gflags/gflags.h>
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

#include "attic/multibody/rigidbody_utils.h"
#include "systems/robot_lcm_systems.h"
#include "examples/Cassie/networking/simple_cassie_udp_subscriber.h"
#include "examples/Cassie/networking/cassie_output_sender.h"
#include "examples/Cassie/networking/cassie_output_receiver.h"
#include "examples/Cassie/cassie_rbt_state_estimator.h"
#include "examples/Cassie/cassie_utils.h"
#include "dairlib/lcmt_cassie_out.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "systems/framework/lcm_driven_loop.h"

namespace dairlib {

using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
using drake::systems::Context;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::TriggerType;

// Simulation parameters.
DEFINE_string(address, "127.0.0.1", "IPv4 address to receive on.");
DEFINE_int64(port, 25001, "Port to receive on.");
DEFINE_double(pub_rate, 0.02, "Network LCM pubishing period (s).");
DEFINE_bool(simulation, false,
    "Simulated or real robot (default=false, real robot)");
DEFINE_bool(test_with_ground_truth_state, false,
    "Get floating base from ground truth state for testing");

// TODO(yminchen): delete the following flag after you finish testing
// cassie_rbt_state_estimator
DEFINE_string(state_channel_name, "CASSIE_STATE_SIMULATION",
    "The name of the lcm channel that sends Cassie's state");

// Cassie model paramter
DEFINE_bool(floating_base, true, "Fixed or floating base model");

// Testing mode
DEFINE_int64(test_mode, -1, "-1: Regular EKF (not testing mode). "
                            "0: both feet always in contact with ground. "
                            "1: both feet never in contact with ground. ");
DEFINE_double(init_imu_height, 0.969223, "The height of imu that we initialize the ekf with");

void setInitialEkfState(const drake::systems::Diagram<double>& diagram,
                        systems::CassieRbtStateEstimator* state_estimator,
                        drake::systems::Context<double>& diagram_context,
                        double t0) {
  auto& state_estimator_context =
      diagram.GetMutableSubsystemContext(*state_estimator, &diagram_context);
  state_estimator->setPreviousTime(&state_estimator_context, t0);
  state_estimator->setInitialImuPosition(
      &state_estimator_context, Eigen::Vector3d(0.0318638, 0, FLAGS_init_imu_height));
  state_estimator->setInitialImuQuaternion(&state_estimator_context,
                                           Eigen::Vector4d(1, 0, 0, 0));
  // Initial imu values are all 0 if the robot is dropped from the air.
  state_estimator->setPreviousImuMeasurement(&state_estimator_context,
                                             Eigen::VectorXd::Zero(6));
}

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  drake::lcm::DrakeLcm lcm_local("udpm://239.255.76.67:7667?ttl=0");
  drake::lcm::DrakeLcm lcm_network("udpm://239.255.76.67:7667?ttl=1");
  DiagramBuilder<double> builder;

  std::unique_ptr<RigidBodyTree<double>> tree;
  if (FLAGS_floating_base) {
    tree = makeCassieTreePointer("examples/Cassie/urdf/cassie_v2.urdf",
                                 drake::multibody::joints::kQuaternion);
    drake::multibody::AddFlatTerrainToWorld(tree.get(), 100, 0.2);
  } else {
    tree = makeCassieTreePointer();
  }

  // Create state estimator
  auto state_estimator = builder.AddSystem<systems::CassieRbtStateEstimator>(
      *tree, FLAGS_floating_base, FLAGS_test_with_ground_truth_state,
      false/*print_info_to_terminal*/, FLAGS_test_mode);

  // Create and connect CassieOutputSender publisher (low-rate for the network)
  // This echoes the messages from the robot
  auto output_sender = builder.AddSystem<systems::CassieOutputSender>();
  auto output_pub = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_cassie_out>("CASSIE_OUTPUT_ECHO",
      &lcm_network, {TriggerType::kPeriodic}, FLAGS_pub_rate));
  // connect cassie_out publisher
  builder.Connect(*output_sender, *output_pub);

  // Connect appropriate input receiver for simulation
  systems::CassieOutputReceiver* input_receiver = nullptr;
  if (FLAGS_simulation) {
    input_receiver =
        builder.AddSystem<systems::CassieOutputReceiver>();
    builder.Connect(*input_receiver, *output_sender);
    builder.Connect(input_receiver->get_output_port(0),
                    state_estimator->get_input_port(0));

    // Adding "CASSIE_STATE_SIMULATION" and "CASSIE_INPUT" ports for testing
    // estimator
    // TODO(yminchen): delete this part after finishing estimator
    if(FLAGS_floating_base && FLAGS_test_with_ground_truth_state){
      auto state_sub = builder.AddSystem(
          LcmSubscriberSystem::Make<dairlib::lcmt_robot_output>(
          FLAGS_state_channel_name, &lcm_local));
      auto state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(*tree);
      builder.Connect(state_sub->get_output_port(),
          state_receiver->get_input_port(0));
      builder.Connect(state_receiver->get_output_port(0),
          state_estimator->get_input_port(1));
    }
  }

  // Create and connect RobotOutput publisher.
  auto robot_output_sender = builder.AddSystem<systems::RobotOutputSender>(*tree,
      true);
  auto state_pub = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_robot_output>(
          "CASSIE_STATE_DISPATCHER", &lcm_local,
          {TriggerType::kForced}));

  // Create and connect RobotOutput publisher (low-rate for the network)
  auto net_state_pub = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_robot_output>(
          "NETWORK_CASSIE_STATE_DISPATCHER", &lcm_network,
          {TriggerType::kPeriodic}, FLAGS_pub_rate));

  // Pass through to drop all but positions and velocities
  auto state_passthrough = builder.AddSystem<systems::SubvectorPassThrough>(
    state_estimator->get_output_port(0).size(),
    0,
    robot_output_sender->get_input_port_state().size());

  // Passthrough to pass efforts
  auto effort_passthrough = builder.AddSystem<systems::SubvectorPassThrough>(
      state_estimator->get_output_port(0).size(),
      robot_output_sender->get_input_port_state().size(),
      robot_output_sender->get_input_port_effort().size());

  builder.Connect(state_estimator->get_output_port(0),
      state_passthrough->get_input_port());
  builder.Connect(state_passthrough->get_output_port(),
      robot_output_sender->get_input_port_state());

  builder.Connect(state_estimator->get_output_port(0),
      effort_passthrough->get_input_port());
  builder.Connect(effort_passthrough->get_output_port(),
      robot_output_sender->get_input_port_effort());

  builder.Connect(*robot_output_sender, *state_pub);

  builder.Connect(*robot_output_sender, *net_state_pub);


  // Create the diagram, simulator, and context.
  auto owned_diagram = builder.Build();
  const auto& diagram = *owned_diagram;
  drake::systems::Simulator<double> simulator(std::move(owned_diagram));
  auto& diagram_context = simulator.get_mutable_context();

  if (FLAGS_simulation) {
    auto& input_receiver_context =
        diagram.GetMutableSubsystemContext(*input_receiver, &diagram_context);

    // Wait for the first message.
    drake::log()->info("Waiting for first lcmt_cassie_out");
    drake::lcm::Subscriber<dairlib::lcmt_cassie_out> input_sub(&lcm_local,
                                                               "CASSIE_OUTPUT");
    LcmHandleSubscriptionsUntil(&lcm_local, [&]() {
      return input_sub.count() > 0; });

    // Initialize the context based on the first message.
    const double t0 = input_sub.message().utime * 1e-6;
    diagram_context.SetTime(t0);
    auto& input_value = input_receiver->get_input_port(0).FixValue(
        &input_receiver_context, input_sub.message());

    // Set EKF previous time
    if (FLAGS_floating_base) {
      setInitialEkfState(diagram, state_estimator, diagram_context, t0);
    }

    drake::log()->info("dispatcher_robot_out started");
    while (true) {
      // Wait for an lcmt_cassie_out message.
      input_sub.clear();
      LcmHandleSubscriptionsUntil(&lcm_local, [&]() {
        return input_sub.count() > 0; });
      // Write the lcmt_robot_input message into the context and advance.
      input_value.GetMutableData()->set_value(input_sub.message());
      const double time = input_sub.message().utime * 1e-6;

      // Check if we are very far ahead or behind
      // (likely due to a restart of the driving clock)
      if (time > simulator.get_context().get_time() + 1.0 ||
          time < simulator.get_context().get_time() - 1.0) {
        std::cout << "Dispatcher time is " << simulator.get_context().get_time()
                  << ", but stepping to " << time << std::endl;
        std::cout << "Difference is too large, resetting dispatcher time." <<
                  std::endl;
        simulator.get_mutable_context().SetTime(time);
      }

      simulator.AdvanceTo(time);
      // Force-publish via the diagram
      diagram.Publish(diagram_context);
    }
  } else {
    auto& output_sender_context =
      diagram.GetMutableSubsystemContext(*output_sender, &diagram_context);
    auto& state_estimator_context =
      diagram.GetMutableSubsystemContext(*state_estimator, &diagram_context);

    // Wait for the first message.
    SimpleCassieUdpSubscriber udp_sub(FLAGS_address, FLAGS_port);
    drake::log()->info("Waiting for first UDP message from Cassie");
    udp_sub.Poll();

    // Initialize the context based on the first message.
    const double t0 = udp_sub.message_time();
    if (FLAGS_floating_base) {
      // Set EKF initial states
      setInitialEkfState(diagram, state_estimator, diagram_context, t0);
    }
    diagram_context.SetTime(t0);
    auto& output_sender_value = output_sender->get_input_port(0).FixValue(
        &output_sender_context, udp_sub.message());
    auto& state_estimator_value = state_estimator->get_input_port(0).FixValue(
        &state_estimator_context, udp_sub.message());
    drake::log()->info("dispatcher_robot_out started");

    while (true) {
      udp_sub.Poll();
      output_sender_value.GetMutableData()->set_value(udp_sub.message());
      state_estimator_value.GetMutableData()->set_value(udp_sub.message());
      const double time = udp_sub.message_time();

      // Check if we are very far ahead or behind
      // (likely due to a restart of the driving clock)
      if (time > simulator.get_context().get_time() + 1.0 ||
          time < simulator.get_context().get_time()) {
        std::cout << "Dispatcher time is " << simulator.get_context().get_time()
            << ", but stepping to " << time << std::endl;
        std::cout << "Difference is too large, resetting dispatcher time." <<
            std::endl;
        simulator.get_mutable_context().SetTime(time);
      }

      simulator.AdvanceTo(time);
      // Force-publish via the diagram
      diagram.Publish(diagram_context);
    }
  }
  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) {
  return dairlib::do_main(argc, argv);
}
