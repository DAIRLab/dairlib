#include <memory>
#include <iostream>
#include <signal.h>
#include <gflags/gflags.h>

#include "dairlib/lcmt_cassie_out.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "examples/Cassie/cassie_state_estimator.h"
#include "examples/Cassie/cassie_utils.h"
#include "examples/Cassie/networking/cassie_output_receiver.h"
#include "examples/Cassie/networking/cassie_output_sender.h"
#include "examples/Cassie/networking/simple_cassie_udp_subscriber.h"
#include "multibody/kinematic/kinematic_evaluator_set.h"
#include "multibody/kinematic/world_point_evaluator.h"
#include "multibody/multibody_solvers.h"
#include "multibody/multibody_utils.h"
#include "systems/perception/camera_utils.h"
#include "systems/framework/output_vector.h"
#include "systems/primitives/subvector_pass_through.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"

#include "drake/common/yaml/yaml_io.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/solvers/choose_best_solver.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/solve.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "examples/Cassie/cassie_state_estimator_settings.h"

#ifdef DAIR_ROS_ON
#include "ros/ros.h"
#include "systems/ros/ros_publisher_system.h"
#include "systems/ros/robot_state_to_ros_pose.h"
#include "systems/ros/multibody_plant_tf_broadcaster_system.h"

void SigintHandler(int sig) {
  ros::shutdown();
  exit(0);
}

#endif

namespace dairlib {

using drake::systems::Context;
using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
using drake::systems::TriggerType;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;

using Eigen::Matrix3d;
using Eigen::Vector3d;

DEFINE_bool(broadcast_robot_state, false, "broadcast to planner thread");

// Simulation parameters.
DEFINE_string(address, "127.0.0.1", "IPv4 address to receive on.");
DEFINE_int64(port, 25001, "Port to receive on.");
DEFINE_double(pub_rate, 0.1, "Network LCM pubishing period (s).");
DEFINE_double(fast_network_pub_rate, 0.005, "Network LCM pubishing period (s).");
DEFINE_bool(simulation, false,
            "Simulated or real robot (default=false, real robot)");
DEFINE_bool(test_with_ground_truth_state, false,
            "Get floating base from ground truth state for testing");
DEFINE_bool(print_ekf_info, false, "Print ekf information to the terminal");
DEFINE_bool(publish_contact, false, "publish contact info to LCM or no");

// TODO(yminchen): delete the flag state_channel_name after finishing testing
// cassie_state_estimator
DEFINE_string(state_channel_name, "CASSIE_STATE_SIMULATION",
              "The name of the lcm channel that sends Cassie's state");

// Cassie model paramter
DEFINE_bool(floating_base, true, "Fixed or floating base model");
DEFINE_string(joint_offset_yaml, "", "yaml with joint offset values");
DEFINE_string(contact_detection_yaml, "examples/Cassie/state_estimator_contact_thresholds.yaml", "Yaml with contact estimation values");

// Testing mode
DEFINE_int64(test_mode, -1,
             "-1: Regular EKF (not testing mode). "
             "0: both feet always in contact with ground. "
             "1: both feet never in contact with ground. "
             "2: both feet always in contact with the ground until contact is"
             " detected in which case it switches to test mode -1.");

// Initial condition (used for simulation)
DEFINE_double(pelvis_x_vel, 0, "external disturbance for testing");
DEFINE_double(pelvis_y_vel, 0, "for stability");


// ROS publishing flags
DEFINE_string(
    camera_pose_calib,
    "examples/perceptive_locomotion/camera_calib/cassie_hardware.yaml",
    "For use with vision sensing - path to yaml file containing the pose of the"
    " realsense base_link in the pelvis frame");
DEFINE_bool(publish_ros, false, "whether to publish state to ROS");
DEFINE_double(ros_pub_period, 0.01, "period to publish to ROS at");

// Run inverse kinematics to get initial pelvis height (assume both feet are
// on the ground), and set the initial state for the EKF.
// Note that we assume the ground is flat in the IK.
void setInitialEkfState(double t0, const cassie_out_t& cassie_output,
                        const drake::multibody::MultibodyPlant<double>& plant,
                        const drake::systems::Diagram<double>& diagram,
                        const systems::CassieStateEstimator& state_estimator,
                        drake::systems::Context<double>* diagram_context) {
  // Copy the joint positions from cassie_out_t to OutputVector
  systems::OutputVector<double> robot_output(
      plant.num_positions(), plant.num_velocities(), plant.num_actuators());
  state_estimator.AssignNonFloatingBaseStateToOutputVector(cassie_output,
                                                           &robot_output);

  multibody::KinematicEvaluatorSet<double> evaluators(plant);
  auto left_toe = LeftToeFront(plant);
  auto left_toe_evaluator = multibody::WorldPointEvaluator(
      plant, left_toe.first, left_toe.second, Eigen::Vector3d(0, 0, 1),
      Eigen::Vector3d::Zero(), false);
  evaluators.add_evaluator(&left_toe_evaluator);
  auto left_heel = LeftToeRear(plant);
  auto left_heel_evaluator = multibody::WorldPointEvaluator(
      plant, left_heel.first, left_heel.second, Eigen::Vector3d(0, 0, 1),
      Eigen::Vector3d::Zero(), false);
  evaluators.add_evaluator(&left_heel_evaluator);
  auto right_toe = RightToeFront(plant);
  auto right_toe_evaluator = multibody::WorldPointEvaluator(
      plant, right_toe.first, right_toe.second, Eigen::Vector3d(0, 0, 1),
      Eigen::Vector3d::Zero(), false);
  evaluators.add_evaluator(&right_toe_evaluator);
  auto right_heel = RightToeRear(plant);
  auto right_heel_evaluator = multibody::WorldPointEvaluator(
      plant, right_heel.first, right_heel.second, Eigen::Vector3d(0, 0, 1),
      Eigen::Vector3d::Zero(), false);
  evaluators.add_evaluator(&right_heel_evaluator);

  auto program = multibody::MultibodyProgram(plant);
  auto q = program.AddPositionVariables();
  auto kinematic_constraint =
      program.AddKinematicPositionConstraint(evaluators, q);

  // Soft constraint on the joint positions
  int n_joints = plant.num_positions() - 7;
  program.AddQuadraticErrorCost(Eigen::MatrixXd::Identity(n_joints, n_joints),
                                robot_output.GetPositions().tail(n_joints),
                                q.tail(n_joints));

  Eigen::VectorXd q_guess(plant.num_positions());
  q_guess << 1, 0, 0, 0, 0, 0, 1, robot_output.GetPositions().tail(n_joints);
  program.SetInitialGuess(q, q_guess);

  std::cout << "Solving inverse kinematics to get initial robot height\n";
  std::cout << "Choose the best solver: "
            << drake::solvers::ChooseBestSolver(program).name() << std::endl;
  auto start = std::chrono::high_resolution_clock::now();
  const auto result = drake::solvers::Solve(program, program.initial_guess());
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  auto q_sol = result.GetSolution(q);
  std::cout << to_string(result.get_solution_result()) << std::endl;
  std::cout << "Solve time:" << elapsed.count() << std::endl;
  std::cout << "Cost:" << result.get_optimal_cost() << std::endl;
  std::cout << "q sol = " << q_sol.transpose() << "\n\n";

  // Pelvis vel
  Vector3d pelvis_vel(FLAGS_pelvis_x_vel, FLAGS_pelvis_y_vel, 0);

  // Set initial time and floating base position
  auto& state_estimator_context =
      diagram.GetMutableSubsystemContext(state_estimator, diagram_context);
  state_estimator.setPreviousTime(&state_estimator_context, t0);
  state_estimator.setInitialPelvisPose(&state_estimator_context, q_sol.head(4),
                                       q_sol.segment<3>(4), pelvis_vel);
  // Set initial imu value
  // Note that initial imu values are all 0 if the robot is dropped from the air
  Eigen::VectorXd init_prev_imu_value = Eigen::VectorXd::Zero(6);
  init_prev_imu_value << 0, 0, 0, 0, 0, 9.81;
  state_estimator.setPreviousImuMeasurement(&state_estimator_context,
                                            init_prev_imu_value);
}

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  DiagramBuilder<double> builder;
  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>("udpm://239.255.76.67:7667?ttl=0");
  drake::lcm::DrakeLcm lcm_network("udpm://239.255.76.67:7667?ttl=1");

  // Build Cassie MBP
  drake::multibody::MultibodyPlant<double> plant(0.0);
  AddCassieMultibody(&plant, nullptr, FLAGS_floating_base /*floating base*/,
                     "examples/Cassie/urdf/cassie_v2.urdf",
                     true /*spring model*/, false /*loop closure*/);
  plant.Finalize();
  auto plant_context = plant.CreateDefaultContext();

  // Evaluators for fourbar linkages
  multibody::KinematicEvaluatorSet<double> fourbar_evaluator(plant);
  auto left_loop = LeftLoopClosureEvaluator(plant);
  auto right_loop = RightLoopClosureEvaluator(plant);
  fourbar_evaluator.add_evaluator(&left_loop);
  fourbar_evaluator.add_evaluator(&right_loop);
  // Evaluators for contact points (The position doesn't matter. It's not used
  // in OSC)
  multibody::KinematicEvaluatorSet<double> left_contact_evaluator(plant);
  auto left_toe = LeftToeFront(plant);
  auto left_heel = LeftToeRear(plant);
  auto left_toe_evaluator = multibody::WorldPointEvaluator(
      plant, left_toe.first, left_toe.second, Matrix3d::Identity(),
      Vector3d::Zero(), {1, 2});
  auto left_heel_evaluator = multibody::WorldPointEvaluator(
      plant, left_heel.first, left_heel.second, Matrix3d::Identity(),
      Vector3d::Zero(), {0, 1, 2});
  left_contact_evaluator.add_evaluator(&left_toe_evaluator);
  left_contact_evaluator.add_evaluator(&left_heel_evaluator);
  multibody::KinematicEvaluatorSet<double> right_contact_evaluator(plant);
  auto right_toe = RightToeFront(plant);
  auto right_heel = RightToeRear(plant);
  auto right_toe_evaluator = multibody::WorldPointEvaluator(
      plant, right_toe.first, right_toe.second, Matrix3d::Identity(),
      Vector3d::Zero(), {1, 2});
  auto right_heel_evaluator = multibody::WorldPointEvaluator(
      plant, right_heel.first, right_heel.second, Matrix3d::Identity(),
      Vector3d::Zero(), {0, 1, 2});
  right_contact_evaluator.add_evaluator(&right_toe_evaluator);
  right_contact_evaluator.add_evaluator(&right_heel_evaluator);

  // Create state estimator
  const auto joint_offset_map =
      (FLAGS_joint_offset_yaml.empty()) ?
      std::map<std::string, double>{} :
      drake::yaml::LoadYamlFile<std::map<std::string, double>>(
          FindResourceOrThrow(FLAGS_joint_offset_yaml));

  CassieStateEstimatorContactThresholds settings = drake::yaml::LoadYamlFile<CassieStateEstimatorContactThresholds>(
      FindResourceOrThrow(FLAGS_contact_detection_yaml));

  auto state_estimator = builder.AddSystem<systems::CassieStateEstimator>(
      plant, &fourbar_evaluator, &left_contact_evaluator,
      &right_contact_evaluator, joint_offset_map,
      FLAGS_test_with_ground_truth_state, FLAGS_print_ekf_info,
      FLAGS_test_mode);

  state_estimator->SetSpringDeflectionThresholds(settings.knee_spring_threshold,
                                                 settings.ankle_spring_threshold);
  state_estimator->SetContactForceThreshold(settings.contact_force_threshold);
  // Create and connect CassieOutputSender publisher (low-rate for the network)
  // This echoes the messages from the robot
  auto output_sender = builder.AddSystem<systems::CassieOutputSender>();
  auto output_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_cassie_out>(
          "CASSIE_OUTPUT_ECHO", &lcm_network, {TriggerType::kPeriodic},
          FLAGS_pub_rate));
  // connect cassie_out publisher
  builder.Connect(*output_sender, *output_pub);

  // Connect appropriate input receiver for simulation
  systems::CassieOutputReceiver* input_receiver = nullptr;
  if (FLAGS_simulation) {
    input_receiver = builder.AddSystem<systems::CassieOutputReceiver>();
    builder.Connect(*input_receiver, *output_sender);
    builder.Connect(input_receiver->get_output_port(0),
                    state_estimator->get_input_port(0));

    // Adding "CASSIE_STATE_SIMULATION" and "CASSIE_INPUT" ports for testing
    // estimator
    // TODO(yminchen): delete this part after finishing estimator
    if (FLAGS_floating_base && FLAGS_test_with_ground_truth_state) {
      auto state_sub = builder.AddSystem(
          LcmSubscriberSystem::Make<dairlib::lcmt_robot_output>(
              FLAGS_state_channel_name, lcm));
      auto state_receiver =
          builder.AddSystem<systems::RobotOutputReceiver>(plant);
      builder.Connect(state_sub->get_output_port(),
                      state_receiver->get_input_port(0));
      builder.Connect(state_receiver->get_output_port(0),
                      state_estimator->get_input_port(1));
    }
  }

  // Create and connect RobotOutput publisher.
  auto robot_output_sender =
      builder.AddSystem<systems::RobotOutputSender>(plant, true, true);

  if (FLAGS_floating_base && FLAGS_publish_contact) {
    // Create and connect contact estimation publisher.
    auto contact_pub =
        builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_contact>(
            "CASSIE_CONTACT_DISPATCHER", lcm, {TriggerType::kForced}));
    builder.Connect(state_estimator->get_contact_output_port(),
                    contact_pub->get_input_port());
    // TODO(yangwill): Consider filtering contact estimation
    auto gm_contact_pub = builder.AddSystem(
        LcmPublisherSystem::Make<drake::lcmt_contact_results_for_viz>(
            "CASSIE_GM_CONTACT_DISPATCHER", lcm, {TriggerType::kForced}));
    builder.Connect(state_estimator->get_gm_contact_output_port(),
                    gm_contact_pub->get_input_port());
  }

  // Pass through to drop all but positions and velocities
  auto state_passthrough = builder.AddSystem<systems::SubvectorPassThrough>(
      state_estimator->get_robot_output_port().size(), 0,
      robot_output_sender->get_input_port_state().size());

  // Passthrough to pass efforts
  auto effort_passthrough = builder.AddSystem<systems::SubvectorPassThrough>(
      state_estimator->get_robot_output_port().size(),
      robot_output_sender->get_input_port_state().size(),
      robot_output_sender->get_input_port_effort().size());

  auto imu_passthrough = builder.AddSystem<systems::SubvectorPassThrough>(
      state_estimator->get_robot_output_port().size(),
      robot_output_sender->get_input_port_state().size() +
          robot_output_sender->get_input_port_effort().size(),
      robot_output_sender->get_input_port_imu().size());

  builder.Connect(state_estimator->get_robot_output_port(),
                  state_passthrough->get_input_port());
  builder.Connect(state_passthrough->get_output_port(),
                  robot_output_sender->get_input_port_state());

  builder.Connect(state_estimator->get_robot_output_port(),
                  effort_passthrough->get_input_port());
  builder.Connect(effort_passthrough->get_output_port(),
                  robot_output_sender->get_input_port_effort());

  builder.Connect(state_estimator->get_robot_output_port(),
                  imu_passthrough->get_input_port());
  builder.Connect(imu_passthrough->get_output_port(),
                  robot_output_sender->get_input_port_imu());


#ifdef DAIR_ROS_ON
  if (FLAGS_publish_ros){
    DRAKE_ASSERT(FLAGS_ros_pub_period > 0);
    ros::init(argc, argv, "dispatcher_robot_out");
    ros::NodeHandle node_handle;
    signal(SIGINT, SigintHandler);

    const auto& pose_sender = builder.AddSystem<systems::RobotStateToRosPose>(
            plant, plant_context.get(), "pelvis");
    const auto& pose_publisher = builder.AddSystem<
            systems::RosPublisherSystem<geometry_msgs::PoseWithCovarianceStamped>>
            ("/geometry_msgs/PoseWithCovarianceStamped",
             &node_handle,
             drake::systems::TriggerTypeSet(
                 {drake::systems::TriggerType::kPeriodic}),
             FLAGS_ros_pub_period);

    std::vector<std::pair<std::string, drake::math::RigidTransformd>> bff;
    bff.push_back({
      "camera_depth_optical_frame",
      camera::ReadCameraPoseFromYaml(FLAGS_camera_pose_calib)
    });
    bff.push_back({
      "map_center",
      drake::math::RigidTransformd(drake::math::RotationMatrixd(),
                                   0.5 * Vector3d::UnitX())
    });
    std::vector<std::string> frames = {"pelvis", "toe_left", "toe_right"};

    const auto& tf_broadcaster =
        builder.AddSystem<systems::MultibodyPlantTfBroadcasterSystem>(
            plant,
            plant_context.get(),
            frames,
            "pelvis",
            "map",
            bff,
            drake::systems::TriggerTypeSet(
                {drake::systems::TriggerType::kPeriodic}),
            FLAGS_ros_pub_period);

    builder.Connect(state_passthrough->get_output_port(),
                    pose_sender->get_input_port_state());
    builder.Connect(state_passthrough->get_output_port(),
                    tf_broadcaster->get_input_port());
    builder.Connect(state_estimator->get_covariance_output_port(),
                    pose_sender->get_input_port_covariance());
    builder.Connect(*pose_sender, *pose_publisher);
  }
#endif

  if (FLAGS_broadcast_robot_state) {
    auto state_pub =
        builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_output>(
            "CASSIE_STATE_DISPATCHER", lcm, {TriggerType::kForced}));
    builder.Connect(*robot_output_sender, *state_pub);
    auto net_state_pub =
        builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_output>(
            "NETWORK_CASSIE_STATE_DISPATCHER", &lcm_network,
            {TriggerType::kPeriodic}, FLAGS_fast_network_pub_rate));
    builder.Connect(*robot_output_sender, *net_state_pub);

  } else {
    auto state_pub =
        builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_output>(
            "CASSIE_STATE_DISPATCHER", lcm, {TriggerType::kForced}));
    builder.Connect(*robot_output_sender, *state_pub);

    // Create and connect RobotOutput publisher (low-rate for the network)
    auto net_state_pub =
        builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_output>(
            "NETWORK_CASSIE_STATE_DISPATCHER", &lcm_network,
            {TriggerType::kPeriodic}, FLAGS_pub_rate));

    builder.Connect(*robot_output_sender, *net_state_pub);
  }

  // Create the diagram, simulator, and context.
  auto owned_diagram = builder.Build();
  owned_diagram->set_name(("dispatcher_robot_out"));
  const auto& diagram = *owned_diagram;
//  DrawAndSaveDiagramGraph(diagram);
  drake::systems::Simulator<double> simulator(std::move(owned_diagram));
  auto& diagram_context = simulator.get_mutable_context();

  if (FLAGS_simulation) {
    auto& input_receiver_context =
        diagram.GetMutableSubsystemContext(*input_receiver, &diagram_context);

    // Wait for the first message.
    drake::log()->info("Waiting for first lcmt_cassie_out");
    drake::lcm::Subscriber<dairlib::lcmt_cassie_out> input_sub(lcm,
                                                               "CASSIE_OUTPUT");
    LcmHandleSubscriptionsUntil(lcm, [&]() { return input_sub.count() > 0; });

    // Initialize the context based on the first message.
    const double t0 = input_sub.message().utime * 1e-6;
    diagram_context.SetTime(t0);
    auto& input_value = input_receiver->get_input_port(0).FixValue(
        &input_receiver_context, input_sub.message());

    // Set EKF time and initial states
    if (FLAGS_floating_base) {
      // Read cassie_out_t from the output port of CassieOutputReceiver()
      const cassie_out_t& simulated_message =
          input_receiver->get_output_port(0).Eval<cassie_out_t>(
              input_receiver_context);

      setInitialEkfState(t0, simulated_message, plant, diagram,
                         *state_estimator, &diagram_context);
    }

    double prev_time = -1;
    drake::log()->info("dispatcher_robot_out started");
    while (true) {
      // Wait for an lcmt_cassie_out message.
      input_sub.clear();
      LcmHandleSubscriptionsUntil(lcm, [&]() { return input_sub.count() > 0; });
      // Write the lcmt_robot_input message into the context and advance.
      input_value.GetMutableData()->set_value(input_sub.message());
      const double time = input_sub.message().utime * 1e-6;

      // Hacks -- for some reason, sometimes the lcm from Mujoco is not in order
      if (prev_time > time) {
        std::cout << time << std::endl;
        continue;
      }

      // Check if we are very far ahead or behind
      // (likely due to a restart of the driving clock)
      if (time > simulator.get_context().get_time() + 1.0 ||
          time < simulator.get_context().get_time() - 1.0) {
        std::cout << "Dispatcher time is " << simulator.get_context().get_time()
                  << ", but stepping to " << time << std::endl;
        std::cout << "Difference is too large, resetting dispatcher time."
                  << std::endl;
        simulator.get_mutable_context().SetTime(time);
        simulator.Initialize();
      }

      state_estimator->set_next_message_time(time);

      simulator.AdvanceTo(time);
      // Force-publish via the diagram
      diagram.ForcedPublish(diagram_context);

      prev_time = time;
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
      // Set EKF time and initial states
      setInitialEkfState(t0, udp_sub.message(), plant, diagram,
                         *state_estimator, &diagram_context);
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
        std::cout << "Difference is too large, resetting dispatcher time."
                  << std::endl;
        simulator.get_mutable_context().SetTime(time);
      }

      state_estimator->set_next_message_time(time);

      simulator.AdvanceTo(time);
      // Force-publish via the diagram
      diagram.ForcedPublish(diagram_context);
    }
  }
  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::do_main(argc, argv); }
