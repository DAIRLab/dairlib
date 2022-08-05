#include <memory>
#include <signal.h>
#include <gflags/gflags.h>
#include <cmath>

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include <drake/lcm/drake_lcm.h>

#include "examples/franka_trajectory_following/systems/c3_state_estimator.h"
#include "examples/franka_trajectory_following/systems/robot_output_passthrough.h"
#include "systems/system_utils.h"
#include "systems/robot_lcm_systems.h"
#include <drake/multibody/tree/multibody_element.h>
#include <drake/multibody/parsing/parser.h>

#include "systems/framework/lcm_driven_loop.h"
#include "examples/franka_trajectory_following/c3_parameters.h"
#include "dairlib/lcmt_robot_output.hpp"

using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;


using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::math::RigidTransform;

DEFINE_string(channel, "FRANKA_OUTPUT",
              "LCM channel for receiving state. "
              "Use FRANKA_OUTPUT to get state from simulator, and "
              "use FRANKA_ROS_OUTPUT to get state from from franka_ros");

DEFINE_int32(TTL, 0,
              "TTL level for publisher. "
              "Default value is 0.");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  C3Parameters param = drake::yaml::LoadYamlFile<C3Parameters>(
    "examples/franka_trajectory_following/parameters.yaml");

  DiagramBuilder<double> builder;
  drake::lcm::DrakeLcm drake_lcm;
  drake::lcm::DrakeLcm drake_lcm_network("udpm://239.255.76.67:7667?ttl=1");

  MultibodyPlant<double> plant(0.0);
  Parser parser(&plant);
  parser.AddModelFromFile("examples/franka_trajectory_following/robot_properties_fingers/urdf/franka_box.urdf");
  parser.AddModelFromFile("examples/franka_trajectory_following/robot_properties_fingers/urdf/sphere.urdf");
  
  /// Fix base of finger to world
  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_link0"), X_WI);
  plant.Finalize();

  /// subscribe to franka state
  auto passthrough = 
    builder.AddSystem<dairlib::systems::RobotOutputPassthrough>(plant);
  
  /// state estimation block
  int p_filter_length = 1;
  int v_filter_length = 2;
  double alpha = 0.9;

  std::vector<double> p_FIR_values = {1};
  std::vector<double> v_FIR_values = {(1-alpha), alpha};

  auto state_estimator = 
    builder.AddSystem<dairlib::systems::C3StateEstimator>(p_FIR_values, v_FIR_values);
  builder.Connect(passthrough->get_output_port(0),
    state_estimator->get_input_port(0));

  /// connect state_susbcriber ball position port
  if (FLAGS_channel == "FRANKA_OUTPUT"){
    /// connections for sim experiment
    auto to_ball_position =
      builder.AddSystem<dairlib::systems::FrankaBallToBallPosition>(
        param.ball_stddev, 1.0/80.0);
    builder.Connect(passthrough->get_output_port(0),
      to_ball_position->get_input_port(0));
    builder.Connect(to_ball_position->get_output_port(0),
      state_estimator->get_input_port(1));
  }
  else if (FLAGS_channel == "FRANKA_ROS_OUTPUT") {
    /// connections for hardware experiment
    auto vision_subscriber = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_ball_position>(
        "VISION_OUTPUT", &drake_lcm));
    std::cout << "vision subscriber waiting for first message" << std::endl;
    vision_subscriber->WaitForMessage(0, nullptr, 1);
    builder.Connect(vision_subscriber->get_output_port(0),
      state_estimator->get_input_port(1));
  }

  drake::lcm::DrakeLcm* pub_lcm;
  if (FLAGS_TTL == 0) {
    pub_lcm = &drake_lcm;
  }
  else if (FLAGS_TTL == 1) {
    pub_lcm = &drake_lcm_network;
  }

  auto sender = builder.AddSystem<dairlib::systems::RobotOutputSender>(plant, true);
  auto robot_output_pub = builder.AddSystem(
    LcmPublisherSystem::Make<dairlib::lcmt_robot_output>(
      "FRANKA_STATE_ESTIMATE", pub_lcm, 
      {drake::systems::TriggerType::kForced}));
  builder.Connect(state_estimator->get_output_port(0), 
    sender->get_input_port(0));
  builder.Connect(state_estimator->get_output_port(1), 
    sender->get_input_port(1));
  builder.Connect(*sender, *robot_output_pub);  


  auto sys = builder.Build();
  auto sys_context = sys->CreateDefaultContext();
  // dairlib::DrawAndSaveDiagramGraph(*sys, "examples/franka_trajectory_following/diagram_run_state_estimator");

  dairlib::systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &drake_lcm, std::move(sys), passthrough, 
      FLAGS_channel, true);
  
  loop.Simulate(std::numeric_limits<double>::infinity());

  return 0;
}

int main(int argc, char* argv[]) {
  return DoMain(argc, argv);
}
