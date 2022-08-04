#include <memory>
#include <signal.h>
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include <drake/lcm/drake_lcm.h>
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"

#include "systems/ros/ros_subscriber_system.h"
#include "systems/ros/ros_publisher_system.h"
#include "systems/ros/c3_ros_conversions.h"
#include "examples/franka_trajectory_following/systems/c3_state_estimator.h"
#include "systems/system_utils.h"
#include "systems/robot_lcm_systems.h"
#include <drake/multibody/tree/multibody_element.h>
#include <drake/multibody/parsing/parser.h>

using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;

using dairlib::systems::RosSubscriberSystem;
using dairlib::systems::RosPublisherSystem;
using dairlib::systems::ROSToRobotOutputLCM;
using dairlib::systems::ROSToBallPositionLCM;

using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::math::RigidTransform;




// Shutdown ROS gracefully and then exit
void SigintHandler(int sig) {
  ros::shutdown();
  exit(sig);
}

int DoMain(ros::NodeHandle& node_handle) {
  DiagramBuilder<double> builder;
  drake::lcm::DrakeLcm drake_lcm;

  MultibodyPlant<double> plant(0.0);
  Parser parser(&plant);
  parser.AddModelFromFile("examples/franka_trajectory_following/robot_properties_fingers/urdf/franka_box.urdf");
  parser.AddModelFromFile("examples/franka_trajectory_following/robot_properties_fingers/urdf/sphere.urdf");
  
  /// Fix base of finger to world
  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_link0"), X_WI);
  plant.Finalize();

  // std::cout << plant.num_positions() << " "
  //   << plant.num_velocities() << " "
  //   << plant.num_actuators() << std::endl;

  /// subscribe to joint states
  auto robot_output_subscriber = 
    builder.AddSystem(RosSubscriberSystem<sensor_msgs::JointState>::Make(
        "/franka_state_controller/joint_states", &node_handle));
  auto to_robot_output = builder.AddSystem(ROSToRobotOutputLCM::Make(14, 13, 7));
  builder.Connect(*robot_output_subscriber, *to_robot_output);

  /// subscribe to ball position
  auto ball_position_subscriber = 
    builder.AddSystem(RosSubscriberSystem<std_msgs::Float64MultiArray>::Make(
        "/c3/position_estimate", &node_handle)); // TODO: confirm topic name
  auto to_ball_position = builder.AddSystem(ROSToBallPositionLCM::Make());
  builder.Connect(*ball_position_subscriber, *to_ball_position);

  /// state estimation block
  int p_filter_length = 10;
  int v_filter_length = 10;
  std::vector<double> p_FIR_values(p_filter_length, 1.0 / p_filter_length);
  std::vector<double> v_FIR_values(v_filter_length, 1.0 / v_filter_length);

  auto state_estimator = 
    builder.AddSystem<dairlib::systems::C3StateEstimator>(p_FIR_values, v_FIR_values);
  builder.Connect(to_robot_output->get_output_port(0), state_estimator->get_input_port(0));
  builder.Connect(to_ball_position->get_output_port(0), state_estimator->get_input_port(1));

  auto sender = builder.AddSystem<dairlib::systems::RobotOutputSender>(plant, true);
  auto robot_output_pub = builder.AddSystem(
    LcmPublisherSystem::Make<dairlib::lcmt_robot_output>(
      "STATE_ESTIMATE", &drake_lcm, 
      {drake::systems::TriggerType::kPeriodic}, 0.0005));
  builder.Connect(state_estimator->get_output_port(0), 
    sender->get_input_port(0));
  builder.Connect(state_estimator->get_output_port(1), 
    sender->get_input_port(1));
  builder.Connect(*sender, *robot_output_pub);  

  auto sys = builder.Build();
  Simulator<double> simulator(*sys);
  // dairlib::DrawAndSaveDiagramGraph(*sys, "examples/franka_trajectory_following/diagram_test_state_estimator");
  simulator.Initialize();
  simulator.set_target_realtime_rate(1.0);

  ros::AsyncSpinner spinner(1);
  spinner.start();
  signal(SIGINT, SigintHandler);

  simulator.AdvanceTo(std::numeric_limits<double>::infinity());

  return 0;
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "test_state_estimation");
  ros::NodeHandle node_handle;

  return DoMain(node_handle);
}
