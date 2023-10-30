#define ROS

#ifdef ROS

#include <memory>
#include <signal.h>
#include <string>
#include <gflags/gflags.h>

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include <drake/lcm/drake_lcm.h>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"

#include "systems/ros/ros_subscriber_system.h"
#include "systems/ros/ros_publisher_system.h"
#include "systems/ros/c3_ros_conversions.h"
#include "systems/system_utils.h"

using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;

using dairlib::systems::RosSubscriberSystem;
using dairlib::systems::RosPublisherSystem;
using dairlib::systems::ROSToRobotOutputLCM;
using dairlib::systems::ROSToC3LCM;
using dairlib::systems::ROSToBallPositionLCM;

// Shutdown ROS gracefully and then exit
void SigintHandler(int sig) {
  ros::shutdown();
  exit(sig);
}

DEFINE_string(joint_channel, "/c3/joint_states",
              "Rostopic for receiving joint state info from Franka.  "
              "Use /c3/joint_states for actual experiments (1000hz channel), and "
              "use /franka_state_controller/joint_states for debugging (100hz channel)."
              "Note that the default rate of the franka_control_interface is 30hz, "
              "but this rate has been overridden to be 100hz on the franka computer");

namespace dairlib {

int DoMain(int argc, char* argv[]){
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  ros::init(argc, argv, "run_c3_ros_to_lcm");
  ros::NodeHandle node_handle;

  drake::lcm::DrakeLcm drake_lcm;
  DiagramBuilder<double> builder;

  MultibodyPlant<double> plant(0.0);

  Parser parser(&plant, &scene_graph);
  drake::multibody::ModelInstanceIndex franka_index =
      parser.AddModels(drake::FindResourceOrThrow(sim_params.franka_model))[0];

  /* -------------------------------------------------------------------------------------------*/
  /// convert franka joint states to lcm
  auto franka_joint_state_ros_subscriber =
      builder.AddSystem(RosSubscriberSystem<sensor_msgs::JointState>::Make(
          FLAGS_joint_channel, &node_handle));
  auto ros_to_lcm_robot_state = builder.AddSystem(RosToLcmRobotState::Make(7, 7, 7));

  // change this to output correctly (i.e. when ros subscriber gets new message)
  auto robot_output_pub = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_robot_output>(
          "FRANKA_STATE", &drake_lcm,
          {drake::systems::TriggerType::kForced}));
  /// connections
  builder.Connect(franka_joint_state_ros_subscriber->get_output_port(), ros_to_lcm_robot_state->get_input_port());
  builder.Connect(ros_to_lcm_robot_state->get_output_port(), robot_output_pub->get_input_port());

  /* -------------------------------------------------------------------------------------------*/
  /// convert ball position estimate to lcm
  auto tray_object_state_ros_subscriber =
      builder.AddSystem(RosSubscriberSystem<std_msgs::Float64MultiArray>::Make(
          "/c3/position_estimate", &node_handle));
  auto ros_to_lcm_object_state = builder.AddSystem(RosToLcmObjectState::Make());

  // change this to output correctly (i.e. when ros subscriber gets new message)
  auto ball_position_pub = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_ball_position>(
          "VISION_OUTPUT", &drake_lcm,
          {drake::systems::TriggerType::kPeriodic}, 0.005));
  /// connections
  builder.Connect(tray_object_state_ros_subscriber->get_output_port(), ros_to_lcm_object_state->get_input_port());
  builder.Connect(ros_to_lcm_object_state->get_output_port(), ball_position_pub->get_input_port());

  /* -------------------------------------------------------------------------------------------*/
  /// convert individual camera measurements to lcm for logging
  int num_cameras = 3;
  std::vector<RosSubscriberSystem<std_msgs::Float64MultiArray>*>
      cam_subscribers(num_cameras, nullptr);
  std::vector<ROSToBallPositionLCM*> ros_lcm_converters(num_cameras, nullptr);
  std::vector<LcmPublisherSystem*> cam_publishers(num_cameras, nullptr);

  for (int i = 0; i < num_cameras; i++){
    // Add systems
    std::string cam_id = std::to_string(i);
    cam_subscribers[i] =
        builder.AddSystem(RosSubscriberSystem<std_msgs::Float64MultiArray>::Make(
            "/c3/cam" + cam_id, &node_handle));
    ros_lcm_converters[i] =
        builder.AddSystem(ROSToBallPositionLCM::Make());
    cam_publishers[i] =
        builder.AddSystem(
            LcmPublisherSystem::Make<dairlib::lcmt_ball_position>(
                "CAM" + cam_id + "_OUTPUT", &drake_lcm,
                {drake::systems::TriggerType::kPeriodic}, 0.005));

    // Connect systems
    builder.Connect(cam_subscribers[i]->get_output_port(),
                    ros_lcm_converters[i]->get_input_port());
    builder.Connect(ros_lcm_converters[i]->get_output_port(),
                    cam_publishers[i]->get_input_port());
  }

  /* -------------------------------------------------------------------------------------------*/

  auto sys = builder.Build();
  // DrawAndSaveDiagramGraph(*sys, "examples/franka_trajectory_following/diagram_run_c3_ros_to_lcm");

  Simulator<double> simulator(*sys);
  simulator.Initialize();
  simulator.set_target_realtime_rate(1.0);

  // figure out what the arguments to this mean
  ros::AsyncSpinner spinner(1);
  spinner.start();
  signal(SIGINT, SigintHandler);
  simulator.AdvanceTo(std::numeric_limits<double>::infinity());

  return 0;
}

} // namespace dairlib

int main(int argc, char* argv[]) { dairlib::DoMain(argc, argv);}

#endif