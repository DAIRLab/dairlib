#include <vector>
#include <math.h>
#include <gflags/gflags.h>
#include <signal.h>

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include <drake/lcm/drake_lcm.h>
#include <drake/multibody/tree/multibody_element.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/common/trajectories/piecewise_polynomial.h>
#include <drake/math/rigid_transform.h>
#include "drake/math/autodiff.h"
#include "drake/systems/primitives/trajectory_source.h"


#include "systems/robot_lcm_systems.h"
#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_c3.hpp"
#include "multibody/multibody_utils.h"
#include "systems/system_utils.h"
#include "systems/primitives/timestamped_subvector_pass_through.h"
#include "systems/controllers/impedance_controller.h"

#include "examples/franka_ball_rolling/c3_parameters.h"
#include "examples/franka_ball_rolling/systems/c3_trajectory_source.h"
#include "examples/franka_ball_rolling/systems/gravity_compensator.h"
#include "systems/robot_lcm_systems.h"
#include "systems/framework/lcm_driven_loop.h"

#define ROS

#ifdef ROS

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "systems/ros/ros_publisher_system.h"
#include "systems/ros/c3_ros_conversions.h"

void SigintHandler(int sig) {
  ros::shutdown();
  exit(sig);
}

#endif

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
using multibody::makeNameToPositionsMap;
using multibody::makeNameToVelocitiesMap;
using drake::trajectories::PiecewisePolynomial;

using Eigen::VectorXd;
using Eigen::MatrixXd;

int DoMain(int argc, char* argv[]){
  C3Parameters param = drake::yaml::LoadYamlFile<C3Parameters>(
    "examples/franka_ball_rolling/parameters.yaml");
  drake::lcm::DrakeLcm drake_lcm;
  DiagramBuilder<double> builder;

  /* -------------------------------------------------------------------------------------------*/
  /// get trajectory info from c3
  auto c3_subscriber = builder.AddSystem(
    LcmSubscriberSystem::Make<dairlib::lcmt_c3>(
      "CONTROLLER_INPUT", &drake_lcm));
  auto c3_receiver = 
    builder.AddSystem<systems::RobotC3Receiver>(14, 9, 6, 9);
  auto subvector_passthrough = builder.AddSystem<systems::TSSubvectorPassThrough<double>>(
    14 + 9 + 6 + 9, 0, 3 + 4);
  builder.Connect(c3_subscriber->get_output_port(0),
    c3_receiver->get_input_port(0));
  builder.Connect(c3_receiver->get_output_port(0),
    subvector_passthrough->get_input_port());

  /* -------------------------------------------------------------------------------------------*/
  /// Send to ROS
    ros::init(argc, argv, "c3_position_controller");
    ros::NodeHandle node_handle;
    signal(SIGINT, SigintHandler);

    auto c3_to_ros = builder.AddSystem<systems::TimestampedVectorToROS>(7);
    // try making this kForced
    auto ros_publisher = builder.AddSystem(
        systems::RosPublisherSystem<std_msgs::Float64MultiArray>::Make("/c3/pose_d", &node_handle, 1 / 80.0));
    
    builder.Connect(subvector_passthrough->get_output_port(), c3_to_ros->get_input_port());
    builder.Connect(c3_to_ros->get_output_port(), ros_publisher->get_input_port());

  auto diagram = builder.Build();
  auto context_d = diagram->CreateDefaultContext();

  systems::LcmDrivenLoop<dairlib::lcmt_c3> loop(
      &drake_lcm, std::move(diagram), c3_receiver, "CONTROLLER_INPUT", true);
  
  /// initialize message
  std::vector<double> msg_data(34, 0);
  msg_data[0] = param.initial_start(0);
  msg_data[1] = param.initial_start(1);
  msg_data[2] = param.initial_start(2);
  msg_data[3] = 1;
  msg_data[7] = param.traj_radius * sin(M_PI * param.phase / 180.0) + param.x_c;
  msg_data[8] = param.traj_radius * cos(M_PI * param.phase / 180.0) + param.y_c;
  msg_data[9] = param.ball_radius + param.table_offset;
  msg_data[28] = msg_data[7];
  msg_data[29] = msg_data[8];
  msg_data[30] = msg_data[9];
  msg_data[31] = msg_data[7];
  msg_data[32] = msg_data[8];
  msg_data[33] = msg_data[9];

  dairlib::lcmt_c3 init_msg;
  init_msg.data = msg_data;
  init_msg.data_size = 34;
  init_msg.utime = 0.0;

  /// assign initial message
  auto& diagram_context = loop.get_diagram_mutable_context();
  auto& ik_subscriber_context =
      loop.get_diagram()->GetMutableSubsystemContext(*c3_subscriber,
                                                      &diagram_context);
  // Note that currently the LcmSubscriber stores the lcm message in the first
  // state of the leaf system (we hard coded index 0 here)
  auto& mutable_state =
      ik_subscriber_context
          .get_mutable_abstract_state<dairlib::lcmt_c3>(0);
  mutable_state = init_msg;

  loop.Simulate(std::numeric_limits<double>::infinity());

  return 0;
}

} // namespace dairlib

int main(int argc, char* argv[]) { dairlib::DoMain(argc, argv);}
