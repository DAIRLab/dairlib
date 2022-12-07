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

#include "examples/franka_trajectory_following/systems/c3_trajectory_source.h"
#include "examples/franka_trajectory_following/systems/gravity_compensator.h"
#include "systems/robot_lcm_systems.h"
#include "systems/controllers/impedance_controller.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/primitives/timestamped_subvector_pass_through.h"


//#define ROS

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
using Eigen::Vector3d;
using Eigen::MatrixXd;

int DoMain(int argc, char* argv[]){
  drake::lcm::DrakeLcm drake_lcm;

  MultibodyPlant<double> plant(0.0);
  Parser parser(&plant);
  parser.AddModelFromFile("examples/franka_trajectory_following/robot_properties_fingers/urdf/franka_box.urdf");
  parser.AddModelFromFile("examples/franka_trajectory_following/robot_properties_fingers/urdf/sphere.urdf");
  
  /// Fix base of finger to world
  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_link0"), X_WI);
  plant.Finalize();

  DiagramBuilder<double> builder;
  auto state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(plant);

 /* -------------------------------------------------------------------------------------------*/

  /// create trajectory source
  // double time_inc = 5;
  // double num_points = 6;
  // double l = 0.1;

  // std::vector<MatrixXd> points(num_points);
  // std::vector<double> times;
  // points[0] = Vector3d(0.55, 0.0, 0.12);
  // points[1] = Vector3d(0.55, 0.0, 0.12);
  // points[2] = Vector3d(0.55, 0.0+l, 0.12);
  // points[3] = Vector3d(0.55-l, 0.0+l, 0.12);
  // points[4] = Vector3d(0.55-l, 0.0, 0.12);
  // points[5] = Vector3d(0.55, 0.0, 0.12);

  double time_inc = 5;
  double num_points = 2;
  double l = 0.15;

  std::vector<MatrixXd> points(num_points);
  std::vector<double> times;
  points[0] = Vector3d(0.55, 0.0, 0.1237);
  points[1] = Vector3d(0.55, 0.0, 0.1237);

  for (int i = 0; i < num_points; i++){
    times.push_back(i*time_inc);
  }

  auto ee_trajectory = drake::trajectories::PiecewisePolynomial<
    double>::FirstOrderHold(times, points);

  auto input_trajectory = builder.AddSystem<systems::C3TrajectorySource>(
      ee_trajectory);
  builder.Connect(state_receiver->get_output_port(0),
    input_trajectory->get_input_port(0));

  /* -------------------------------------------------------------------------------------------*/

  /// Publish to ROS topic
  ros::init(argc, argv, "impedance_controller");
  ros::NodeHandle node_handle;
  signal(SIGINT, SigintHandler);

  auto subvector_passthrough = builder.AddSystem<systems::TSSubvectorPassThrough<double>>(
    14 + 9 + 6 + 9, 0, 3 + 4);
  auto c3_to_ros = builder.AddSystem<systems::TimestampedVectorToROS>(7);
  auto ros_publisher = builder.AddSystem(
      systems::RosPublisherSystem<std_msgs::Float64MultiArray>::Make("/c3/pose_d", &node_handle, 1 / 80.0));
  builder.Connect(input_trajectory->get_output_port(),
    subvector_passthrough->get_input_port());
  builder.Connect(subvector_passthrough->get_output_port(),
    c3_to_ros->get_input_port());
  builder.Connect(c3_to_ros->get_output_port(), ros_publisher->get_input_port());

 /* -------------------------------------------------------------------------------------------*/

  auto diagram = builder.Build();
  // DrawAndSaveDiagramGraph(*diagram, "examples/franka_trajectory_following/diagram_run_position_experiments");
  auto context_d = diagram->CreateDefaultContext();
  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &drake_lcm, std::move(diagram), state_receiver, "FRANKA_ROS_OUTPUT", true);
  
  loop.Simulate(std::numeric_limits<double>::infinity());

  return 0;
}

} // namespace dairlib

int main(int argc, char* argv[]) { dairlib::DoMain(argc, argv);}