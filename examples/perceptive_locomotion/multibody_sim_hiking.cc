#include <memory>
#include <signal.h>

#include <drake/systems/primitives/multiplexer.h>
#include <gflags/gflags.h>

#include "dairlib/lcmt_cassie_out.hpp"
#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "drake/lcmt_point_cloud.hpp"

#include "examples/Cassie/cassie_fixed_point_solver.h"
#include "examples/Cassie/cassie_utils.h"
#include "multibody/multibody_utils.h"
#include "multibody/stepping_stone_utils.h"
#include "systems/system_utils.h"
#include "systems/robot_lcm_systems.h"
#include "systems/perception/camera_utils.h"
#include "systems/perception/pointcloud/voxel_grid_filter.h"
#include "systems/framework/geared_motor.h"
#include "systems/primitives/subvector_pass_through.h"


#ifdef DAIR_ROS_ON
#include "systems/ros/ros_publisher_system.h"
#include "systems/ros/robot_state_to_ros_pose.h"
#include "systems/ros/multibody_plant_tf_broadcaster_system.h"
#include "systems/perception/pointcloud/drake_to_ros_pointcloud.h"

void SigintHandler(int sig) {
  ros::shutdown();
  exit(0);
}


#endif

#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/discrete_time_delay.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/sensors/rgbd_sensor.h"
#include "drake/perception/depth_image_to_point_cloud.h"
#include "drake/perception/point_cloud_to_lcm.h"
#include "drake/geometry/render_vtk/render_engine_vtk_params.h"
#include "drake/geometry/render_vtk/factory.h"

namespace dairlib {

#ifdef DAIR_ROS_ON
using camera::DrakeToRosPointCloud;
#endif

using systems::SubvectorPassThrough;
using perception::VoxelGridFilter;
using drake::geometry::SceneGraph;
using drake::multibody::ContactResultsToLcmSystem;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
using drake::systems::sensors::PixelType;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::TriggerTypeSet;
using drake::systems::TriggerType;
using drake::perception::pc_flags::BaseField::kXYZs;
using drake::perception::pc_flags::BaseField::kRGBs;
using drake::perception::DepthImageToPointCloud;
using drake::perception::PointCloudToLcm;

using drake::math::RotationMatrix;
using drake::math::RigidTransformd;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::VectorXd;

// Simulation parameters.
DEFINE_bool(floating_base, true, "Fixed or floating base model");
DEFINE_bool(publish_efforts, true, "Flag to publish the efforts.");
DEFINE_bool(spring_model, true, "Use a URDF with or without legs springs");
DEFINE_bool(publish_ros_pose, false, "if true, publishes the pelvis tf");
DEFINE_bool(publish_points, true, "publish ros pointcloud messages");
DEFINE_bool(time_stepping, true,
            "If 'true', the plant is modeled as a "
            "discrete system with periodic updates. "
            "If 'false', the plant is modeled as a continuous system.");

DEFINE_double(publish_rate, 1000, "Publish rate for simulator");
DEFINE_double(toe_spread, .15, "Initial toe spread in m.");
DEFINE_double(ros_state_pub_period, 0.01, "tf and pose publish period");
DEFINE_double(points_pub_period, 1.0/30.0, "pointcloud publish period");
DEFINE_double(dt, 5e-4,
              "The step size to use for time_stepping, ignored for continuous");
DEFINE_double(end_time, std::numeric_limits<double>::infinity(),
              "End time for simulator");
DEFINE_double(init_height, .95,
              "Initial starting height of the pelvis above "
              "ground");
DEFINE_double(actuator_delay, 0.0,
              "Duration of actuator delay. Set to 0.0 by default.");
DEFINE_double(start_time, 0.0,
              "Starting time of the simulator, useful for initializing the "
              "state at a particular configuration");
DEFINE_double(target_realtime_rate, 1.0,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");
DEFINE_string(radio_channel, "CASSIE_VIRTUAL_RADIO",
              "LCM channel for virtual radio command");
DEFINE_string(channel_u, "CASSIE_INPUT",
              "LCM channel to receive controller inputs on");
DEFINE_string(stepping_stone_filename,
              "examples/perceptive_locomotion/terrains/flat.yaml",
              "YAML file defining stepping stones");
DEFINE_string(camera_calib_yaml,
              "examples/perceptive_locomotion/camera_calib/cassie_hardware.yaml",
              "yaml with camera calib");
DEFINE_string(points_pub_channel, "CASSIE_DEPTH", "depth pointcloud channel");
DEFINE_double(pc_visualization_period, 1.0 / 15.0,
              "period for visualization of the point cloud for debugging");


int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Plant/System initialization
  DiagramBuilder<double> builder;
  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  const double time_step = FLAGS_time_stepping ? FLAGS_dt : 0.0;
  MultibodyPlant<double>& plant = *builder.AddSystem<MultibodyPlant>(time_step);
  multibody::AddSteppingStonesToSimFromYaml(
      &plant, &scene_graph, FLAGS_stepping_stone_filename, 1.0);

  std::string urdf;
  if (FLAGS_spring_model) {
    urdf = "examples/Cassie/urdf/cassie_v2.urdf";
  } else {
    urdf = "examples/Cassie/urdf/cassie_fixed_springs.urdf";
  }

  plant.set_discrete_contact_solver(drake::multibody::DiscreteContactSolver::kSap);
  AddCassieMultibody(&plant, &scene_graph, FLAGS_floating_base, urdf,
                     FLAGS_spring_model, true);
  plant.set_name("plant");

  std::string renderer_name = "pelvis_cam";
  scene_graph.AddRenderer(renderer_name,
                          drake::geometry::MakeRenderEngineVtk(
                          drake::geometry::RenderEngineVtkParams()));

  const auto cam_transform = camera::ReadCameraPoseFromYaml(FLAGS_camera_calib_yaml);

  plant.Finalize();

  auto context = plant.CreateDefaultContext();

  // Create lcm systems.
  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>(
      "udpm://239.255.76.67:7667?ttl=0"
  );
  auto input_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_robot_input>(
          FLAGS_channel_u, lcm));
  auto input_receiver = builder.AddSystem<systems::RobotInputReceiver>(plant);
  auto passthrough = builder.AddSystem<SubvectorPassThrough>(
      input_receiver->get_output_port(0).size(), 0,
      plant.get_actuation_input_port().size());
  auto discrete_time_delay =
      builder.AddSystem<drake::systems::DiscreteTimeDelay>(
          1.0 / FLAGS_publish_rate, FLAGS_actuator_delay * FLAGS_publish_rate,
          plant.num_actuators() + 1);
  auto state_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_output>(
          "CASSIE_STATE_SIMULATION", lcm, 1.0 / FLAGS_publish_rate));
  auto state_sender = builder.AddSystem<systems::RobotOutputSender>(
      plant, FLAGS_publish_efforts);

  // Sensor aggregator and publisher of lcmt_cassie_out
  auto radio_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<lcmt_radio_out>(
          FLAGS_radio_channel, lcm));

  const auto& cassie_motor = AddMotorModel(&builder, plant);

  const auto& sensor_aggregator =
      AddImuAndAggregator(&builder, plant, cassie_motor.get_output_port());

  auto sensor_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_cassie_out>(
          "CASSIE_OUTPUT", lcm, 1.0 / FLAGS_publish_rate));

  // ROS interfaces
#ifdef DAIR_ROS_ON
  ros::init(argc, argv, "cassie_hiking_simulaton");
  ros::NodeHandle node_handle;
  signal(SIGINT, SigintHandler);

  if (FLAGS_publish_ros_pose) {
    const auto& cov_source =
        builder.AddSystem<drake::systems::ConstantVectorSource>(VectorXd::Zero(36));
    const auto& pose_sender =
        builder.AddSystem<systems::RobotStateToRosPose>(
            plant, context.get(), "pelvis");
    const auto& pose_publisher =
        builder.AddSystem<
            systems::RosPublisherSystem<
                geometry_msgs::PoseWithCovarianceStamped>>(
                    "/geometry_msgs/PoseWithCovarianceStamped",
                    &node_handle,
                    TriggerTypeSet({TriggerType::kPeriodic}),
                    FLAGS_ros_state_pub_period);

    std::vector<std::pair<std::string, drake::math::RigidTransformd>> bff;
    bff.push_back({ "camera_depth_optical_frame", cam_transform});
    std::vector<std::string> frames = {"pelvis", "toe_left", "toe_right"};

    const auto& tf_broadcaster =
        builder.AddSystem<systems::MultibodyPlantTfBroadcasterSystem>(
            plant,
            context.get(),
            frames,
            "pelvis",
            "map",
            bff,
            TriggerTypeSet({TriggerType::kPeriodic}),
            FLAGS_ros_state_pub_period);

    builder.Connect(plant.get_state_output_port(),
                    pose_sender->get_input_port_state());
    builder.Connect(plant.get_state_output_port(),
                    tf_broadcaster->get_input_port());
    builder.Connect(cov_source->get_output_port(),
                    pose_sender->get_input_port_covariance());
    builder.Connect(*pose_sender, *pose_publisher);
  }
#endif
  if (FLAGS_publish_points) {
    const auto& [color_camera, depth_camera] = camera::MakeDairD455CameraModel(
        renderer_name, camera::D455ImageSize::k424x240
    );
    const auto intrinsics = depth_camera.core().intrinsics();
    const auto parent_body_id = plant.GetBodyFrameIdIfExists(
            plant.GetFrameByName("pelvis").body().index()
    );
    const auto camera = builder.AddSystem<drake::systems::sensors::RgbdSensor>(
        parent_body_id.value(), cam_transform, color_camera, depth_camera
    );
    const auto depth_to_points = builder.AddSystem<DepthImageToPointCloud>(
        depth_camera.core().intrinsics(), PixelType::kDepth32F, 1.0, kXYZs
    );
    const auto voxel_grid_filter = builder.AddSystem<VoxelGridFilter>(
        0.025, false
    );
    const auto pc_to_lcm = builder.AddSystem<PointCloudToLcm>(renderer_name);
    const auto pc_pub = builder.AddSystem(
        LcmPublisherSystem::Make<drake::lcmt_point_cloud>(
            FLAGS_points_pub_channel, lcm, FLAGS_points_pub_period
    ));
    const auto pc_pub_viz = builder.AddSystem(
        LcmPublisherSystem::Make<drake::lcmt_point_cloud>(
            "DRAKE_POINT_CLOUD", lcm, FLAGS_pc_visualization_period
    ));


    builder.Connect(scene_graph.get_query_output_port(),
                    camera->query_object_input_port());
    builder.Connect(camera->depth_image_32F_output_port(),
                    depth_to_points->depth_image_input_port());
    builder.Connect(*depth_to_points, *voxel_grid_filter);
    builder.Connect(*voxel_grid_filter, *pc_to_lcm);
    builder.Connect(*pc_to_lcm, *pc_pub);
    builder.Connect(*pc_to_lcm, *pc_pub_viz);

#ifdef DAIR_ROS_ON
    const auto points_bridge = builder.AddSystem<DrakeToRosPointCloud>("camera_depth_optical_frame");
    const auto points_pub = builder.AddSystem<
        systems::RosPublisherSystem<sensor_msgs::PointCloud2>>(
        "/camera/depth/color/points",
            &node_handle,
            TriggerTypeSet({TriggerType::kPeriodic}),
            FLAGS_points_pub_period);
    builder.Connect(*depth_to_points, *points_bridge);
    builder.Connect(*points_bridge, *points_pub);
#endif
  }


  // connect leaf systems
  builder.Connect(*input_sub, *input_receiver);
  builder.Connect(input_receiver->get_output_port(),
                  discrete_time_delay->get_input_port());
  builder.Connect(discrete_time_delay->get_output_port(),
                  passthrough->get_input_port());
  builder.Connect(passthrough->get_output_port(),
                  cassie_motor.get_input_port_command());
  builder.Connect(cassie_motor.get_output_port(),
                  plant.get_actuation_input_port());
  builder.Connect(plant.get_state_output_port(),
                  state_sender->get_input_port_state());
  builder.Connect(plant.get_state_output_port(),
                  cassie_motor.get_input_port_state());
  builder.Connect(cassie_motor.get_output_port(),
                  state_sender->get_input_port_effort());
  builder.Connect(*state_sender, *state_pub);
  builder.Connect(
      plant.get_geometry_poses_output_port(),
      scene_graph.get_source_pose_port(plant.get_source_id().value()));
  builder.Connect(scene_graph.get_query_output_port(),
                  plant.get_geometry_query_input_port());
  builder.Connect(radio_sub->get_output_port(),
                  sensor_aggregator.get_input_port_radio());
  builder.Connect(sensor_aggregator.get_output_port(0),
                  sensor_pub->get_input_port());

  auto diagram = builder.Build();
  diagram->set_name(("multibody_sim"));
  //  DrawAndSaveDiagramGraph(*diagram);

  // Create a context for this system:
  std::unique_ptr<Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  diagram_context->EnableCaching();
  diagram->SetDefaultContext(diagram_context.get());
  Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  // Set initial conditions of the simulation
  VectorXd q_init, u_init, lambda_init;
  double mu_fp = 0;
  double min_normal_fp = 70;
  double toe_spread = FLAGS_toe_spread;
  // Create a plant for CassieFixedPointSolver.
  // Note that we cannot use the plant from the above diagram, because after the
  // diagram is built, plant.get_actuation_input_port().HasValue(*context)
  // throws a segfault error
  drake::multibody::MultibodyPlant<double> plant_for_solver(0.0);
  AddCassieMultibody(&plant_for_solver, nullptr,
                     FLAGS_floating_base /*floating base*/, urdf,
                     FLAGS_spring_model, true);
  plant_for_solver.Finalize();
  if (FLAGS_floating_base) {
    CassieFixedPointSolver(plant_for_solver, FLAGS_init_height, mu_fp,
                           min_normal_fp, true, toe_spread, &q_init, &u_init,
                           &lambda_init);
  } else {
    CassieFixedBaseFixedPointSolver(plant_for_solver, &q_init, &u_init,
                                    &lambda_init);
  }
  plant.SetPositions(&plant_context, q_init);
  plant.SetVelocities(&plant_context, VectorXd::Zero(plant.num_velocities()));
  diagram_context->SetTime(FLAGS_start_time);
  Simulator<double> simulator(*diagram, std::move(diagram_context));

  if (!FLAGS_time_stepping) {
    // simulator.get_mutable_integrator()->set_maximum_step_size(0.01);
    // simulator.get_mutable_integrator()->set_target_accuracy(1e-1);
    // simulator.get_mutable_integrator()->set_fixed_step_mode(true);
    simulator.reset_integrator<drake::systems::RungeKutta2Integrator<double>>(
        FLAGS_dt);
  }

  simulator.set_publish_every_time_step(false);
  simulator.set_publish_at_initialization(false);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.AdvanceTo(FLAGS_end_time);

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::do_main(argc, argv); }
