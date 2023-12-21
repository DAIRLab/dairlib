#include "gflags/gflags.h"

// Need to include ros stuff before any pcl dependencies to avoid
// boost errors (pcl can use ros's copy of the boost headers,
// but not the other way around)
#ifdef DAIR_ROS_ON
#include <signal.h>
// ros deps
#include "systems/ros/ros_subscriber_system.h"
#include "sensor_msgs/PointCloud2.h"
#include "systems/perception/pointcloud/ros_point_cloud2_receiver.h"
void SigintHandler(int sig) {
  ros::shutdown();
  exit(0);
}
#endif

#include "dairlib/lcmt_robot_output.hpp"
#include "drake/lcmt_point_cloud.hpp"

#include "examples/Cassie/cassie_utils.h"

#include "systems/robot_lcm_systems.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/perception/grid_map_visualizer.h"
#include "systems/perception/elevation_mapping_system.h"
#include "systems/perception/perceptive_locomotion_preprocessor.h"
#include "systems/perception/pointcloud/lcm_to_pcl_point_cloud.h"
#include "systems/perception/camera_utils.h"
#include "systems/plant_visualizer.h"
#include "systems/system_utils.h"

#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace dairlib {

using Eigen::Vector3d;

using systems::RobotOutputReceiver;
using perception::ElevationMappingSystem;
using perception::elevation_mapping_params;
using perception::elevation_mapping_params_io;
using perception::PerceptiveLocomotionPreprocessor;
using perception::LcmToPclPointCloud;
using perception::perceptive_locomotion_preprocessor_params;
using camera::ReadCameraPoseFromYaml;

using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::ConstantVectorSource;
using drake::lcmt_point_cloud;

DEFINE_bool(visualize, true, "whether to add visualization");
DEFINE_string(channel_x, "CASSIE_STATE_DISPATCHER", "state lcm channel");
// TODO (@Brian-Acosta) Yaml config with option for multiple input sources
DEFINE_string(channel_point_cloud, "CASSIE_DEPTH", "pointcloud lcm channel");
DEFINE_string(camera_calib_yaml,
              "examples/perceptive_locomotion/camera_calib/cassie_hardware.yaml",
              "camera calibration yaml");
DEFINE_string(elevation_mapping_params_yaml,
              "examples/perceptive_locomotion/camera_calib/"
              "elevation_mapping_params_simulation.yaml",
              "elevation mapping parameters file");

DEFINE_bool(get_points_from_ros, false,
            "Get point clouds from ros instead of lcm");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

#ifndef DAIR_ROS_ON
  DRAKE_DEMAND(FLAGS_get_points_from_ros == false);
#else
  ros::init(argc, argv, "elevation_mapping");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
#endif

  drake::systems::DiagramBuilder<double> builder;
  drake::lcm::DrakeLcm lcm_local("udpm://239.255.76.67:7667?ttl=0");

  // Set up the plant
  const std::string urdf = "examples/Cassie/urdf/cassie_v2.urdf";
  drake::multibody::MultibodyPlant<double> plant(0.0);
  AddCassieMultibody(&plant, nullptr, true, urdf, true, false);
  plant.Finalize();
  auto plant_context = plant.CreateDefaultContext();

  elevation_mapping_params mapping_params =
      elevation_mapping_params_io::ReadElevationMappingParamsFromYaml(
          FLAGS_elevation_mapping_params_yaml
      );

  perceptive_locomotion_preprocessor_params processor_params {
      "examples/perceptive_locomotion/camera_calib/d455_noise_model.yaml",
      {
          {"toe_left", Vector3d(0.3, 0.1, 0.2),
           CassieTransformFootToToeFrame()},
          {"tarsus_left", Vector3d(0.5, 0.2, 0.2),
           drake::math::RigidTransformd(Vector3d(0.204, -0.02, 0))},
          {"toe_right", Vector3d(0.3, 0.1, 0.2),
           CassieTransformFootToToeFrame()},
          {"tarsus_right", Vector3d(0.5, 0.2, 0.2),
           drake::math::RigidTransformd(Vector3d(0.204, -0.02, 0))},
      } // crop boxes
  };

  auto elevation_mapping = builder.AddSystem<ElevationMappingSystem>(
      plant, plant_context.get(), mapping_params
  );
  auto processor = std::make_shared<PerceptiveLocomotionPreprocessor>(
      plant, plant_context.get(), processor_params,
      elevation_mapping::SensorProcessorBase::GeneralParameters{"pelvis", "world"}
  );
  elevation_mapping->AddSensorPreProcessor("pelvis_depth", std::move(processor));

  auto state_receiver = builder.AddSystem<RobotOutputReceiver>(plant);

  drake::systems::LeafSystem<double>* pcl_subscriber;
  drake::systems::LeafSystem<double>* pcl_receiver;


  if (FLAGS_get_points_from_ros) {
#ifdef DAIR_ROS_ON
    signal(SIGINT, SigintHandler);

    pcl_subscriber = builder.AddSystem<systems::RosSubscriberSystem<
        sensor_msgs::PointCloud2>>(FLAGS_channel_point_cloud, &node_handle);

    pcl_receiver = builder.AddSystem<perception::RosPointCloud2Receiver
        <pcl::PointXYZRGBConfidenceRatio>>();
#endif
  } else {
    pcl_subscriber = builder.AddSystem(
        LcmSubscriberSystem::Make<lcmt_point_cloud>(
            FLAGS_channel_point_cloud, &lcm_local
        )
    );
    pcl_receiver = builder.AddSystem<
        LcmToPclPointCloud<pcl::PointXYZRGBConfidenceRatio>>();
  }

  Eigen::MatrixXd base_cov_dummy = 0.1 * Eigen::MatrixXd::Identity(6, 6);
  base_cov_dummy.resize(36,1);
  auto cov_source = builder.AddSystem<ConstantVectorSource<double>>(
      base_cov_dummy
  );

  builder.Connect(*pcl_subscriber, *pcl_receiver);
  builder.Connect(
      pcl_receiver->get_output_port(),
      elevation_mapping->get_input_port_pointcloud("pelvis_depth")
  );
  builder.Connect(
      state_receiver->get_output_port(),
      elevation_mapping->get_input_port_state()
  );
  builder.Connect(
      cov_source->get_output_port(),
      elevation_mapping->get_input_port_covariance()
  );

  if (FLAGS_visualize) {
    std::vector<std::string> layers_to_visualize = {"elevation"};
    auto visualizer = builder.AddSystem<systems::PlantVisualizer>(urdf);
    auto grid_map_visualizer = builder.AddSystem<perception::GridMapVisualizer>(
        visualizer->get_meshcat(), 30.0, layers_to_visualize
    );
    builder.Connect(
        state_receiver->get_output_port(), visualizer->get_input_port()
    );
    builder.Connect(
        elevation_mapping->get_output_port_grid_map(),
        grid_map_visualizer->get_input_port()
    );
  }

  auto diagram = builder.Build();
  DrawAndSaveDiagramGraph(*diagram, "../elevation_mapping_diagram");
  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm_local, std::move(diagram), state_receiver, FLAGS_channel_x, true
  );

#ifdef DAIR_ROS_ON
  if (FLAGS_get_points_from_ros) {
    spinner.start();
  }
#endif
  loop.Simulate();
  return 0;
}
}

int main(int argc, char* argv[]) {
  return dairlib::DoMain(argc, argv);
}