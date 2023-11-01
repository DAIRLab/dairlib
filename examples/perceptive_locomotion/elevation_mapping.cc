#include "gflags/gflags.h"

#include "dairlib/lcmt_robot_output.hpp"
#include "drake/lcmt_point_cloud.hpp"

#include "examples/Cassie/cassie_utils.h"

#include "systems/robot_lcm_systems.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/perception/grid_map_visualizer.h"
#include "systems/perception/elevation_mapping_system.h"
#include "systems/perception/perceptive_locomotion_preprocessor.h"
#include "systems/perception/lcm_to_pcl_pointcloud.h"
#include "systems/perception/camera_utils.h"
#include "systems/plant_visualizer.h"

#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace dairlib {

using perception::ElevationMappingSystem;
using perception::elevation_mapping_params;
using perception::PerceptiveLocomotionPreprocessor;
using perception::LcmToPclPointCloud;
using perception::perceptive_locomotion_preprocessor_params;
using camera::ReadCameraPoseFromYaml;

using drake::systems::lcm::LcmSubscriberSystem;
using drake::lcmt_point_cloud;

DEFINE_bool(visualize, true, "whether to add visualization");
DEFINE_string(channel_x, "CASSIE_STATE_DISPATCHED", "state lcm channel");
// TODO (@Brian-Acosta) Yaml config with option for multiple input sources
DEFINE_string(channel_point_cloud, "CASSIE_DEPTH", "pointcloud lcm channel");
DEFINE_string(camera_calib_yaml,
              "examples/perceptive_locomotion/camera_calib/cassie_hardware.yaml",
              "camera calibration yaml");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  drake::systems::DiagramBuilder<double> builder;
  drake::lcm::DrakeLcm lcm_local("udpm://239.255.76.67:7667?ttl=0");

  // Set up the plant
  const std::string urdf = "examples/Cassie/urdf/cassie_v2.urdf";
  drake::multibody::MultibodyPlant<double> plant(0.0);
  AddCassieMultibody(&plant, nullptr, true, urdf, true, false);
  plant.Finalize();
  auto plant_context = plant.CreateDefaultContext();

  elevation_mapping_params mapping_params {
      {  // sensor pose params
        {"pelvis_cam", "pelvis", ReadCameraPoseFromYaml(FLAGS_camera_calib_yaml)},
      },
      { // map update params
        drake::systems::TriggerType::kForced
      },
      "pelvis",                       // robot base frame
      0.5 * Eigen::Vector3d::UnitX() // track point (in base frame)
  };

  perceptive_locomotion_preprocessor_params processor_params {
    "examples/perceptive_locomotion/camera_calib/d455_noise_model.yaml",
    {}
  };

  auto elevation_mapping = builder.AddSystem<ElevationMappingSystem>(
      plant, plant_context.get(), mapping_params
  );
  auto processor = std::make_unique<PerceptiveLocomotionPreprocessor>(
      plant, plant_context.get(), processor_params,
      elevation_mapping::SensorProcessorBase::GeneralParameters{"pelvis", "world"}
  );
  elevation_mapping->AddSensorPreProcessor("pelvis_cam", std::move(processor));

  auto state_receiver = builder.AddSystem(
      LcmSubscriberSystem::Make<lcmt_robot_output>(FLAGS_channel_x, &lcm_local)
  );
  auto pcl_subscriber = builder.AddSystem(
      LcmSubscriberSystem::Make<lcmt_point_cloud>(
          FLAGS_channel_point_cloud, &lcm_local
      )
  );
  auto pcl_receiver = builder.AddSystem<
      LcmToPclPointCloud<pcl::PointXYZRGBConfidenceRatio>>();
  if (FLAGS_visualize) {
    // TODO (@Brian-Acosta)
  }

  return 0;
}
}

int main(int argc, char* argv[]) {
  return dairlib::DoMain(argc, argv);
}