#include "gflags/gflags.h"

#include "examples/Cassie/cassie_utils.h"

#include "systems/robot_lcm_systems.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/perception/elevation_mapping_system.h"
#include "systems/perception/perceptive_locomotion_preprocessor.h"
#include "systems/perception/camera_utils.h"

#include "drake/systems/framework/diagram_builder.h"

namespace dairlib {

using perception::ElevationMappingSystem;
using perception::elevation_mapping_params;
using perception::PerceptiveLocomotionPreprocessor;
using camera::ReadCameraPoseFromYaml;

DEFINE_bool(visualize, true, "whether to add visualization");
DEFINE_string(channel_x, "CASSIE_STATE_DISPATCHED", "state lcm channel");
DEFINE_string(camera_calib_yaml,
              "examples/perceptive_locomotion/camera_calib/cassie_hardware.yaml",
              "camera calibration yaml");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  drake::systems::DiagramBuilder<double> builder;

  // Set up the plant
  drake::multibody::MultibodyPlant<double> plant(0.0);
  const std::string urdf = "examples/Cassie/urdf/cassie_v2.urdf";
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

  auto elevation_mapping = builder.AddSystem<ElevationMappingSystem>(
      plant, plant_context.get(), mapping_params
  );






  if (FLAGS_visualize) {


  }


}
}

int main(int argc, char* argv[]) {
  return dairlib::DoMain(argc, argv);
}