#include "examples/Cassie/cassie_utils.h"
#include "examples/perceptive_locomotion/cassie_perception_utils.h"
#include "systems/perception/pointcloud/point_cloud_conversions.h"

#include "drake/geometry/meshcat.h"
#include "drake/geometry/meshcat_visualizer.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/rendering/multibody_position_to_geometry_pose.h"

namespace dairlib::perceptive_locomotion {

using Eigen::VectorXd;

using drake::geometry::SceneGraph;
using drake::geometry::MeshcatVisualizerd;
using drake::systems::rendering::MultibodyPositionToGeometryPose;

namespace {

void initialize_point_cloud(pcl::PointCloud<pcl::PointXYZRGBConfidenceRatio>::Ptr cloud) {
  float step_size = 0.02f;
  float half_size = 0.5f;
  float z_min = 0.0f;
  float z_max = 1.0f;

  for (float x = -half_size; x <= half_size; x += step_size) {
    for (float y = -half_size; y <= half_size; y += step_size) {
      for (float z = z_min; z <= z_max; z += step_size) {
        pcl::PointXYZRGBConfidenceRatio point;
        point.x = x;
        point.y = y;
        point.z = z;
        cloud->points.push_back(point);
      }
    }
  }

  // Set the width, height, and dense flag
  cloud->width = static_cast<uint32_t>(cloud->points.size());
  cloud->height = 1;  // Unorganized point cloud (1D)
  cloud->is_dense = true;
}
}

int DoMain(int argc, char **argv) {

  const std::string urdf = "examples/Cassie/urdf/cassie_v2.urdf";
  drake::multibody::MultibodyPlant<double> plant(0.0);
  drake::systems::DiagramBuilder<double> builder;

  SceneGraph<double>* scene_graph = builder.AddSystem<SceneGraph>();

  auto instance = AddCassieMultibody(&plant, scene_graph, true, urdf, true, false);
  plant.Finalize();

  auto plant_context = plant.CreateDefaultContext();
  auto preprocessor = MakeCassieElevationMappingPreProcessorForCropBoxTest(
      plant, plant_context.get()
  );

  // visualization
  auto to_pose =
      builder.AddSystem<MultibodyPositionToGeometryPose<double>>(plant);
  builder.Connect(
      to_pose->get_output_port(),
      scene_graph->get_source_pose_port(plant.get_source_id().value()));
  auto meshcat = std::make_shared<drake::geometry::Meshcat>();
  MeshcatVisualizerd::AddToBuilder(&builder, *scene_graph, meshcat);

  VectorXd q = VectorXd::Zero(23);
  q << 1, 0, 0, 0, 0, 0, 0.95, -0.0320918, 0, 0.539399, -1.31373,
      -0.0410844, 1.61932, -0.0301574, -1.67739, 0.0320918, 0, 0.539399,
      -1.31373, -0.0404818, 1.61925, -0.0310551, -1.6785;


  auto diagram = builder.Build();
  auto diagram_context = diagram->CreateDefaultContext();
  auto& to_pose_subcontext = diagram->GetMutableSubsystemContext(*to_pose, diagram_context.get());
  to_pose->get_input_port().FixValue(&to_pose_subcontext, q);

  diagram->ForcedPublish(*diagram_context);

  pcl::PointCloud<pcl::PointXYZRGBConfidenceRatio>::Ptr pointcloud =
      std::make_shared<pcl::PointCloud<pcl::PointXYZRGBConfidenceRatio>>();

  initialize_point_cloud(pointcloud);

  plant.SetPositions(plant_context.get(), q);
  preprocessor->TestFilter(pointcloud);
  drake::perception::PointCloud drake_cloud;

  perception::AssignFields<pcl::PointXYZRGBConfidenceRatio>(pointcloud, drake_cloud);

  meshcat->SetObject("pointcloud", drake_cloud);

  while (true) {
    std::this_thread::sleep_for(std::chrono::microseconds(1000));
  }

  return 0;
}

}

int main(int argc, char** argv) {
  return dairlib::perceptive_locomotion::DoMain(argc, argv);
}