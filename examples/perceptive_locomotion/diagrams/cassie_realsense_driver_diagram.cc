#include "drake/lcmt_point_cloud.hpp"

#include "cassie_realsense_driver_diagram.h"

#include "examples/Cassie/cassie_utils.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"
#include "systems/perception/feature_tracking/feature_tracker.h"

#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/lcm/lcm_publisher_system.h"

namespace dairlib {
namespace perceptive_locomotion {

using drake::systems::ConstantVectorSource;
using drake::multibody::Frame;

using Eigen::Vector3d;

using perception::ElevationMappingSystem;
using perception::RealsensePointCloudSubscriber;
using perception::RealsenseImagePairSubscriber;
using perception::FeatureTracker;
using perception::elevation_mapping_params_io;
using systems::RobotOutputReceiver;

using drake::systems::lcm::LcmPublisherSystem;
using drake::lcmt_point_cloud;


CassieRealSenseDriverDiagram::CassieRealSenseDriverDiagram(const std::string& params_yaml) {

  const std::string urdf = "examples/Cassie/urdf/cassie_v2.urdf";
  const std::string feat_params = "examples/perceptive_locomotion/feature_tracking_node_params.yaml";

  AddCassieMultibody(&plant_, nullptr, true, urdf, true, false);
  plant_.Finalize();
  plant_context_ = plant_.CreateDefaultContext();

  elevation_mapping_params_ =
      elevation_mapping_params_io::ReadElevationMappingParamsFromYaml(
          params_yaml
      );

  sensor_processor_ = MakeCassieElevationMappingPreProcessor(
      plant_, plant_context_.get()
  );

  drake::systems::DiagramBuilder<double> builder;

  auto state_receiver = builder.AddSystem<RobotOutputReceiver>(plant_);
  elevation_mapping_system_ = builder.AddSystem<ElevationMappingSystem>(
      plant_, plant_context_.get(), elevation_mapping_params_
  );

  point_cloud_subscriber_ = builder.AddSystem<
      RealsensePointCloudSubscriber<pcl::PointXYZRGBConfidenceRatio>>(&realsense_);
   image_pair_subscriber_ =
       builder.AddSystem<RealsenseImagePairSubscriber>(&realsense_);

   auto feature_tracker = builder.AddSystem<FeatureTracker>(feat_params);


  Eigen::MatrixXd base_cov_dummy = 0.1 * Eigen::MatrixXd::Identity(6, 6);
  base_cov_dummy.resize(36,1);
  auto cov_source = builder.AddSystem<ConstantVectorSource<double>>(
      base_cov_dummy
  );

  elevation_mapping_system_->AddSensorPreProcessor(
      "pelvis_depth", sensor_processor_
  );

  builder.Connect(
      point_cloud_subscriber_->get_output_port(),
      elevation_mapping_system_->get_input_port_pointcloud("pelvis_depth")
  );
  builder.Connect(
      state_receiver->get_output_port(),
      elevation_mapping_system_->get_input_port_state()
  );
  builder.Connect(
      cov_source->get_output_port(),
      elevation_mapping_system_->get_input_port_covariance()
  );
  // builder.Connect(*image_pair_subscriber_, *feature_tracker);

  input_port_robot_state_ = builder.ExportInput(
      state_receiver->get_input_port(), "lcmt_robot_output"
  );

  if (elevation_mapping_system_->has_contacts()) {
    input_port_contact_ = builder.ExportInput(
        elevation_mapping_system_->get_input_port_contact(), "lcmt_contact"
    );
  }

  output_port_grid_map_ = builder.ExportOutput(
      elevation_mapping_system_->get_output_port_grid_map(),
      "elevation_grid_map"
  );
   output_port_landmarks_ = builder.ExportOutput(
       feature_tracker->get_output_port(),
       "lcmt_landmark_array"
   );

  builder.BuildInto(this);
  this->set_name("elevation_mapping_ros_diagram");
  DrawAndSaveDiagramGraph(*this);
}

void CassieRealSenseDriverDiagram::InitializeElevationMap(
    const Eigen::VectorXd& robot_state,
    drake::systems::Context<double>* root_context) const {

  auto& context = elevation_mapping_system_->GetMyMutableContextFromRoot(
      root_context
  );
  std::pair<const Vector3d, const Frame<double>&> left_heel = LeftToeRear(plant_);
  std::pair<const Vector3d, const Frame<double>&> right_heel = RightToeRear(plant_);

  elevation_mapping_system_->InitializeFlatTerrain(
      robot_state,
      {left_heel, right_heel},
      context
  );

}

}
}