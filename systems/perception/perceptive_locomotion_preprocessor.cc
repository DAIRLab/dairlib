#include "perceptive_locomotion_preprocessor.h"
#include "pcl/filters/passthrough.h"

namespace dairlib {
namespace perception {

using drake::multibody::MultibodyPlant;
using drake::systems::Context;

using elevation_mapping::PointCloudType;
using elevation_mapping::SensorProcessorBase;
using elevation_mapping::StereoSensorProcessor;

PerceptiveLocomotionPreprocessor::PerceptiveLocomotionPreprocessor(
    const MultibodyPlant<double>& plant, Context<double>* context,
    const perceptive_locomotion_preprocessor_params &params,
    const SensorProcessorBase::GeneralParameters& general_params) :
    StereoSensorProcessor(general_params), plant_(plant), context_(context) {

  // Read the sensor params into the super class
  this->readParameters(params.stereo_sensor_params_yaml_);

  // make the crop_boxes
  for (const auto& crop_box_params : params.crop_boxes_) {
    DRAKE_DEMAND(plant_.HasBodyNamed(crop_box_params.body_name));

    pcl::CropBox<pcl::PointXYZRGBConfidenceRatio> crop_box;
    Eigen::Vector4f min_extent = Eigen::Vector4f::UnitW();
    Eigen::Vector4f max_extent = Eigen::Vector4f::UnitW();
    min_extent.head<3>() = -0.5 * crop_box_params.length_xyz.cast<float>();
    max_extent.head<3>() = 0.5 * crop_box_params.length_xyz.cast<float>();
    crop_box.setMin(min_extent);
    crop_box.setMax(max_extent);
    crop_box.setNegative(true); // remove points inside the box
    crop_boxes_.insert(
        {crop_box_params.body_name,
        {crop_box_params.pose_in_parent_body, crop_box}}
    );
  }
}

bool PerceptiveLocomotionPreprocessor::filterPointCloudSensorType(
    PointCloudType::Ptr pointCloud) {
  const Parameters parameters{parameters_.getData()};
  pcl::PassThrough<pcl::PointXYZRGBConfidenceRatio> passThroughFilter;

  // cutoff points outside correct z values
  passThroughFilter.setInputCloud(pointCloud);
  passThroughFilter.setFilterFieldName("z");
  passThroughFilter.setFilterLimits(
    parameters.sensorParameters_.at("cutoff_min_depth"),
    parameters.sensorParameters_.at("cutoff_max_depth")
  );
  passThroughFilter.filter(*pointCloud);

  for (auto& [name, pose_and_crop_box]: crop_boxes_) {
    const auto parent_pose = plant_.EvalBodyPoseInWorld(
        *context_, plant_.GetBodyByName(name)
    );
    // X_WB = X_WP * X_PB
    const auto box_pose = parent_pose * pose_and_crop_box.first;
    auto& crop_box = pose_and_crop_box.second;
    crop_box.setTransform(box_pose.inverse().GetAsIsometry3().cast<float>());
    crop_box.setInputCloud(pointCloud);
    crop_box.filter(*pointCloud);
  }

  return true;
}

}
}