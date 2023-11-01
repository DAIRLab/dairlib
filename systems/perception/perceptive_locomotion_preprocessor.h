#pragma once

#include "elevation_mapping/sensor_processors/StructuredLightSensorProcessor.hpp"

#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/multibody_plant.h"

#include "pcl/filters/crop_box.h"


namespace dairlib {
namespace perception {

struct perceptive_locomotion_preprocessor_params {
  /*
   * Defines a crop box rigidly attached to a frame of the robot.
   */
  struct cropbox_params {
    std::string body_name;
    Eigen::Vector3d length_xyz;
    drake::math::RigidTransformd pose_in_parent_body;
  };
  std::string sensor_params_yaml_;
  std::vector<cropbox_params> crop_boxes_;
};

/// Adapts the StereoSensorProcessor from elevation_mapping to include crop boxes
/// to mask out parts of the robot.
class PerceptiveLocomotionPreprocessor : public elevation_mapping::StructuredLightSensorProcessor {
 public:
  PerceptiveLocomotionPreprocessor(
      const drake::multibody::MultibodyPlant<double>& plant,
      drake::systems::Context<double>* context,
      const perceptive_locomotion_preprocessor_params& params,
      const elevation_mapping::SensorProcessorBase::GeneralParameters& generalParameters);

  // return the context in order to check against another context if needed
  drake::systems::Context<double>* context() {return context_;}
 private:

  bool filterPointCloudSensorType(
      elevation_mapping::PointCloudType::Ptr pointCloud) override;

  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;

  //! Crop box to mask out robot body parts
  std::map<std::string,
    std::pair<drake::math::RigidTransformd,
    pcl::CropBox<pcl::PointXYZRGBConfidenceRatio>>> crop_boxes_;

};

}
}