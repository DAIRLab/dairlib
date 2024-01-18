#include "cassie_perception_utils.h"
#include "examples/Cassie/cassie_utils.h"

namespace dairlib {
namespace perceptive_locomotion {

using Eigen::Vector3d;

std::shared_ptr<perception::PerceptiveLocomotionPreprocessor>
MakeCassieElevationMappingPreProcessor(
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* plant_context) {
  perception::perceptive_locomotion_preprocessor_params processor_params {
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
      }
  };
  processor_params.min_y = -0.15;
  processor_params.min_z = 0.7;
  processor_params.max_z = 3.0;

  return std::make_shared<perception::PerceptiveLocomotionPreprocessor>(
      plant, plant_context, processor_params,
      elevation_mapping::SensorProcessorBase::GeneralParameters{"pelvis", "world"}
  );
}

}
}

