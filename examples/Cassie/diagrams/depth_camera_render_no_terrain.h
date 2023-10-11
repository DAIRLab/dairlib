#pragma once

#include <memory>
#include <utility>

#include <drake/multibody/plant/multibody_plant.h>
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/system.h"
#include "systems/perception/camera_utils.h"

namespace dairlib {
namespace examples {

class DepthCameraRenderNoTerrain : public drake::systems::Diagram<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DepthCameraRenderNoTerrain)

  DepthCameraRenderNoTerrain(
      std::unique_ptr<drake::multibody::MultibodyPlant<double>> plant,
      drake::math::RigidTransform<double>  camera_pose,
      const std::string& urdf = "examples/Cassie/urdf/cassie_v2.urdf");

  /// @return the output port for the plant state as an OutputVector.
  const drake::systems::OutputPort<double>& get_state_input_port() const {
    return this->get_output_port(state_input_port_index_);
  }

  /// @return the camera image output port.
  const drake::systems::OutputPort<double>& get_camera_out_output_port() const {
    return this->get_output_port(camera_out_output_port_index_);
  }

  /// @return the camera pose output port.
  const drake::systems::OutputPort<double>& get_camera_pose_output_port() const {
    return this->get_output_port(camera_pose_output_port_index_);
  }

  drake::math::RigidTransform<double> X_BC() {
    return cam_transform_;
  }

 private:

  drake::math::RigidTransform<double> cam_transform_ =
      drake::math::RigidTransform<double>(
          camera::MakeXZAlignedCameraRotation(-0.85*M_PI/2),
          Eigen::Vector3d(0.175, 0, 0.15));
  drake::multibody::MultibodyPlant<double>* plant_;
  drake::geometry::SceneGraph<double>* scene_graph_;
  const int state_input_port_index_ = 0;
  const int camera_out_output_port_index_ = 0;
  const int camera_pose_output_port_index_ = 1;
};
}  // namespace examples
}  // namespace dairlib
