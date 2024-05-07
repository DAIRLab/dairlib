#pragma once

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram.h"
#include "drake/geometry/meshcat.h"

/*!
 * Wrapper class for visualizing a plant with meshcat when the visualizer runs
 * in its own process (i.e. state information does not come from the simulation
 * scene graph)
 */
namespace dairlib {
namespace systems {

class MeshcatCameraManager : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MeshcatCameraManager);
  MeshcatCameraManager(const drake::multibody::MultibodyPlant<double>& plant,
                       const drake::multibody::Frame<double>& camera_track_frame,
                       std::shared_ptr<drake::geometry::Meshcat> meshcat);

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return get_input_port(input_port_state_);
  };

  const drake::systems::InputPort<double>& get_input_port_cam_pos() const {
    return get_input_port(input_port_cam_pos_);
  };

 private:

  drake::systems::EventStatus UpdateMeshcat(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  const drake::multibody::MultibodyPlant<double>& plant_;
  std::unique_ptr<drake::systems::Context<double>> plant_context_;
  std::shared_ptr<drake::geometry::Meshcat> meshcat_;
  const drake::multibody::Frame<double>& camera_track_frame_;

  drake::systems::InputPortIndex input_port_state_;
  drake::systems::InputPortIndex input_port_cam_pos_;
};

class PlantVisualizer : public drake::systems::Diagram<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PlantVisualizer);
  explicit PlantVisualizer(const std::string& urdf,
                           const std::string& body_to_track="");

  std::shared_ptr<drake::geometry::Meshcat> get_meshcat() {
    return meshcat_;
  }

  const drake::multibody::MultibodyPlant<double>& get_plant() {
    return plant_;
  }

 private:
  std::shared_ptr<drake::geometry::Meshcat> meshcat_;
  drake::multibody::MultibodyPlant<double> plant_;

};

} // systems
} // dairlib
