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

class PlantVisualizer : public drake::systems::Diagram<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PlantVisualizer);
  PlantVisualizer(const std::string& urdf);

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
