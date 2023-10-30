#pragma once

#include "drake/systems/framework/diagram.h"

/*!
 * Wrapper class for visualizing a plant with meshcat when the visualizer runs
 * in its own process
 */
namespace dairlib {
namespace systems {
class PlantVisualizer : drake::systems::Diagram<double> {
 public:

};

} // systems
} // dairlib
