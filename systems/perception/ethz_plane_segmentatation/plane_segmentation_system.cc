#include "plane_segmentation_system.h"

namespace dairlib {
namespace perception {

using grid_map::GridMap;

PlaneSegmentationSystem::PlaneSegmentationSystem() {

  DeclareAbstractInputPort("elevation_map", drake::Value<GridMap>());
  DeclareAbstractOutputPort("segmented_map", &PlaneSegmentationSystem::CalcOutput);

}

void PlaneSegmentationSystem::CalcOutput(
    const drake::systems::Context<double>& context, GridMap *map_out) const {

}

}
}