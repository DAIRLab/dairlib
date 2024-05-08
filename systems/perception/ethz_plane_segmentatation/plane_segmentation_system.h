#pragma once

#include "convex_plane_decomposition/ransac/RansacPlaneExtractor.hpp"
#include "convex_plane_decomposition/sliding_window_plane_extraction/SlidingWindowPlaneExtractor.h"

#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace perception {

class PlaneSegmentationSystem : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PlaneSegmentationSystem);
  PlaneSegmentationSystem();
 private:

  void CalcOutput(const drake::systems::Context<double>& context,
                  grid_map::GridMap* map_out) const;
};

}
}

