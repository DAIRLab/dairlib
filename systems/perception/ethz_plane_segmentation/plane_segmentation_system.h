#pragma once

#include "convex_plane_decomposition/sliding_window_plane_extraction/SlidingWindowPlaneExtractor.h"
#include "convex_plane_decomposition/PlaneDecompositionParams.h"
#include "convex_plane_decomposition/GridMapPreprocessing.h"

#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace perception {

using convex_plane_decomposition::sliding_window_plane_extractor::SlidingWindowPlaneExtractor;
using convex_plane_decomposition::GridMapPreprocessing;
using convex_plane_decomposition::PlaneDecompositionParams;

class PlaneSegmentationSystem : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PlaneSegmentationSystem);
  explicit PlaneSegmentationSystem(std::string params_yaml);
 private:

  void CalcOutput(const drake::systems::Context<double>& context,
                  grid_map::GridMap* map_out) const;

  std::unique_ptr<GridMapPreprocessing> preprocessor_ = nullptr;
  std::unique_ptr<SlidingWindowPlaneExtractor> plane_extractor_ = nullptr;

  PlaneDecompositionParams params_;

};

}
}

