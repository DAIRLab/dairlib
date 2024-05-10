#include "common/find_resource.h"
#include "plane_segmentation_system.h"
#include "opencv2/core/eigen.hpp"

namespace dairlib {
namespace perception {

using Eigen::MatrixXf;

using grid_map::GridMap;
using convex_plane_decomposition::PlaneDecompositionParams;
using convex_plane_decomposition::GridMapPreprocessing;
using convex_plane_decomposition::sliding_window_plane_extractor::SlidingWindowPlaneExtractor;

PlaneSegmentationSystem::PlaneSegmentationSystem(std::string params_yaml) {
  DeclareAbstractInputPort("elevation_map", drake::Value<GridMap>());
  DeclareAbstractOutputPort("segmented_map", &PlaneSegmentationSystem::CalcOutput);

  auto params = drake::yaml::LoadYamlFile<PlaneDecompositionParams>(
      FindResourceOrThrow(params_yaml));

  preprocessor_ = std::make_unique<GridMapPreprocessing>(params.preprocessing_params);
  plane_extractor_ = std::make_unique<SlidingWindowPlaneExtractor>(
      params.sliding_window_params, params.ransac_params);

}

void PlaneSegmentationSystem::CalcOutput(
    const drake::systems::Context<double>& context, GridMap *map_out) const {

  auto grid_map = this->EvalAbstractInput(context, 0)->get_value<GridMap>();
  preprocessor_->preprocess(grid_map, "elevation");
  plane_extractor_->runExtraction(grid_map, "elevation");

  MatrixXf segmentation = MatrixXf::Zero(
      grid_map.getSize()(0), grid_map.getSize()(1));
  cv::cv2eigen(plane_extractor_->getBinaryLabeledImage(), segmentation);
  grid_map.add("segmented_elevation", segmentation);

  *map_out = grid_map;
}

}
}