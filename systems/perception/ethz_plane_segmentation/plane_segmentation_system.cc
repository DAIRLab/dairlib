#include "common/find_resource.h"
#include "plane_segmentation_system.h"
#include "systems/perception/grid_map_filters/inpainting.h"
#include "opencv2/core/eigen.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <limits>

namespace dairlib {
namespace perception {

using Eigen::MatrixXf;

using grid_map::GridMap;
using convex_plane_decomposition::PlaneDecompositionParams;
using convex_plane_decomposition::GridMapPreprocessing;
using convex_plane_decomposition::sliding_window_plane_extractor::SlidingWindowPlaneExtractor;

PlaneSegmentationSystem::PlaneSegmentationSystem(std::string params_yaml) {
  this->set_name("Plane Segmentation");
  DeclareAbstractInputPort("elevation_map", drake::Value<GridMap>());
  DeclareAbstractOutputPort("segmented_map", &PlaneSegmentationSystem::CalcOutput);

  auto params = drake::yaml::LoadYamlFile<PlaneDecompositionParams>(
      FindResourceOrThrow(params_yaml));

  params_ = params;

  preprocessor_ = std::make_unique<GridMapPreprocessing>(params.preprocessing_params);
  plane_extractor_ = std::make_unique<SlidingWindowPlaneExtractor>(
      params.sliding_window_params, params.ransac_params);

  int erosionSize = 1 + params_.marginSize;  // single sided length of the kernel
  int erosionType = cv::MORPH_ELLIPSE;
  margin_kernel_ = cv::getStructuringElement(
      erosionType, cv::Size(2 * erosionSize + 1, 2 * erosionSize + 1));

}

void PlaneSegmentationSystem::CalcOutput(
    const drake::systems::Context<double>& context, GridMap *map_out) const {

  GridMap grid_map = this->EvalAbstractInput(context, 0)->get_value<GridMap>();

  grid_map.convertToDefaultStartIndex();

  preprocessor_->preprocess(grid_map, "elevation");
  plane_extractor_->runExtraction(grid_map, "elevation");

  cv::Mat binary_image;
  if (plane_extractor_->has_plane_extraction()) {
    cv::Mat local_binary_image;
    const auto& planesMap = plane_extractor_->getSegmentedPlanesMap();
    for (const auto& label_plane : planesMap.labelPlaneParameters) {
      const int label = label_plane.first;
      local_binary_image = planesMap.labeledImage == label;

      // Try with safety margin
      cv::erode(local_binary_image, local_binary_image, margin_kernel_,
                cv::Point(-1, -1), 1, cv::BORDER_REPLICATE);

      binary_image = binary_image.empty() ?
          local_binary_image : binary_image | local_binary_image;
    }
  } else {
    binary_image = plane_extractor_->getBinaryLabeledImage();
    cv::erode(binary_image, binary_image, margin_kernel_, cv::Point(-1,-1), 1, cv::BORDER_REPLICATE);
  }

  MatrixXf segmentation = MatrixXf::Zero(grid_map.getSize()(0), grid_map.getSize()(1));
  cv::cv2eigen(binary_image, segmentation);

  MatrixXf segmented_elevation = grid_map.get("elevation");
  for (int r = 0; r < segmented_elevation.rows(); ++r) {
    for (int c = 0; c < segmented_elevation.cols(); ++c) {
      if (segmentation(r, c) == 0) {
        segmented_elevation(r, c) = std::numeric_limits<float>::quiet_NaN();
      }
    }
  }

  InpaintWithMinimumValues(grid_map, "elevation", "elevation_inpainted");

  grid_map["segmentation"] = segmentation;
  grid_map["segmented_elevation"] = segmented_elevation;
  *map_out = grid_map;
}

}
}