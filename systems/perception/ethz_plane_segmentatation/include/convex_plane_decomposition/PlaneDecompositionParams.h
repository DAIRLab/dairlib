#pragma once

#include <cmath>
#include "convex_plane_decomposition/sliding_window_plane_extraction/SlidingWindowPlaneExtractorParameters.h"
#include "convex_plane_decomposition/ransac/RansacPlaneExtractorParameters.h"
#include "convex_plane_decomposition/GridMapPreprocessing.h"

#include "drake/common/yaml/yaml_read_archive.h"
#include "drake/common/yaml/yaml_io.h"

namespace convex_plane_decomposition {

using sliding_window_plane_extractor::SlidingWindowPlaneExtractorParameters;
using ransac_plane_extractor::RansacPlaneExtractorParameters;

struct PlaneDecompositionParams {
  // preprocessing
  double resolution;
  int kernelSize;
  int numberOfRepeats;
  double gaussBlur;

  // sliding window
  int kernel_size;
  int planarity_opening_filter;
  double plane_inclination_threshold_degrees;
  double local_plane_inclination_threshold_degrees;
  double plane_patch_error_threshold;
  int min_number_points_per_label;
  int connectivity;
  bool include_ransac_refinement;
  double global_plane_fit_distance_error_threshold;
  double global_plane_fit_angle_error_threshold_degrees;

  // ransac
  double probability;
  double min_points;
  double epsilon;
  double cluster_epsilon;
  double normal_threshold;

  // contour extraction
  int marginSize = 1;

  SlidingWindowPlaneExtractorParameters sliding_window_params{};
  RansacPlaneExtractorParameters ransac_params{};
  PreprocessingParameters preprocessing_params{};

  template<typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(resolution));
    a->Visit(DRAKE_NVP(kernelSize));
    a->Visit(DRAKE_NVP(numberOfRepeats));
    a->Visit(DRAKE_NVP(gaussBlur));
    a->Visit(DRAKE_NVP(kernel_size));
    a->Visit(DRAKE_NVP(planarity_opening_filter));
    a->Visit(DRAKE_NVP(plane_inclination_threshold_degrees));
    a->Visit(DRAKE_NVP(local_plane_inclination_threshold_degrees));
    a->Visit(DRAKE_NVP(plane_patch_error_threshold));
    a->Visit(DRAKE_NVP(min_number_points_per_label));
    a->Visit(DRAKE_NVP(connectivity));
    a->Visit(DRAKE_NVP(include_ransac_refinement));
    a->Visit(DRAKE_NVP(global_plane_fit_distance_error_threshold));
    a->Visit(DRAKE_NVP(global_plane_fit_angle_error_threshold_degrees));
    a->Visit(DRAKE_NVP(probability));
    a->Visit(DRAKE_NVP(min_points));
    a->Visit(DRAKE_NVP(epsilon));
    a->Visit(DRAKE_NVP(cluster_epsilon));
    a->Visit(DRAKE_NVP(normal_threshold));
    a->Visit(DRAKE_NVP(marginSize));

    preprocessing_params.resolution = resolution;
    preprocessing_params.kernelSize = kernelSize;
    preprocessing_params.numberOfRepeats = numberOfRepeats;
    preprocessing_params.gaussBlur = gaussBlur;

    sliding_window_params.kernel_size = kernel_size;
    sliding_window_params.planarity_opening_filter = planarity_opening_filter;
    sliding_window_params.plane_inclination_threshold = M_PI * plane_inclination_threshold_degrees / 180.0;
    sliding_window_params.local_plane_inclination_threshold = M_PI * local_plane_inclination_threshold_degrees / 180.0;
    sliding_window_params.plane_patch_error_threshold = plane_patch_error_threshold;
    sliding_window_params.min_number_points_per_label = min_number_points_per_label;
    sliding_window_params.connectivity = connectivity;
    sliding_window_params.include_ransac_refinement = include_ransac_refinement;
    sliding_window_params.global_plane_fit_distance_error_threshold = global_plane_fit_distance_error_threshold;
    sliding_window_params.global_plane_fit_angle_error_threshold_degrees = global_plane_fit_angle_error_threshold_degrees;

    ransac_params.probability = probability;
    ransac_params.min_points = min_points;
    ransac_params.epsilon = epsilon;
    ransac_params.cluster_epsilon = cluster_epsilon;
    ransac_params.normal_threshold = normal_threshold;

  }

  static PreprocessingParameters GetPreProcessingParams(std::string fname) {
    return drake::yaml::LoadYamlFile<PlaneDecompositionParams>(fname).preprocessing_params;
  }

  static SlidingWindowPlaneExtractorParameters GetSlidingWindowParams(std::string fname) {
    return drake::yaml::LoadYamlFile<PlaneDecompositionParams>(fname).sliding_window_params;
  }

  static RansacPlaneExtractorParameters GetRansacParams(std::string fname) {
    return drake::yaml::LoadYamlFile<PlaneDecompositionParams>(fname).ransac_params;
  }
};

}