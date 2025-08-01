#pragma once

#include "common/file_utils.h"
#include "drake/common/yaml/yaml_read_archive.h"


/* Sample generation options:
  0. kRadiallySymmetric:  radially distributed samples on a planar circle.
  1. kRandomOnCircle:     random samples on a planar circle.
  2. kRandomOnSphere:     random samples on a spherical surface.
  3. kFixed:              fixed set of samples.
  4. kRandomOnPerimeter:  random samples on a perimeter offset past the object
                          surface (roughly planar).
  5. kRandomOnShell:      random samples on a shell offset past the object
                          surface.
  6. kMeshNormal:         random samples on a mesh surface using the mesh
                          face normal projection offset from the mesh.
*/
enum SamplingStrategy {
  kRadiallySymmetric,
  kRandomOnCircle,
  kRandomOnSphere,
  kFixed,
  kRandomOnPerimeter,
  kRandomOnShell,
  kMeshNormal,
  kMeshNormalMultiObject
};

struct SamplingParams {
  SamplingStrategy sampling_strategy;

  /// Parameters relevant for all sampling strategies.
  bool filter_samples_for_safety;

  /// Number of samples.  Add 2 for repositioning (current location, previous
  /// repositioning target) and 1 for C3 (current location) to get total number
  /// solved in parallel for each control loop.
  int num_additional_samples_repos;
  int num_additional_samples_c3;

  /// Sample buffer parameters.
  bool consider_best_buffer_sample_when_leaving_c3;
  int N_sample_buffer;
  double pos_error_sample_retention;
  double ang_error_sample_retention;

  /// Shared across multiple sampling strategies.
  double sampling_radius;               // kRadiallySymmetric, kRandomOnCircle,
                                        // kRandomOnSphere
  double sampling_height;               // kRadiallySymmetric, kRandomOnCircle
                                        // kRandomOnPerimeter
  double sample_projection_clearance;   // kRandomOnPerimeter, kRandomOnShell, kMeshNormal
  double min_angle_from_vertical;       // kRandomOnSphere, kRandomOnShell
  double max_angle_from_vertical;       // kRandomOnSphere, kRandomOnShell

  /// kFixed parameters.
  Eigen::MatrixXd fixed_sample_locations;   // n_samples rows, 3 columns

  /// kRandomOnPerimeter parameters.
  std::vector<double> grid_x_limits;
  std::vector<double> grid_y_limits;

  /// kRandomOnShell parameters.
  double min_sampling_radius;
  double max_sampling_radius;

  // kMeshNormal parameters
  double buffer_distance;
  int max_attempts; 
  double barycentric_bias;

  double z_height;
  bool ee_z_close;
  bool gen_planar_samples;
  double c3_min_clearance;


  template <typename Archive>
  void Serialize(Archive* a) {
    ENUM_DESERIALIZE(a, sampling_strategy);
    a->Visit(DRAKE_NVP(filter_samples_for_safety));
    a->Visit(DRAKE_NVP(fixed_sample_locations));
    a->Visit(DRAKE_NVP(sampling_radius));
    a->Visit(DRAKE_NVP(min_angle_from_vertical));
    a->Visit(DRAKE_NVP(max_angle_from_vertical));
    a->Visit(DRAKE_NVP(sampling_height));
    a->Visit(DRAKE_NVP(grid_x_limits));
    a->Visit(DRAKE_NVP(grid_y_limits));
    a->Visit(DRAKE_NVP(sample_projection_clearance));
    a->Visit(DRAKE_NVP(min_sampling_radius));
    a->Visit(DRAKE_NVP(max_sampling_radius));
    a->Visit(DRAKE_NVP(num_additional_samples_repos));
    a->Visit(DRAKE_NVP(num_additional_samples_c3));
    a->Visit(DRAKE_NVP(consider_best_buffer_sample_when_leaving_c3));
    a->Visit(DRAKE_NVP(N_sample_buffer));
    a->Visit(DRAKE_NVP(pos_error_sample_retention));
    a->Visit(DRAKE_NVP(ang_error_sample_retention));
    a->Visit(DRAKE_NVP(sample_projection_clearance));
    a->Visit(DRAKE_NVP(buffer_distance));
    a->Visit(DRAKE_NVP(max_attempts));
    a->Visit(DRAKE_NVP(barycentric_bias));
    a->Visit(DRAKE_NVP(z_height));
    a->Visit(DRAKE_NVP(c3_min_clearance));
    a->Visit(DRAKE_NVP(ee_z_close));
    a->Visit(DRAKE_NVP(gen_planar_samples));

  }
};
