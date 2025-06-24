#pragma once
#include <random>

#include "systems/controllers/sampling_params.h"
#include "solvers/c3_options.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include "multibody/multibody_utils.h"
#include "multibody/geom_geom_collider.h"
#include "multibody/hydroelastic_geom_collider.h"
#include <vector>
#include <string>
#include <random>
#include <stdexcept>
#include <cmath>
#include <iostream>
#include <igl/readOBJ.h>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/algorithms/union.hpp>
#include <boost/geometry/algorithms/buffer.hpp>
#include <boost/geometry/strategies/buffer.hpp>
#include <drake/geometry/meshcat.h>
#include <drake/geometry/query_object.h>
#include <drake/geometry/scene_graph.h>
#include <drake/geometry/shape_specification.h>
#include <drake/geometry/proximity/triangle_surface_mesh.h>
#include <drake/geometry/proximity/obj_to_surface_mesh.h>
#include <drake/geometry/geometry_state.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/systems/framework/diagram_builder.h>

using Eigen::VectorXd;
using Eigen::Vector3d;


#define PI 3.14159265359

namespace dairlib{
namespace systems{

// Public function signature.
std::vector<Eigen::VectorXd> generate_sample_states(
    const int& n_q,
    const int& n_v,
    const int& n_u,
    const Eigen::VectorXd& x_lcs,
    const bool& is_doing_c3,
    const SamplingC3SamplingParams sampling_params,
    const C3Options c3_options,
    drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context,
    drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad,
    drake::systems::Context<drake::AutoDiffXd>* context_ad,
    const std::vector<std::vector<drake::SortedPair<drake::geometry::GeometryId>>>& contact_geoms,
    drake::geometry::TriangleSurfaceMesh<double> mesh);

// Private function signatures.
bool is_sample_within_workspace(
    const Eigen::VectorXd& candidate_state,
    const C3Options c3_options);

Eigen::VectorXd generate_radially_symmetric_sample_location(
    const int& n_q,
    const int& n_v,
    const Eigen::VectorXd& x_lcs,
    const int& num_samples,
    const int& i,
    const double& sampling_radius,
    const double& sampling_height);

Eigen::VectorXd generate_random_sample_location_on_circle(
    const int& n_q,
    const int& n_v,
    const Eigen::VectorXd& x_lcs,
    const double& sampling_radius,
    const double& sampling_height);

Eigen::VectorXd generate_random_sample_location_on_sphere(
    const int& n_q,
    const int& n_v,
    const Eigen::VectorXd& x_lcs,
    const double& sampling_radius,
    const double& min_angle_from_vertical,
    const double& max_angle_from_vertical);

Eigen::VectorXd generate_fixed_sample(
  const int& n_q,
  const int& n_v,
  const Eigen::VectorXd& x_lcs,
  const double& sampling_height,
  Eigen::VectorXd fixed_sample_location);

Eigen::VectorXd generate_sample_on_grid(
    const int& n_q,
    const int& n_v,
    const int& n_u,
    const Eigen::VectorXd& x_lcs,
    drake::multibody::MultibodyPlant<double>& plant, 
  drake::systems::Context<double>* context, 
  drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad,
  drake::systems::Context<drake::AutoDiffXd>* context_ad,
  const std::vector<std::vector<drake::SortedPair<drake::geometry::GeometryId>>>& contact_geoms,
  const SamplingC3SamplingParams& sampling_params,
  const C3Options c3_options);

Eigen::VectorXd generate_sample_in_shell(
    const int& n_q,
    const int& n_v,
    const int& n_u,
    const Eigen::VectorXd& x_lcs,
    drake::multibody::MultibodyPlant<double>& plant, 
  drake::systems::Context<double>* context, 
  drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad,
  drake::systems::Context<drake::AutoDiffXd>* context_ad,
  const std::vector<std::vector<drake::SortedPair<drake::geometry::GeometryId>>>& contact_geoms,
  const SamplingC3SamplingParams& sampling_params,
  const C3Options c3_options);

Eigen::VectorXd generate_sample_mesh_buffer(
    const int& n_q,
    const int& n_v,
    const int& n_u,
    const Eigen::VectorXd& x_lcs,
    const std::string& mesh_path,
    const SamplingC3SamplingParams& sampling_params,
    C3Options c3_options);

Eigen::VectorXd generate_sample_mesh_drake(
    const int& n_q,
    const int& n_v,
    const int& n_u,
    const Eigen::VectorXd& x_lcs,
    drake::multibody::MultibodyPlant<double>& plant, 
    drake::systems::Context<double>* context, 
    drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad,
    drake::systems::Context<drake::AutoDiffXd>* context_ad,
    const std::vector<std::vector<drake::SortedPair<drake::geometry::GeometryId>>>& contact_geoms,
    const std::string& mesh_path,
    const SamplingC3SamplingParams& sampling_params,
    const drake::geometry::QueryObject<double>& query_object,
    C3Options c3_options,
    const drake::geometry::GeometryId mesh_geometry_id,
    const drake::geometry::TriangleSurfaceMesh<double>& mesh);

bool check_collision(
    const int& n_q,
    const int& n_v,
    const int& n_u,
    const Eigen::VectorXd& candidate_state,
    drake::multibody::MultibodyPlant<double>& plant, 
    drake::systems::Context<double>* context,
    drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad,
    drake::systems::Context<drake::AutoDiffXd>* context_ad,
    const std::vector<std::vector<drake::SortedPair<drake::geometry::GeometryId>>>& contact_geoms,
    const SamplingC3SamplingParams& sampling_params,
    C3Options c3_options,
    int& min_distance_index);

void UpdateContext(
    const int& n_q,
    const int& n_v,
    const int& n_u,
    drake::multibody::MultibodyPlant<double>& plant, 
    drake::systems::Context<double>* context, 
    drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad,
    drake::systems::Context<drake::AutoDiffXd>* context_ad,
    Eigen::VectorXd lcs_state);

Eigen::VectorXd project_to_surface(Eigen::VectorXd& candidate_state, 
  int min_distance_index,
  const SamplingC3SamplingParams& sampling_params,
  drake::multibody::MultibodyPlant<double>& plant,
  drake::systems::Context<double>* context,
  const std::vector<std::vector<drake::SortedPair<drake::geometry::GeometryId>>>& contact_geoms);

} // namespace systems
} // namespace dairlib


