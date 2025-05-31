#pragma once
#include <random>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <numbers>

#include "examples/sampling_c3/parameter_headers/sampling_c3_options.h"
#include "multibody/geom_geom_collider.h"
#include "multibody/multibody_utils.h"
#include "solvers/c3_options.h"
#include "systems/controllers/sampling_params.h"
#include "common/update_context.h"

using Eigen::Vector3d;
using Eigen::VectorXd;

inline constexpr double kPi = std::numbers::pi;

namespace dairlib {
namespace systems {

// Helper to create a single instance of a random number generator for every
// function call.
inline std::mt19937& Rng() {
  static thread_local std::mt19937 rng{std::random_device{}()};
  return rng;
}

// Public function signature.
std::vector<Eigen::VectorXd> generate_sample_states(
    const int& n_q, const int& n_v, const int& n_u,
    const Eigen::VectorXd& x_lcs, const bool& is_doing_c3,
    const SamplingC3SamplingParams sampling_params,
    const SamplingC3Options& sampling_c3_options,
    drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context,
    drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad,
    drake::systems::Context<drake::AutoDiffXd>* context_ad,
    const std::vector<
        std::vector<drake::SortedPair<drake::geometry::GeometryId>>>&
        contact_geoms);

// Helper function signatures.
bool is_sample_within_workspace(const Eigen::VectorXd& candidate_state,
                                const SamplingC3Options& sampling_c3_options);

void generate_radially_symmetric_sample_location(
    const int& n_q, const int& n_v, Eigen::VectorXd& candidate_state,
    const int& num_samples, const int& i, const double& sampling_radius,
    const double& sampling_height);

void generate_random_sample_location_on_circle(const int& n_q, const int& n_v,
                                               Eigen::VectorXd& candidate_state,
                                               const double& sampling_radius,
                                               const double& sampling_height);

void generate_random_sample_location_on_sphere(
    const int& n_q, const int& n_v, Eigen::VectorXd& candidate_state,
    const double& sampling_radius, const double& min_angle_from_vertical,
    const double& max_angle_from_vertical);

void generate_fixed_sample(const int& n_q, const int& n_v,
                           Eigen::VectorXd& candidate_state,
                           const double& sampling_height,
                           Eigen::VectorXd fixed_sample_location);

Eigen::VectorXd generate_sample_on_perimeter(
    const int& n_q, const int& n_v, const int& n_u,
    const Eigen::VectorXd& x_lcs,
    drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context,
    drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad,
    drake::systems::Context<drake::AutoDiffXd>* context_ad,
    const std::vector<
        std::vector<drake::SortedPair<drake::geometry::GeometryId>>>&
        contact_geoms,
    const SamplingC3SamplingParams& sampling_params,
    const SamplingC3Options sampling_c3_options);

Eigen::VectorXd generate_sample_on_shell(
    const int& n_q, const int& n_v, const int& n_u,
    const Eigen::VectorXd& x_lcs,
    drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context,
    drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad,
    drake::systems::Context<drake::AutoDiffXd>* context_ad,
    const std::vector<
        std::vector<drake::SortedPair<drake::geometry::GeometryId>>>&
        contact_geoms,
    const SamplingC3SamplingParams& sampling_params,
    const SamplingC3Options sampling_c3_options);

bool is_lacking_clearance(
    const int& n_q, const int& n_v, const int& n_u,
    const Eigen::VectorXd& candidate_state,
    drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context,
    drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad,
    drake::systems::Context<drake::AutoDiffXd>* context_ad,
    const std::vector<
        std::vector<drake::SortedPair<drake::geometry::GeometryId>>>&
        contact_geoms,
    const SamplingC3SamplingParams& sampling_params,
    SamplingC3Options sampling_c3_options, int& min_distance_index);

Eigen::VectorXd project_to_clearance_shell(
    Eigen::VectorXd& candidate_state, int min_distance_index,
    const SamplingC3SamplingParams& sampling_params,
    drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context,
    const std::vector<
        std::vector<drake::SortedPair<drake::geometry::GeometryId>>>&
        contact_geoms);

}  // namespace systems
}  // namespace dairlib