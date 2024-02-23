#pragma once
#include <random>

#include "systems/controllers/sampling_params.h"
#include <Eigen/Core>
#include <Eigen/Dense>

using Eigen::VectorXd;
using Eigen::Vector3d;


#define PI 3.14159265359

namespace dairlib{
namespace systems{

// Public function signature.
std::vector<Eigen::VectorXd> generate_sample_states(
    const int& n_q,
    const int& n_v,
    const Eigen::VectorXd& x_lcs,
    const bool& is_doing_c3,
    const SamplingC3SamplingParams sampling_params);

// Private function signatures.
static Eigen::VectorXd generate_radially_symmetric_sample_location(
    const int& n_q,
    const int& n_v,
    const Eigen::VectorXd& x_lcs,
    const int& num_samples,
    const int& i,
    const double& sampling_radius,
    const double& sampling_height);

static Eigen::VectorXd generate_random_sample_location_on_circle(
    const int& n_q,
    const int& n_v,
    const Eigen::VectorXd& x_lcs,
    const double& sampling_radius,
    const double& sampling_height);

static Eigen::VectorXd generate_random_sample_location_on_sphere(
    const int& n_q,
    const int& n_v,
    const Eigen::VectorXd& x_lcs,
    const double& sampling_radius,
    const double& min_angle_from_vertical,
    const double& max_angle_from_vertical);
} // namespace systems
} // namespace dairlib


