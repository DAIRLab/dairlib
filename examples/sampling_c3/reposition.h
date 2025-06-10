#include <Eigen/Core>

#include "examples/sampling_c3/parameter_headers/sampling_params.h"
#include "examples/sampling_c3/parameter_headers/sampling_c3_options.h"

namespace dairlib {
namespace systems {

/// Public function for generating a set of repositioning knot points.  If the
/// repositioning trajectory gets to the target within a single timestep,
/// finished_reposition_flag is set to true.
Eigen::MatrixXd reposition(
    const int& n_q,
    const int& n_x,
    const int& N,
    const Eigen::VectorXd& x_lcs,
    const Eigen::Vector3d& repos_target,
    const double& dt,
    const bool& is_doing_c3,
    bool& finished_reposition_flag,
    const SamplingParams& sampling_params,
    const SamplingC3Options& sampling_c3_options
);

/// Individual repositioning functions for each type of trajectory.  Each sets
/// the knot points and finished_reposition_flag appropriately.
void reposition_straight_line(
    Eigen::MatrixXd& knots,
    const int& n_q,
    const int& n_x,
    const int& N,
    const Eigen::VectorXd& x_lcs,
    const Eigen::Vector3d& repos_target,
    const double& dt,
    const bool& is_doing_c3,
    bool& finished_reposition_flag,
    const SamplingParams& sampling_params
);
void reposition_spline(
    Eigen::MatrixXd& knots,
    const int& n_q,
    const int& N,
    const Eigen::VectorXd& x_lcs,
    const Eigen::Vector3d& repos_target,
    const double& dt,
    const bool& is_doing_c3,
    bool& finished_reposition_flag,
    const SamplingParams& sampling_params,
    const SamplingC3Options& sampling_c3_options);
void reposition_spherical(
    Eigen::MatrixXd& knots,
    const int& n_q,
    const int& N,
    const Eigen::VectorXd& x_lcs,
    const Eigen::Vector3d& repos_target,
    const double& dt,
    const bool& is_doing_c3,
    bool& finished_reposition_flag,
    const SamplingParams& sampling_params,
    const SamplingC3Options& sampling_c3_options);
void reposition_circular(
    Eigen::MatrixXd& knots,
    const int& n_q,
    const int& N,
    const Eigen::VectorXd& x_lcs,
    const Eigen::Vector3d& repos_target,
    const double& dt,
    const bool& is_doing_c3,
    bool& finished_reposition_flag,
    const SamplingParams& sampling_params);
void reposition_piecewise_linear(
    Eigen::MatrixXd& knots,
    const int& N,
    const Eigen::VectorXd& x_lcs,
    const Eigen::Vector3d& repos_target,
    const double& dt,
    const bool& is_doing_c3,
    bool& finished_reposition_flag,
    const SamplingParams& sampling_params);

}   // namespace systems
}   // namespace dairlib
