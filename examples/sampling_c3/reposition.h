#include <utility>

#include <Eigen/Core>

#include "examples/sampling_c3/parameter_headers/reposition_params.h"
#include "examples/sampling_c3/parameter_headers/sampling_c3_options.h"

#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/query_object.h"

namespace dairlib {
namespace systems {

/// Public function for generating a set of repositioning knot points.  If the
/// repositioning trajectory gets to the target within a single timestep,
/// finished_reposition_flag is set to true.
///
/// For the piecewise-linear strategy, if reposition_params enables adaptive
/// repositioning and a populated query_object is supplied, the candidate move
/// is collision-checked (see ComputeRepositionClearance): a clear direct 3D
/// segment routes to the diagonal RepositionStraightLine, and otherwise the
/// up/over/down move rises only to the lowest collision-free cruise height
/// rather than the fixed reposition_params.pwl_waypoint_height.  Pass
/// query_object == nullptr (the default) to disable that check and reproduce
/// the original fixed-height behavior; ee_geometry_id / ee_radius are the EE
/// collision geometry and its radius, used only for the check.
Eigen::MatrixXd Reposition(
    const int& n_q, const int& n_x, const int& N, const Eigen::VectorXd& x_lcs,
    const Eigen::Vector3d& repos_target, const double& dt,
    const bool& is_doing_c3, bool& finished_reposition_flag,
    const SamplingC3RepositionParams& reposition_params,
    const SamplingC3Options& sampling_c3_options,
    const drake::geometry::QueryObject<double>* query_object = nullptr,
    drake::geometry::GeometryId ee_geometry_id = {}, double ee_radius = 0.0);

/// Individual repositioning functions for each type of trajectory.  Each sets
/// the knot points and finished_reposition_flag appropriately.
void RepositionStraightLine(
    Eigen::MatrixXd& knots, const int& n_q, const int& n_x, const int& N,
    const Eigen::VectorXd& x_lcs, const Eigen::Vector3d& repos_target,
    const double& dt, const bool& is_doing_c3, bool& finished_reposition_flag,
    const SamplingC3RepositionParams& reposition_params);
void RepositionSpline(Eigen::MatrixXd& knots, const int& n_q, const int& N,
                      const Eigen::VectorXd& x_lcs,
                      const Eigen::Vector3d& repos_target, const double& dt,
                      const bool& is_doing_c3, bool& finished_reposition_flag,
                      const SamplingC3RepositionParams& reposition_params,
                      const SamplingC3Options& sampling_c3_options);
void RepositionSpherical(Eigen::MatrixXd& knots, const int& n_q, const int& N,
                         const Eigen::VectorXd& x_lcs,
                         const Eigen::Vector3d& repos_target, const double& dt,
                         const bool& is_doing_c3,
                         bool& finished_reposition_flag,
                         const SamplingC3RepositionParams& reposition_params,
                         const SamplingC3Options& sampling_c3_options);
void RepositionCircular(Eigen::MatrixXd& knots, const int& n_q, const int& N,
                        const Eigen::VectorXd& x_lcs,
                        const Eigen::Vector3d& repos_target, const double& dt,
                        const bool& is_doing_c3, bool& finished_reposition_flag,
                        const SamplingC3RepositionParams& reposition_params);
void RepositionPiecewiseLinear(
    Eigen::MatrixXd& knots, const int& N, const Eigen::VectorXd& x_lcs,
    const Eigen::Vector3d& repos_target, const double& dt,
    const bool& is_doing_c3, bool& finished_reposition_flag,
    const SamplingC3RepositionParams& reposition_params,
    const double& adaptive_waypoint_height);

void EnforceNoGroundPenetration(Eigen::MatrixXd& knots, double min_z);

/// Collision-checks a candidate repositioning move for adaptive
/// piecewise-linear repositioning (the direct_path_clear /
/// adaptive_waypoint_height arguments to Reposition()).  Every geometry except
/// ee_geometry_id must stay at least
/// (sampling_c3_options.workspace_margins + ee_radius +
///  reposition_params.pwl_clearance_margin) clear.
///
/// Returns {direct_path_clear, min_cruise_height}:
///  - direct_path_clear: whether the straight 3D segment
///    current_ee_location -> target is clear;
///  - min_cruise_height: the lowest collision-free horizontal cruise height in
///    [max(ee_z, target_z, workspace floor),
///    reposition_params.pwl_waypoint_height], scanned in
///    reposition_params.pwl_height_search_step increments (falls back to the
///    upper bound if none is clear).
std::pair<bool, double> ComputeRepositionClearance(
    const drake::geometry::QueryObject<double>& query_object,
    drake::geometry::GeometryId ee_geometry_id,
    const Eigen::Vector3d& current_ee_location, const Eigen::Vector3d& target,
    double ee_radius, const SamplingC3RepositionParams& reposition_params,
    const SamplingC3Options& sampling_c3_options);

}  // namespace systems
}  // namespace dairlib
