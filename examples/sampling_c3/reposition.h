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

/// A repositioning plan that first backs the end effector away from the object
/// and only then heads for @p repos_target.
///
/// The first @p num_retreat_knots knots step along @p retreat_direction, one
/// knot period @p dt apart at MaxSpeedAlongDirection() -- i.e. as fast as that
/// direction allows -- and the remaining (N - num_retreat_knots) are whatever
/// Reposition() plans from the end of that retreat.  Knot 0 stays exactly at
/// the current end effector position.
///
/// Falls back to plain Reposition() when there is nothing to retreat along
/// (a zero @p retreat_direction) or no room to do it in (@p num_retreat_knots
/// <= 0), rather than inventing a direction.  @p num_retreat_knots is clamped
/// to N - 1 so at least one knot is always left for the repositioning leg.
/// Unlike Reposition() there is no finished_reposition_flag out-param: a
/// retreating plan has not arrived anywhere.
///
/// @p retreat_direction need not be normalized; a zero vector disables the
/// retreat.  The remaining arguments carry the same meaning as Reposition().
Eigen::MatrixXd RepositionWithRetreat(
    const int& n_q, const int& n_x, const int& N, const Eigen::VectorXd& x_lcs,
    const Eigen::Vector3d& repos_target, const double& dt,
    const bool& is_doing_c3, const Eigen::Vector3d& retreat_direction,
    const int& num_retreat_knots,
    const SamplingC3RepositionParams& reposition_params,
    const SamplingC3Options& sampling_c3_options,
    const drake::geometry::QueryObject<double>* query_object = nullptr,
    drake::geometry::GeometryId ee_geometry_id = {}, double ee_radius = 0.0);

/// The fastest the end effector may travel along @p direction without exceeding
/// horizontal or vertical speeds. @p reposition_params.speed_horizontal and the
/// vertical one at speed_vertical, so whichever saturates first sets the speed.
///
/// @p direction need not be normalized; a zero @p direction returns 0.
double MaxSpeedAlongDirection(
    const Eigen::Vector3d& direction,
    const SamplingC3RepositionParams& reposition_params);

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
