#pragma once

#include <limits>
#include <utility>

#include <Eigen/Core>

#include "examples/sampling_c3/parameter_headers/reposition_params.h"
#include "examples/sampling_c3/parameter_headers/sampling_c3_options.h"

#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_set.h"
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
/// rather than the fixed reposition_params.pwl_waypoint_height.  A short hop
/// (under use_straight_line_traj_under_piecewise_linear of xy travel) also
/// goes straight, but only if StraightHopIsClear(); otherwise it lifts first
/// like any other blocked move.  Pass
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
/// retreat.  @p max_retreat_distance caps the retreat's total length [m], so
/// a retreat aimed at a point stops on it rather than overshooting; the knots
/// are then spaced evenly over that shorter distance.  The remaining arguments
/// carry the same meaning as Reposition().
Eigen::MatrixXd RepositionWithRetreat(
    const int& n_q, const int& n_x, const int& N, const Eigen::VectorXd& x_lcs,
    const Eigen::Vector3d& repos_target, const double& dt,
    const bool& is_doing_c3, const Eigen::Vector3d& retreat_direction,
    const int& num_retreat_knots,
    const SamplingC3RepositionParams& reposition_params,
    const SamplingC3Options& sampling_c3_options,
    const drake::geometry::QueryObject<double>* query_object = nullptr,
    drake::geometry::GeometryId ee_geometry_id = {}, double ee_radius = 0.0,
    double max_retreat_distance = std::numeric_limits<double>::infinity());

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

/// Whether the straight hop @p start -> @p target keeps the EE clear of every
/// geometry except ee_geometry_id: its centre stays at least (ee_radius +
/// sampling_c3_options.workspace_margins) from them, checked at
/// reposition_params.pwl_num_path_collision_samples interior points plus both
/// ends, or as far as @p target itself is when that is less.  A hop that
/// starts inside that clearance (the EE resting on the object, say) may only
/// move away until it is clear; one that slides deeper, or comes back in once
/// clear, is not clear.
///
/// This is the check for the short hops Reposition() takes straight to the
/// target, whose ends sit too close to the object for
/// ComputeRepositionClearance's wider clearance ever to call them clear.
bool StraightHopIsClear(
    const drake::geometry::QueryObject<double>& query_object,
    drake::geometry::GeometryId ee_geometry_id, const Eigen::Vector3d& start,
    const Eigen::Vector3d& target, double ee_radius,
    const SamplingC3RepositionParams& reposition_params,
    const SamplingC3Options& sampling_c3_options);

/// Clamps @p p to sampling_c3_options.workspace_limits, held
/// sampling_c3_options.workspace_margins inside each bound.
void ClampEEPositionToWorkspace(const SamplingC3Options& sampling_c3_options,
                                Eigen::Vector3d* p);

/// The signed distance [m] from the EE centre @p p to the nearest of
/// @p fixed_geometries; infinity if the set reports nothing.  Throws like
/// ClearEEPlanOfFixedGeometries().
double DistanceToFixedGeometries(
    const drake::geometry::QueryObject<double>& query_object,
    const drake::geometry::GeometrySet& fixed_geometries,
    const Eigen::Vector3d& p);

/// Projects the EE centre @p p at least @p knot_clearance (EE centre to
/// surface) off @p fixed_geometries, along the nearest geometry's gradient, and
/// clamps it to the workspace (see ClampEEPositionToWorkspace) before every
/// distance query -- what ClearEEPlanOfFixedGeometries() does to each
/// non-exempt knot.  A clear point inside the workspace is left where it is, so
/// a repositioning target passed through this is exactly where the cleared
/// plan's last knot lands.  Throws like ClearEEPlanOfFixedGeometries().
void ProjectEEPositionOffFixedGeometries(
    const drake::geometry::QueryObject<double>& query_object,
    const drake::geometry::GeometrySet& fixed_geometries, double knot_clearance,
    const SamplingC3Options& sampling_c3_options, Eigen::Vector3d* p);

/// Whether one retreat @p step [m] from @p start along @p direction is mostly
/// undone by the knot projection (see ProjectEEPositionOffFixedGeometries):
/// less than half of it survives.  A plan that retreats that way, rebuilt from
/// the same start every loop, never gets anywhere.  False for a zero
/// direction.
bool RetreatIsBlockedByFixedGeometries(
    const drake::geometry::QueryObject<double>& query_object,
    const drake::geometry::GeometrySet& fixed_geometries, double knot_clearance,
    const SamplingC3Options& sampling_c3_options, const Eigen::Vector3d& start,
    const Eigen::Vector3d& direction, double step);

/// What ClearEEPlanOfFixedGeometries() found along the plan's path.
struct FixedGeometryPathCheck {
  /// The first knot whose incoming straight segment comes within
  /// path_clearance of a fixed geometry, or -1 if the whole path is clear.
  int first_blocked_knot{-1};
  /// Where to hold a blocked plan: the last point before the block, along the
  /// path, that clears knot_clearance (or, if none does, path_clearance), so a
  /// held plan sits where its knots would have been projected to.
  Eigen::Vector3d last_clear_point{Eigen::Vector3d::Zero()};
  /// The EE-centre signed distance [m] where the path was found blocked.
  double blocked_distance{std::numeric_limits<double>::infinity()};
};

/// Keeps an EE position plan (3 x N, one column per knot) off the fixed scene,
/// along its whole path rather than only at its knots.
///
/// First, every knot from @p num_exempt_knots on is projected at least
/// @p knot_clearance (EE centre to surface) away from @p fixed_geometries,
/// along the nearest geometry's gradient, and clamped to the workspace (see
/// ClampEEPositionToWorkspace) before every distance query.  Clamping inside
/// the loop matters for geometry that rests on the floor: a knot pushed out
/// through such a piece's underside would otherwise be lifted straight back
/// into it by a clamp applied afterwards.  The exempt leading knots are only
/// clamped.
///
/// Then the straight segments between consecutive knots are walked from knot
/// max(@p num_exempt_knots - 1, 0), and the first one that comes within
/// @p path_clearance of the geometry is reported -- projecting knot by knot
/// alone lets neighbouring knots inside a wall land on its opposite faces, with
/// the segment between them running straight through it.  A path that starts
/// inside path_clearance (the end of an exempt retreat, say) may leave it; it
/// is checked from the first clear point on.  The walk sphere-traces: from a
/// point at distance d it steps (d - path_clearance), at least 0.5 mm.
///
/// The caller decides what to do with a blocked path; see HoldEEPlanFrom().
/// Throws if a query reports a penetration deeper than 5 cm, which only an
/// unreliable query (a bad collision mesh) produces in this scene.
FixedGeometryPathCheck ClearEEPlanOfFixedGeometries(
    const drake::geometry::QueryObject<double>& query_object,
    const drake::geometry::GeometrySet& fixed_geometries, double knot_clearance,
    double path_clearance, const SamplingC3Options& sampling_c3_options,
    int num_exempt_knots, Eigen::MatrixXd* ee_positions);

/// Holds an EE position plan (3 x N) at @p point from knot @p from_knot on.
void HoldEEPlanFrom(int from_knot, const Eigen::Vector3d& point,
                    Eigen::MatrixXd* ee_positions);

/// What HoldEEPlanAbovePressedObject() carries from one control loop to the
/// next.
struct EEPressLatchState {
  bool engaged{false};
  /// The highest knot-0 height [m] since the latch engaged.
  double floor_z{-std::numeric_limits<double>::infinity()};
};

/// What one HoldEEPlanAbovePressedObject() call did.
struct EEPressLatchStep {
  bool engaged{false};   ///< The latch engaged on this call.
  bool released{false};  ///< The latch released on this call.
  /// Knot 0's EE-surface gap to the object [m]; NaN when the query was not
  /// usable.
  double gap{std::numeric_limits<double>::quiet_NaN()};
  int knots_raised{0};
  double max_lift{0.0};  ///< [m]
};

/// Keeps an EE position plan (3 x N) from descending while it presses down on
/// the object.  This is especially useful for the 3D printer demos:  the
/// printer's z axis is stiff and the finger gives only sideways.  This
/// mechanism stops it from deepening.
///
/// Engages when knot 0 (the plan's start) is inside @p object_geometries and
/// the nearest geometry's outward normal there has a z component of at least @p
/// min_normal_z.  While engaged, every knot below the highest knot-0 height
/// since engaging is raised to it, and xy is left alone, so a push into the
/// object's side is unaffected.  Releases once knot 0 is at least
/// @p release_gap clear of the object.  A query that reports nothing, or a
/// penetration deeper than 5 cm (an unreliable query), changes neither
/// engagement nor release.
///
/// Raising knots can move them toward a downward-facing fixed surface, so a
/// caller that keeps plans off the fixed scene should check the plan again
/// when this raised any knot.
EEPressLatchStep HoldEEPlanAbovePressedObject(
    const drake::geometry::QueryObject<double>& query_object,
    const drake::geometry::GeometrySet& object_geometries, double ee_radius,
    double min_normal_z, double release_gap, EEPressLatchState* state,
    Eigen::MatrixXd* ee_positions);

}  // namespace systems
}  // namespace dairlib
