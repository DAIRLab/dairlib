#include "reposition.h"

#include <algorithm>

#include "drake/common/drake_assert.h"

namespace dairlib {
namespace systems {

// TODO @bibit further cleanup could require +/- z workspace limits instead of
// sampling_c3_options
Eigen::MatrixXd Reposition(const int& n_q, const int& n_x, const int& N,
                           const Eigen::VectorXd& x_lcs,
                           const Eigen::Vector3d& repos_target,
                           const double& dt, const bool& is_doing_c3,
                           bool& finished_reposition_flag,
                           const SamplingC3RepositionParams& reposition_params,
                           const SamplingC3Options& sampling_c3_options,
                           const drake::geometry::QueryObject<double>*
                               query_object,
                           drake::geometry::GeometryId ee_geometry_id,
                           double ee_radius) {
  Eigen::MatrixXd knots = Eigen::MatrixXd::Zero(n_x, N);

  Eigen::Vector3d current_ee_location = x_lcs.head(3);
  Eigen::Vector3d current_object_location = x_lcs.segment(n_q - 3, 3);
  Eigen::Vector3d curr_to_goal_vec = repos_target - current_ee_location;

  // For adaptive piecewise-linear repositioning, collision-check the candidate
  // move so we can take the direct diagonal when it is clear, or otherwise rise
  // only to the lowest collision-free cruise height instead of the fixed
  // pwl_waypoint_height.  Without a scene (query_object == nullptr) or when
  // disabled, fall back to the original fixed-height behavior.
  bool direct_path_clear = false;
  double adaptive_waypoint_height = reposition_params.pwl_waypoint_height;
  if (query_object != nullptr &&
      reposition_params.pwl_adaptive_waypoint_height &&
      reposition_params.traj_type ==
          RepositioningTrajectoryType::kPiecewiseLinear) {
    const auto clearance = ComputeRepositionClearance(
        *query_object, ee_geometry_id, current_ee_location, repos_target,
        ee_radius, reposition_params, sampling_c3_options);
    direct_path_clear = clearance.first;
    adaptive_waypoint_height = clearance.second;
  }

  // Get two unit vectors in the plane of the arc between the current and goal
  // ee locations.
  Eigen::Vector3d v1 =
      (current_ee_location - current_object_location).normalized();
  Eigen::Vector3d v2 = (repos_target - current_object_location).normalized();
  // NOTE:  Need to clamp between not quite (-1, 1) to avoid NaNs.
  double travel_angle = std::acos(std::clamp(v1.dot(v2), -0.9999, 0.9999));

  // Compute EE position errors to repositioning target.
  double travel_distance = curr_to_goal_vec.norm();
  double xy_travel_distance = curr_to_goal_vec.head(2).norm();
  // Use a straight line trajectory if close to the target.
  RepositioningTrajectoryType traj_type = reposition_params.traj_type;
  bool allow_ground_penetration = false;
  if ((travel_distance <
           reposition_params.use_straight_line_traj_under_spline &&
       traj_type == RepositioningTrajectoryType::kSpline) ||
      ((traj_type == RepositioningTrajectoryType::kSpherical ||
        traj_type == RepositioningTrajectoryType::kCircular) &&
       travel_angle < reposition_params.use_straight_line_traj_within_angle) ||
      (traj_type == RepositioningTrajectoryType::kPiecewiseLinear &&
       (direct_path_clear ||
        (xy_travel_distance <
         reposition_params.use_straight_line_traj_under_piecewise_linear) ||
        ((xy_travel_distance <
          reposition_params.use_straight_line_traj_under_piecewise_linear +
              0.01) &&
         (current_ee_location[2] < reposition_params.pwl_waypoint_height))))) {
    RepositionStraightLine(knots, n_q, n_x, N, x_lcs, repos_target, dt,
                           is_doing_c3, finished_reposition_flag,
                           reposition_params);
    allow_ground_penetration = true;
  } else if (traj_type == RepositioningTrajectoryType::kSpline) {
    RepositionSpline(knots, n_q, N, x_lcs, repos_target, dt, is_doing_c3,
                     finished_reposition_flag, reposition_params,
                     sampling_c3_options);
  } else if (traj_type == RepositioningTrajectoryType::kSpherical) {
    RepositionSpherical(knots, n_q, N, x_lcs, repos_target, dt, is_doing_c3,
                        finished_reposition_flag, reposition_params,
                        sampling_c3_options);
  } else if (traj_type == RepositioningTrajectoryType::kCircular) {
    RepositionCircular(knots, n_q, N, x_lcs, repos_target, dt, is_doing_c3,
                       finished_reposition_flag, reposition_params);
  } else if (traj_type == RepositioningTrajectoryType::kPiecewiseLinear) {
    RepositionPiecewiseLinear(knots, N, x_lcs, repos_target, dt, is_doing_c3,
                              finished_reposition_flag, reposition_params,
                              adaptive_waypoint_height);
  }

  // Every strategy above pins the first knot to x0 (the current, predicted EE
  // state); the C3 execution path relies on the same invariant.  Assert it here
  // so a future strategy edit can't silently break plan/state continuity.
  DRAKE_DEMAND((knots.col(0).head(3) - x_lcs.head(3)).norm() < 1e-9);

  if (!allow_ground_penetration) {
    EnforceNoGroundPenetration(knots,
                               sampling_c3_options.workspace_limits[2][3] +
                                   sampling_c3_options.workspace_margins);
  }
  return knots;
}

void RepositionStraightLine(
    Eigen::MatrixXd& knots, const int& n_q, const int& n_x, const int& N,
    const Eigen::VectorXd& x_lcs, const Eigen::Vector3d& repos_target,
    const double& dt, const bool& is_doing_c3, bool& finished_reposition_flag,
    const SamplingC3RepositionParams& reposition_params) {
  Eigen::Vector3d current_ee_location = x_lcs.head(3);
  Eigen::Vector3d curr_to_goal_vec = repos_target - current_ee_location;
  double xy_travel_distance = curr_to_goal_vec.head(2).norm();
  double z_travel_distance = std::abs(curr_to_goal_vec(2));

  // Time the move so neither the horizontal nor the vertical axes are asked to
  // exceed their own max speed -- i.e. scale to whichever axis needs more time,
  // the same "slowest-required-axis" logic the printer driver uses
  // (github.com/DAIRLab/printer_robot_driver).
  double total_travel_time =
      std::max(xy_travel_distance / reposition_params.speed_horizontal,
               z_travel_distance / reposition_params.speed_vertical);
  // Ensure the denominator used for interpolation fractions is nonzero.
  double denom = std::max(total_travel_time, 0.0001);

  Eigen::VectorXd next_lcs_state = x_lcs;
  next_lcs_state.head(3) = repos_target;
  next_lcs_state.segment(n_q, 3) = Eigen::Vector3d::Zero();

  // Linearly interpolate from x_lcs to next_lcs_state directly, rather than
  // going through PiecewisePolynomial::FirstOrderHold(...).value(...) for what
  // is just a 2-point linear interpolation -- that path was found (via
  // AddressSanitizer) to read one byte past the end of a heap-allocated buffer
  // inside Drake's PiecewisePolynomial::do_value(), which could occasionally
  // pull in unrelated heap garbage and produce a knot far from the intended
  // straight-line path.
  for (int i = 0; i < N; i++) {
    double t_line = std::min((i)*dt, total_travel_time);
    double frac = t_line / denom;
    knots.col(i) = (1.0 - frac) * x_lcs + frac * next_lcs_state;

    // If one step gets to the goal, set finished_reposition_flag.
    if (i == 1 && t_line >= total_travel_time && !is_doing_c3) {
      finished_reposition_flag = true;
    }
  }
}

void RepositionSpline(Eigen::MatrixXd& knots, const int& n_q, const int& N,
                      const Eigen::VectorXd& x_lcs,
                      const Eigen::Vector3d& repos_target, const double& dt,
                      const bool& is_doing_c3, bool& finished_reposition_flag,
                      const SamplingC3RepositionParams& reposition_params,
                      const SamplingC3Options& sampling_c3_options) {
  Eigen::Vector3d current_ee_location = x_lcs.head(3);
  Eigen::Vector3d current_object_location = x_lcs.segment(n_q - 3, 3);
  Eigen::Vector3d curr_to_goal_vec = repos_target - current_ee_location;
  double travel_distance = curr_to_goal_vec.norm();

  // Compute spline waypoints.
  Eigen::Vector3d p0 = current_ee_location;
  Eigen::Vector3d p3 = repos_target;
  Eigen::Vector3d p1 =
      current_ee_location + 0.25 * curr_to_goal_vec - current_object_location;
  p1 =
      current_object_location + reposition_params.spline_width * p1 / p1.norm();
  Eigen::Vector3d p2 =
      current_ee_location + 0.75 * curr_to_goal_vec - current_object_location;
  p2 =
      current_object_location + reposition_params.spline_width * p2 / p2.norm();

  for (int i = 0; i < N; i++) {
    double total_travel_time =
        travel_distance / reposition_params.speed_horizontal;
    double t_spline = (i)*dt / total_travel_time;
    t_spline = std::min(1.0, t_spline);  // Don't overshoot end of the spline.

    // Set finished_reposition_flag if only one step is required.
    if (i == 1 && t_spline == 1 && !is_doing_c3) {
      finished_reposition_flag = true;
      std::cout << "WARNING! Spline finished repositioning in 1 step."
                << std::endl;
    }

    Eigen::Vector3d next_ee_loc =
        p0 + t_spline * (-3 * p0 + 3 * p1) +
        std::pow(t_spline, 2) * (3 * p0 - 6 * p1 + 3 * p2) +
        std::pow(t_spline, 3) * (-p0 + 3 * p1 - 3 * p2 + p3);

    // Set the next LCS state as the current state with updated end effector
    // location and zero end effector velocity. Note that this means that the
    // object does not move in the planned trajectory. TODO An alternative is to
    // simulate the object's motion with 0 input.
    Eigen::VectorXd next_lcs_state = x_lcs;
    next_lcs_state.head(3) = next_ee_loc;
    next_lcs_state.segment(n_q, 3) = Eigen::Vector3d::Zero();
    // If z is under the table, set it to a min height.
    if (next_lcs_state[2] < sampling_c3_options.workspace_limits[2][3] +
                                sampling_c3_options.workspace_margins) {
      next_lcs_state[2] = sampling_c3_options.workspace_limits[2][3] +
                          sampling_c3_options.workspace_margins;
    }

    knots.col(i) = next_lcs_state;
  }
}

void RepositionSpherical(Eigen::MatrixXd& knots, const int& n_q, const int& N,
                         const Eigen::VectorXd& x_lcs,
                         const Eigen::Vector3d& repos_target, const double& dt,
                         const bool& is_doing_c3,
                         bool& finished_reposition_flag,
                         const SamplingC3RepositionParams& reposition_params,
                         const SamplingC3Options& sampling_c3_options) {
  Eigen::Vector3d current_ee_location = x_lcs.head(3);
  Eigen::Vector3d current_object_location = x_lcs.segment(n_q - 3, 3);

  // Get two unit vectors in the plane of the arc between the current and goal
  // ee locations.
  Eigen::Vector3d v1 =
      (current_ee_location - current_object_location).normalized();
  Eigen::Vector3d v2 = (repos_target - current_object_location).normalized();
  // NOTE:  Need to clamp between not quite (-1, 1) to avoid NaNs.
  double travel_angle = std::acos(std::clamp(v1.dot(v2), -0.9999, 0.9999));

  // Get the two waypoints as the ends of the arc trajectory.
  Eigen::Vector3d waypoint1 =
      current_object_location + v1 * reposition_params.sphere_radius;
  Eigen::Vector3d waypoint2 =
      current_object_location + v2 * reposition_params.sphere_radius;

  Eigen::Vector3d v3 = v1.cross(v2).normalized();
  Eigen::Vector3d v4 = v3.cross(v1).normalized();

  // Ensure the traversed arc stays above the ground.
  if (v4(2) < -0.5 && travel_angle > M_PI_2) {
    v4 = -v4;
    travel_angle = 2 * M_PI - travel_angle;
  }
  // Prevent getting stuck if the EE is 180 degrees around from the target.
  if (travel_angle > 0.9 * M_PI) {
    Eigen::Vector3d almost_v4 = v1;
    almost_v4(2) += 1;
    v3 = v1.cross(almost_v4.normalized()).normalized();
    v4 = v3.cross(v1).normalized();
  }

  // The arc is defined by the equation:
  // x = x_c + r*cos(theta)*v1 + r*sin(theta)*v4
  // ...where theta should be [0, travel_angle] and r*dtheta should be the
  // desired travel distance.
  double dtheta =
      reposition_params.speed_horizontal * dt / reposition_params.sphere_radius;
  double step_size = reposition_params.speed_horizontal * dt;

  knots.col(0) = x_lcs;
  int i = 1;
  // Handle the first leg:  straight line from current EE location to waypoint1.
  double dist_to_wp1 = (current_ee_location - waypoint1).norm();
  while ((i * step_size < dist_to_wp1) && (i < N)) {
    Eigen::Vector3d straight_line_point =
        current_ee_location +
        i * step_size / dist_to_wp1 * (waypoint1 - current_ee_location);

    Eigen::VectorXd next_lcs_state = x_lcs;
    next_lcs_state.head(3) = straight_line_point;
    knots.col(i) = next_lcs_state;
    i++;
  }

  // Handle the second leg:  arc from waypoint1 to waypoint2.
  int leg1_i = i;
  double dtheta0 = (i * step_size - dist_to_wp1) / step_size * dtheta;
  while ((dtheta0 + (i - leg1_i) * dtheta < travel_angle) && (i < N)) {
    Eigen::Vector3d arc_point =
        current_object_location +
        reposition_params.sphere_radius *
            (std::cos(dtheta0 + (i - leg1_i) * dtheta) * v1 +
             std::sin(dtheta0 + (i - leg1_i) * dtheta) * v4);

    Eigen::VectorXd next_lcs_state = x_lcs;
    next_lcs_state.head(3) = arc_point;
    knots.col(i) = next_lcs_state;
    i++;
  }

  // Handle the last leg:  straight line from waypoint2 to goal EE location.
  int leg2_i = i;
  double dstep =
      (dtheta0 + (i - leg1_i) * dtheta - travel_angle) / dtheta * step_size;
  double dist_wp2_to_goal = (waypoint2 - repos_target).norm();
  while ((dstep + (i - leg2_i) * step_size < dist_wp2_to_goal) && (i < N)) {
    Eigen::Vector3d straight_line_point =
        waypoint2 + (dstep + (i - leg2_i) * step_size) / dist_wp2_to_goal *
                        (repos_target - waypoint2);

    Eigen::VectorXd next_lcs_state = x_lcs;
    next_lcs_state.head(3) = straight_line_point;
    knots.col(i) = next_lcs_state;
    i++;
  }

  // Enforce minimum z height for end effector.
  for (int j = 0; j < i; j++) {
    if (knots(2, j) < sampling_c3_options.workspace_limits[2][3] +
                          sampling_c3_options.workspace_margins) {
      knots(2, j) = sampling_c3_options.workspace_limits[2][3] +
                    sampling_c3_options.workspace_margins;
    }
  }

  // Fill in the rest of the knots with the goal EE location.
  for (int j = i; j < N; j++) {
    Eigen::Vector3d x_lcs_goal = x_lcs;
    x_lcs_goal.head(3) = repos_target;
    knots.col(j) = x_lcs_goal;
    if (j == 1 && !is_doing_c3) {
      finished_reposition_flag = true;
    }
  }
}

void RepositionCircular(Eigen::MatrixXd& knots, const int& n_q, const int& N,
                        const Eigen::VectorXd& x_lcs,
                        const Eigen::Vector3d& repos_target, const double& dt,
                        const bool& is_doing_c3, bool& finished_reposition_flag,
                        const SamplingC3RepositionParams& reposition_params) {
  Eigen::Vector3d current_ee_location = x_lcs.head(3);
  Eigen::Vector3d current_object_location = x_lcs.segment(n_q - 3, 3);

  // Project positions to the plane of the circle.
  Eigen::Vector3d current_object_projection = current_object_location;
  current_object_projection(2) = reposition_params.circle_height;
  Eigen::Vector3d curr_ee_projection = current_ee_location;
  curr_ee_projection(2) = reposition_params.circle_height;
  Eigen::Vector3d best_sample_projection = repos_target;
  best_sample_projection(2) = reposition_params.circle_height;

  Eigen::Vector3d v1 =
      (curr_ee_projection - current_object_projection).normalized();
  Eigen::Vector3d v2 =
      (best_sample_projection - current_object_projection).normalized();

  // Compute travel angle.
  double travel_angle = std::acos(v1.dot(v2));

  // Define the waypoints for the circular trajectory.
  Eigen::Vector3d waypoint1 =
      current_object_projection + reposition_params.circle_radius * v1;
  Eigen::Vector3d waypoint2 =
      current_object_projection + reposition_params.circle_radius * v2;

  // Compute tangent vector to the circle at waypoint1.
  Eigen::Vector3d v3 = v1.cross(v2).normalized();
  Eigen::Vector3d v4 = v3.cross(v1).normalized();

  if (travel_angle > M_PI) {
    travel_angle = 2 * M_PI - travel_angle;
    v4 = -v4;
  }

  // The arc is defined by the equation: x = x_c + r*cos(theta) + r*sin(theta)
  // the relation between the linear velocity and the angular velocity is
  // given by v = r*omega, where v is the linear velocity and omega is the
  // angular velocity. The angular velocity is given by omega = v/r. The
  // angular velocity is given by omega = dtheta/dt. Therefore, dtheta =
  // v/r*dt. This is the increment in the angle in the time dt.
  double dtheta =
      reposition_params.speed_horizontal * dt / reposition_params.circle_radius;
  double step_size = reposition_params.speed_horizontal * dt;

  knots.col(0) = x_lcs;
  int i = 1;
  // Handle the first leg:  straight line from current EE location to
  // waypoint1.
  double dist_to_wp1 = (current_ee_location - waypoint1).norm();
  while ((i * step_size < dist_to_wp1) && (i < N)) {
    Eigen::Vector3d straight_line_point =
        current_ee_location +
        i * step_size / dist_to_wp1 * (waypoint1 - current_ee_location);

    Eigen::VectorXd next_lcs_state = x_lcs;
    next_lcs_state.head(3) = straight_line_point;
    knots.col(i) = next_lcs_state;
    i++;
  }

  // Handle the second leg:  arc from waypoint1 to waypoint2.
  int leg1_i = i;
  double dtheta0 = (i * step_size - dist_to_wp1) / step_size * dtheta;
  while (((i - leg1_i) * dtheta < travel_angle) && (i < N)) {
    Eigen::Vector3d arc_point =
        current_object_projection +
        reposition_params.circle_radius *
            (std::cos(dtheta0 + (i - leg1_i) * dtheta) * v1 +
             std::sin(dtheta0 + (i - leg1_i) * dtheta) * v4);
    arc_point(2) = reposition_params.circle_height;

    Eigen::VectorXd next_lcs_state = x_lcs;
    next_lcs_state.head(3) = arc_point;
    knots.col(i) = next_lcs_state;
    i++;
  }

  // Handle the last leg:  straight line from waypoint2 to goal EE location.
  int leg2_i = i;
  double dstep =
      (dtheta0 + (i - leg1_i) * dtheta - travel_angle) / dtheta * step_size;
  double dist_wp2_to_goal = (waypoint2 - repos_target).norm();
  while ((dstep + (i - leg2_i) * step_size < dist_wp2_to_goal) && (i < N)) {
    Eigen::Vector3d straight_line_point =
        waypoint2 + (dstep + (i - leg2_i) * step_size) / dist_wp2_to_goal *
                        (repos_target - waypoint2);

    Eigen::VectorXd next_lcs_state = x_lcs;
    next_lcs_state.head(3) = straight_line_point;
    knots.col(i) = next_lcs_state;
    i++;
  }

  // Fill in the rest of the knots with the goal EE location.
  for (int j = i; j < N; j++) {
    Eigen::Vector3d x_lcs_goal = x_lcs;
    x_lcs_goal.head(3) = repos_target;
    knots.col(j) = x_lcs_goal;
    if (j == 1 && !is_doing_c3) {
      finished_reposition_flag = true;
    }
  }
}

// The piecewise linear trajectory uses three legs:  go up from current location
// to a repositioning (cruise) height, move in a straight line to right above
// the sample, then move down to the sample.  adaptive_waypoint_height is the
// cruise height to rise to; when adaptive piecewise-linear repositioning is
// disabled it is just reposition_params.pwl_waypoint_height.
void RepositionPiecewiseLinear(
    Eigen::MatrixXd& knots, const int& N, const Eigen::VectorXd& x_lcs,
    const Eigen::Vector3d& repos_target, const double& dt,
    const bool& is_doing_c3, bool& finished_reposition_flag,
    const SamplingC3RepositionParams& reposition_params,
    const double& adaptive_waypoint_height) {
  Eigen::Vector3d current_ee_location = x_lcs.head(3);

  // Cruise at the requested height, but never dip below the current EE height
  // to get there (that would add a needless slow vertical leg).  When the EE is
  // already at or above the cruise height, dist_leg1 is 0 and the loop below
  // starts translating immediately.
  double cruise_z =
      std::max(current_ee_location(2), adaptive_waypoint_height);

  // Define the waypoints for the three-leg repositioning.
  Eigen::Vector3d waypoint_above_ee = current_ee_location;
  waypoint_above_ee(2) = cruise_z;
  Eigen::Vector3d waypoint_above_sample = repos_target;
  waypoint_above_sample(2) = cruise_z;

  // Legs 1 and 3 are purely vertical (only z changes); leg 2 is purely
  // horizontal (both endpoints are at the cruise height). Time each leg by
  // its own axis's speed, then parametrize the whole three-leg path by elapsed
  // time -- the same approach RepositionStraightLine() uses -- rather than
  // stepping knot-by-knot with a leftover distance carried from one leg into
  // the next: a leftover computed at one leg's step size doesn't translate
  // correctly into a leg with a different step size.
  double dist_leg1 = (current_ee_location - waypoint_above_ee).norm();
  double dist_leg2 = (waypoint_above_ee - waypoint_above_sample).norm();
  // This leg is really unused in practice because Reposition() enters the
  // use_straight_line condition before the EE gets this close to the target.
  double dist_leg3 = (waypoint_above_sample - repos_target).norm();
  double time_leg1 = dist_leg1 / reposition_params.speed_vertical;
  double time_leg2 = dist_leg2 / reposition_params.speed_horizontal;
  double time_leg3 = dist_leg3 / reposition_params.speed_vertical;
  double total_time = time_leg1 + time_leg2 + time_leg3;

  knots.col(0) = x_lcs;
  for (int i = 1; i < N; i++) {
    double t = i * dt;
    Eigen::Vector3d point;
    if (t < time_leg1) {
      double frac = (time_leg1 > 0.0) ? (t / time_leg1) : 1.0;
      point = current_ee_location +
              frac * (waypoint_above_ee - current_ee_location);
    } else if (t < time_leg1 + time_leg2) {
      double frac = (time_leg2 > 0.0) ? ((t - time_leg1) / time_leg2) : 1.0;
      point = waypoint_above_ee +
              frac * (waypoint_above_sample - waypoint_above_ee);
    } else if (t < total_time) {
      double frac =
          (time_leg3 > 0.0) ? ((t - time_leg1 - time_leg2) / time_leg3) : 1.0;
      point =
          waypoint_above_sample + frac * (repos_target - waypoint_above_sample);
    } else {
      point = repos_target;
      if (i == 1 && !is_doing_c3) {
        finished_reposition_flag = true;
      }
    }

    Eigen::VectorXd next_lcs_state = x_lcs;
    next_lcs_state.head(3) = point;
    knots.col(i) = next_lcs_state;
  }
}

void EnforceNoGroundPenetration(Eigen::MatrixXd& knots, double min_z) {
  for (int i = 0; i < knots.cols(); i++) {
    if (knots(2, i) < min_z) {
      knots(2, i) = min_z;
    }
  }
}

std::pair<bool, double> ComputeRepositionClearance(
    const drake::geometry::QueryObject<double>& query_object,
    drake::geometry::GeometryId ee_geometry_id,
    const Eigen::Vector3d& current_ee_location, const Eigen::Vector3d& target,
    double ee_radius, const SamplingC3RepositionParams& reposition_params,
    const SamplingC3Options& sampling_c3_options) {
  const double clearance = sampling_c3_options.workspace_margins + ee_radius +
                           reposition_params.pwl_clearance_margin;

  // A point is clear iff nothing except the EE's own geometry lies within
  // `clearance` of it.  Passing `clearance` as the query threshold means the
  // result only contains geometries that are already too close.
  auto point_is_clear = [&](const Eigen::Vector3d& p) {
    const auto& results =
        query_object.ComputeSignedDistanceToPoint(p, clearance);
    for (const auto& result : results) {
      if (result.id_G != ee_geometry_id) {
        return false;
      }
    }
    return true;
  };
  const int num_samples =
      std::max(1, reposition_params.pwl_num_path_collision_samples);
  auto segment_is_clear = [&](const Eigen::Vector3d& a,
                              const Eigen::Vector3d& b) {
    for (int i = 0; i <= num_samples; ++i) {
      const double s = static_cast<double>(i) / num_samples;
      if (!point_is_clear(a + s * (b - a))) {
        return false;
      }
    }
    return true;
  };

  const bool direct_path_clear = segment_is_clear(current_ee_location, target);

  const double floor_z = sampling_c3_options.workspace_limits[2][3] +
                         sampling_c3_options.workspace_margins;
  const double lo =
      std::max({current_ee_location(2), target(2), floor_z});
  const double hi = std::max(reposition_params.pwl_waypoint_height, lo);
  const double step = std::max(1e-3, reposition_params.pwl_height_search_step);
  double min_cruise_height = hi;  // fallback if no lower height is clear
  for (double z = lo; z <= hi + 1e-9; z += step) {
    const Eigen::Vector3d a(current_ee_location(0), current_ee_location(1), z);
    const Eigen::Vector3d c(target(0), target(1), z);
    if (segment_is_clear(a, c)) {
      min_cruise_height = z;
      break;
    }
  }
  return {direct_path_clear, min_cruise_height};
}

}  // namespace systems
}  // namespace dairlib
