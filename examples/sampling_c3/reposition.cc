#include "reposition.h"

#include "drake/common/trajectories/piecewise_polynomial.h"

namespace dairlib {

using drake::trajectories::PiecewisePolynomial;

namespace systems {

Eigen::MatrixXd Reposition(
    const int& n_q,
    const int& n_x,
    const int& N,
    const Eigen::VectorXd& x_lcs,
    const Eigen::Vector3d& repos_target,
    const double& dt,
    const bool& is_doing_c3,
    bool& finished_reposition_flag,
    const SamplingParams& sampling_params,
    const SamplingC3Options& sampling_c3_options) {

  Eigen::MatrixXd knots = Eigen::MatrixXd::Zero(n_x, N);

  // Get the current end effector location.
  Eigen::Vector3d current_ee_location = x_lcs.head(3);
  Eigen::Vector3d current_object_location = x_lcs.segment(n_q-3, 3);

  Eigen::Vector3d curr_to_goal_vec = repos_target - current_ee_location;

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
  RepositioningTrajectoryType traj_type =
    sampling_params.repositioning_trajectory_type;
  if ((travel_distance < sampling_params.use_straight_line_traj_under &&
       traj_type == RepositioningTrajectoryType::kSpline) ||
      ((traj_type == RepositioningTrajectoryType::kSpherical ||
        traj_type == RepositioningTrajectoryType::kCircular) &&
       travel_angle < sampling_params.use_straight_line_traj_within_angle) ||
      (traj_type == RepositioningTrajectoryType::kPiecewiseLinear &&
       xy_travel_distance <
         sampling_params.use_straight_line_traj_under_piecewise_linear)) {
    RepositionStraightLine(
      knots, n_q, n_x, N, x_lcs, repos_target, dt, is_doing_c3,
      finished_reposition_flag, sampling_params);
  }
  else if (traj_type == RepositioningTrajectoryType::kSpline) {
    RepositionSpline(
      knots, n_q, N, x_lcs, repos_target, dt, is_doing_c3,
      finished_reposition_flag, sampling_params, sampling_c3_options);
  }
  else if (traj_type == RepositioningTrajectoryType::kSpherical) {
    RepositionSpherical(
      knots, n_q, N, x_lcs, repos_target, dt, is_doing_c3,
      finished_reposition_flag, sampling_params, sampling_c3_options);
  }
  else if (traj_type == RepositioningTrajectoryType::kCircular) {
    RepositionCircular(
      knots, n_q, N, x_lcs, repos_target, dt, is_doing_c3,
      finished_reposition_flag, sampling_params);
  }
  else if (traj_type == RepositioningTrajectoryType::kPiecewiseLinear) {
    RepositionPiecewiseLinear(
      knots, N, x_lcs, repos_target, dt, is_doing_c3, finished_reposition_flag,
      sampling_params);
  }

  return knots;
}

void RepositionStraightLine(
    Eigen::MatrixXd& knots, const int& n_q, const int& n_x, const int& N,
    const Eigen::VectorXd& x_lcs, const Eigen::Vector3d& repos_target,
    const double& dt, const bool& is_doing_c3, bool& finished_reposition_flag,
    const SamplingParams& sampling_params) {
  // Extract a few values.
  Eigen::Vector3d current_ee_location = x_lcs.head(3);
  Eigen::Vector3d curr_to_goal_vec = repos_target - current_ee_location;
  double travel_distance = curr_to_goal_vec.norm();

  Eigen::VectorXd times = Eigen::VectorXd::Zero(2);
  times[0] = 0;
  // Ensure the times used to define PiecewisePolynomial are increasing.
  double total_travel_time = travel_distance/sampling_params.reposition_speed;
  times[1] = std::max(total_travel_time, 0.0001);

  Eigen::MatrixXd points = Eigen::MatrixXd::Zero(n_x, 2);
  points.col(0) = x_lcs;
  Eigen::VectorXd next_lcs_state = x_lcs;
  next_lcs_state.head(3) = repos_target;
  next_lcs_state.segment(n_q, 3) = Eigen::Vector3d::Zero();
  points.col(1) = next_lcs_state;
  auto trajectory =
    PiecewisePolynomial<double>::FirstOrderHold(times, points);

  for (int i = 0; i < N; i++) {
    double t_line = std::min((i)*dt, total_travel_time);
    knots.col(i) = trajectory.value(t_line);

    // If one step gets to the goal, set finished_reposition_flag.
    if (i == 1 && t_line >= total_travel_time && !is_doing_c3) {
      finished_reposition_flag = true;
    }
  }
}

void RepositionSpline(
    Eigen::MatrixXd& knots, const int& n_q, const int& N,
    const Eigen::VectorXd& x_lcs, const Eigen::Vector3d& repos_target,
    const double& dt, const bool& is_doing_c3, bool& finished_reposition_flag,
    const SamplingParams& sampling_params,
    const SamplingC3Options& sampling_c3_options) {
  // Extract a few values.
  Eigen::Vector3d current_ee_location = x_lcs.head(3);
  Eigen::Vector3d current_object_location = x_lcs.segment(n_q-3, 3);
  Eigen::Vector3d curr_to_goal_vec = repos_target - current_ee_location;
  double travel_distance = curr_to_goal_vec.norm();

  // Compute spline waypoints.
  Eigen::Vector3d p0 = current_ee_location;
  Eigen::Vector3d p3 = repos_target;
  Eigen::Vector3d p1 =
      current_ee_location + 0.25 * curr_to_goal_vec - current_object_location;
  p1 = current_object_location +
        sampling_params.spline_width * p1 / p1.norm();
  Eigen::Vector3d p2 =
      current_ee_location + 0.75 * curr_to_goal_vec - current_object_location;
  p2 = current_object_location +
        sampling_params.spline_width * p2 / p2.norm();

  for (int i = 0; i < N; i++) {
    // This is a curve_fraction and is not in the units of time or distance.
    // When it is 0, it is the current location. When it is 1, it is the goal.
    double total_travel_time = travel_distance /
      sampling_params.reposition_speed;
    double t_spline = (i)*dt / total_travel_time;

    if (i == 1 && t_spline >= 1 && !is_doing_c3) {
      // If it can get there in one step, then set finished_reposition_flag
      // to true.
      finished_reposition_flag = true;
      std::cout
        << "WARNING! Using spline but finished repositioning in one step."
        << std::endl;
    }
    // Don't overshoot the end of the spline.
    t_spline = std::min(1.0, t_spline);

    Eigen::Vector3d next_ee_loc =
      p0 + t_spline * (-3 * p0 + 3 * p1) +
      std::pow(t_spline, 2) * (3 * p0 - 6 * p1 + 3 * p2) +
      std::pow(t_spline, 3) * (-p0 + 3 * p1 - 3 * p2 + p3);

    // Set the next LCS state as the current state with updated end effector
    // location and zero end effector velocity. Note that this means that the
    // object does not move in the planned trajectory. An alternative is that
    // we could simulate the object's motion with 0 input.
    Eigen::VectorXd next_lcs_state = x_lcs;
    next_lcs_state.head(3) = next_ee_loc;
    next_lcs_state.segment(n_q, 3) = Eigen::Vector3d::Zero();
    // If z is under the table, set it to a min height.
    if (next_lcs_state[2] < sampling_c3_options.workspace_limits[2][3]) {
      next_lcs_state[2] = sampling_c3_options.workspace_limits[2][3];
    }

    knots.col(i) = next_lcs_state;
  }
}

void RepositionSpherical(
    Eigen::MatrixXd& knots, const int& n_q, const int& N,
    const Eigen::VectorXd& x_lcs, const Eigen::Vector3d& repos_target,
    const double& dt, const bool& is_doing_c3, bool& finished_reposition_flag,
    const SamplingParams& sampling_params,
    const SamplingC3Options& sampling_c3_options) {
  // Extract a few values.
  Eigen::Vector3d current_ee_location = x_lcs.head(3);
  Eigen::Vector3d current_object_location = x_lcs.segment(n_q-3, 3);

  // Get two unit vectors in the plane of the arc between the current and goal
  // ee locations.
  Eigen::Vector3d v1 =
    (current_ee_location - current_object_location).normalized();
  Eigen::Vector3d v2 = (repos_target - current_object_location).normalized();
  // NOTE:  Need to clamp between not quite (-1, 1) to avoid NaNs.
  double travel_angle = std::acos(std::clamp(v1.dot(v2), -0.9999, 0.9999));

  // Get the two waypoints as the ends of the arc trajectory.
  Eigen::Vector3d waypoint1 =
    current_object_location +
    v1 * sampling_params.spherical_repositioning_radius;
  Eigen::Vector3d waypoint2 =
    current_object_location +
    v2 * sampling_params.spherical_repositioning_radius;

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
  double dtheta = sampling_params.reposition_speed * dt /
                  sampling_params.spherical_repositioning_radius;
  double step_size = sampling_params.reposition_speed * dt;

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
      sampling_params.spherical_repositioning_radius *
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

  // Enforce minimum z height for end effector, with a small buffer.
  for (int j = 0; j < i; j++) {
    if (knots(2, j) < sampling_c3_options.workspace_limits[2][3]) {
      knots(2, j) = sampling_c3_options.workspace_limits[2][3];
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

void RepositionCircular(
    Eigen::MatrixXd& knots, const int& n_q, const int& N,
    const Eigen::VectorXd& x_lcs, const Eigen::Vector3d& repos_target,
    const double& dt, const bool& is_doing_c3, bool& finished_reposition_flag,
    const SamplingParams& sampling_params) {
  // Extract a few values.
  Eigen::Vector3d current_ee_location = x_lcs.head(3);
  Eigen::Vector3d current_object_location = x_lcs.segment(n_q-3, 3);

  // Current object projection onto the plane of the circle.
  Eigen::Vector3d current_object_projection = current_object_location;
  current_object_projection(2) =
    sampling_params.circular_repositioning_height;

  // Project current ee location onto the plane of the circle.
  Eigen::Vector3d curr_ee_projection = current_ee_location;
  curr_ee_projection(2) = sampling_params.circular_repositioning_height;

  // project best sample onto the repositioning plane.
  Eigen::Vector3d best_sample_projection = repos_target;
  best_sample_projection(2) = sampling_params.circular_repositioning_height;

  Eigen::Vector3d v1 =
    (curr_ee_projection - current_object_projection).normalized();
  Eigen::Vector3d v2 =
    (best_sample_projection - current_object_projection).normalized();

  // Compute travel angle
  double travel_angle = std::acos(v1.dot(v2));

  // Define the waypoints for the circular trajectory.
  Eigen::Vector3d waypoint1 =
    current_object_projection +
    sampling_params.circular_repositioning_radius * v1;
  Eigen::Vector3d waypoint2 =
    current_object_projection +
    sampling_params.circular_repositioning_radius * v2;

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
  double dtheta = sampling_params.reposition_speed * dt /
                  sampling_params.circular_repositioning_radius;
  double step_size = sampling_params.reposition_speed * dt;

  knots.col(0) = x_lcs;
  int i = 1;
  // Handle the first leg:  straight line from current EE location to
  // waypoint1.
  double dist_to_wp1 = (current_ee_location - waypoint1).norm();
  while ((i * step_size < dist_to_wp1) && (i < N)) {
    // The division by dist_to_wp1 is to normalize the direction vector from
    // the current ee location to waypoint1.
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
      sampling_params.circular_repositioning_radius *
        (std::cos(dtheta0 + (i - leg1_i) * dtheta) * v1 +
          std::sin(dtheta0 + (i - leg1_i) * dtheta) * v4);
    arc_point(2) = sampling_params.circular_repositioning_height;

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
// to a fixed repositioning height, move in a straight line to right above the
// sample, then move down to the sample.
void RepositionPiecewiseLinear(
    Eigen::MatrixXd& knots, const int& N, const Eigen::VectorXd& x_lcs,
    const Eigen::Vector3d& repos_target, const double& dt,
    const bool& is_doing_c3, bool& finished_reposition_flag,
    const SamplingParams& sampling_params) {
  // Extract a few values.
  Eigen::Vector3d current_ee_location = x_lcs.head(3);

  // Define the waypoints for the three-leg repositioning.
  Eigen::Vector3d waypoint_above_ee = current_ee_location;
  waypoint_above_ee(2) = sampling_params.repositioning_waypoint_height;

  Eigen::Vector3d waypoint_above_sample = repos_target;
  waypoint_above_sample(2) = sampling_params.repositioning_waypoint_height;

  knots.col(0) = x_lcs;
  int i = 1;
  double step_size = sampling_params.reposition_speed * dt;

  // First leg: straight line from current EE location to waypoint_above_ee.
  double dist_to_wp1 = (current_ee_location - waypoint_above_ee).norm();
  while ((i * step_size < dist_to_wp1) && (i < N)) {
    Eigen::Vector3d straight_line_point =
      current_ee_location + i * step_size / dist_to_wp1 *
                                (waypoint_above_ee - current_ee_location);

    Eigen::VectorXd next_lcs_state = x_lcs;
    next_lcs_state.head(3) = straight_line_point;
    knots.col(i) = next_lcs_state;
    i++;
  }

  // Second leg: straight line from waypoint_above_ee to
  // waypoint_above_sample.
  int leg1_i = i;
  double dstep0 = i * step_size - dist_to_wp1;
  double dist_to_wp2 = (waypoint_above_ee - waypoint_above_sample).norm();
  while ((dstep0 + (i - leg1_i) * step_size < dist_to_wp2) && (i < N)) {
    Eigen::Vector3d straight_line_point =
      waypoint_above_ee + (dstep0 + (i - leg1_i) * step_size) / dist_to_wp2 *
      (waypoint_above_sample - waypoint_above_ee);

    Eigen::VectorXd next_lcs_state = x_lcs;
    next_lcs_state.head(3) = straight_line_point;
    knots.col(i) = next_lcs_state;
    i++;
  }

  // Third leg: straight line from waypoint_above_sample to
  // repos_target. Really this doesn't even get used because it enters
  // the use_straight_line condition before then.
  int leg2_i = i;
  double dstep1 = (i - leg1_i) * step_size - dist_to_wp2;
  double dist_to_goal = (waypoint_above_sample - repos_target).norm();
  while ((dstep1 + (i - leg2_i) * step_size < dist_to_goal) && (i < N)) {
    Eigen::Vector3d straight_line_point =
      waypoint_above_sample +
      (dstep1 + (i - leg2_i) * step_size) / dist_to_goal *
      (repos_target - waypoint_above_sample);

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

}   // namespace systems
}   // namespace dairlib
