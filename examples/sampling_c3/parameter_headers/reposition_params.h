#pragma once

#include "common/file_utils.h"

#include "drake/common/yaml/yaml_read_archive.h"

/* Repositioning trajectory type options:
  0. kSpline:           move from current to target with spline that distorts
                        around the object.
  1. kSpherical:        move away from object to a radius, along the spherical
                        surface to outside target, then in to target.
  2. kCircular:         (planar) move away from object to a planar radius,
                        along the circle to outside target, then in to target.
  3. kPiecewiseLinear:  move up to waypoint height, over to above target, then
                        down.
*/
enum RepositioningTrajectoryType {
  kSpline,
  kSpherical,
  kCircular,
  kPiecewiseLinear
};

struct SamplingC3RepositionParams {
  RepositioningTrajectoryType traj_type;

  /// Parameters used for multiple repositioning trajectory types.
  double speed_horizontal;
  double speed_vertical;

  /// Thresholds for switching to straight line trajectories.
  double use_straight_line_traj_under_spline;
  double use_straight_line_traj_within_angle;
  double use_straight_line_traj_under_piecewise_linear;

  /// Spline-specific parameters.
  double spline_width;

  /// Sphere-specific parameters.
  double sphere_radius;

  /// Circle-specific parameters.
  double circle_radius;
  double circle_height;

  /// Piecewise-linear-specific parameters.
  double pwl_waypoint_height;

  double max_tilt_angle;  // angle of ee tilt in degrees

  template <typename Archive>
  void Serialize(Archive* a) {
    ENUM_DESERIALIZE(a, traj_type);
    a->Visit(DRAKE_NVP(speed_horizontal));
    a->Visit(DRAKE_NVP(speed_vertical));
    a->Visit(DRAKE_NVP(use_straight_line_traj_under_spline));
    a->Visit(DRAKE_NVP(use_straight_line_traj_within_angle));
    a->Visit(DRAKE_NVP(use_straight_line_traj_under_piecewise_linear));
    a->Visit(DRAKE_NVP(spline_width));
    a->Visit(DRAKE_NVP(sphere_radius));
    a->Visit(DRAKE_NVP(circle_radius));
    a->Visit(DRAKE_NVP(circle_height));
    a->Visit(DRAKE_NVP(pwl_waypoint_height));
    a->Visit(DRAKE_NVP(max_tilt_angle));
  }
};
