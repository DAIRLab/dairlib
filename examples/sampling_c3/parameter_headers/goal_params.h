#pragma once

#include <numeric>

#include "common/file_utils.h"

#include "drake/common/yaml/yaml_read_archive.h"

/* Goal mode options:
  0. kRandom:               randomly generate a new goal.
  1. kOrientationSequence:  keep position goal the same, and cycle through a
                            sequence of orientations.
  2. kFixedGoal:            keep the same goal.
*/
enum GoalMode { kRandom, kOrientationSequence, kFixedGoal };

struct SamplingC3GoalParams {
  GoalMode goal_mode;

  /// Parameters used for multiple goal modes.
  /// Success thresholds for position and orientation.
  double position_success_threshold;
  double orientation_success_threshold;
  bool only_use_xy_position;

  std::vector<double> resting_object_heights;  // in world frame for each object
  double ee_target_z_offset_above_object;  // defines EE goal wrt object height
  bool ignore_roll_when_tracking_orientation;  // ignore roll when computing orientation error

  /// Lookahead parameters to define a sub-goal for C3.
  double lookahead_step_size;
  double lookahead_angle;

  /// Apply hysteresis on the lookahead angle so the sub-goal orientation does
  /// not flip back and forth near the 180 degree error singularity.  A lower
  /// number means the sub-goal orientation can switch more often; a higher
  /// number means the sub-goal orientation is more stable but possibly less
  /// optimal.
  double angle_hysteresis;

  /// Factor to convert an angular error to angular velocity command (a value of
  /// 0 disables this feature).)
  double angle_err_to_vel_factor;

  /// Initial goal (and only goal for fixed goal mode).
  std::vector<Eigen::Vector3d> fixed_target_positions;
  std::vector<Eigen::Vector4d> fixed_target_orientations;

  std::vector<std::pair<double, double>> sampling_area_y_limits;
  std::vector<int> default_object_index_to_sampling_area_index_map;

  /// Random-specific parameters.
  Eigen::VectorXd random_goal_x_limits;
  Eigen::VectorXd random_goal_y_limits;
  Eigen::VectorXd random_goal_radius_limits;

  int random_goal_gen_max_attempts;
  double pairwise_goal_distance;

  template <typename Archive>
  void Serialize(Archive* a) {
    ENUM_DESERIALIZE(a, goal_mode);
    a->Visit(DRAKE_NVP(position_success_threshold));
    a->Visit(DRAKE_NVP(orientation_success_threshold));
    a->Visit(DRAKE_NVP(resting_object_heights));
    a->Visit(DRAKE_NVP(ee_target_z_offset_above_object));
    a->Visit(DRAKE_NVP(ignore_roll_when_tracking_orientation));
    a->Visit(DRAKE_NVP(lookahead_step_size));
    a->Visit(DRAKE_NVP(lookahead_angle));
    a->Visit(DRAKE_NVP(angle_hysteresis));
    a->Visit(DRAKE_NVP(angle_err_to_vel_factor));
    a->Visit(DRAKE_NVP(fixed_target_positions));
    a->Visit(DRAKE_NVP(fixed_target_orientations));
    a->Visit(DRAKE_NVP(random_goal_x_limits));
    a->Visit(DRAKE_NVP(random_goal_y_limits));
    a->Visit(DRAKE_NVP(random_goal_radius_limits));
    a->Visit(DRAKE_NVP(random_goal_gen_max_attempts));
    a->Visit(DRAKE_NVP(pairwise_goal_distance));
    a->Visit(DRAKE_NVP(only_use_xy_position));
    ComputeSamplingAreaYLimits();
    SetDefaultObjectIndexToSamplingAreaIndexMap();
  }

 private:
  // Compute y limits for each sampling area of each object
  // Note: all sampling areas have the same x limits
  void ComputeSamplingAreaYLimits() {
    int num_objects = fixed_target_positions.size();
    double step =
        (random_goal_y_limits[1] - random_goal_y_limits[0]) / num_objects;
    for (int i = 0; i < num_objects; ++i) {
      sampling_area_y_limits.push_back(
          std::make_pair(random_goal_y_limits[0] + i * step,
                         random_goal_y_limits[0] + (i + 1) * step));
    }
  }

  // By default, object index ith will be in sampling area ith
  void SetDefaultObjectIndexToSamplingAreaIndexMap() {
    int num_objects = fixed_target_positions.size();
    default_object_index_to_sampling_area_index_map.resize(num_objects);
    std::iota(default_object_index_to_sampling_area_index_map.begin(),
              default_object_index_to_sampling_area_index_map.end(), 0);
  }
};
