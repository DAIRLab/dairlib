#pragma once

#include "common/file_utils.h"
#include "drake/common/yaml/yaml_read_archive.h"


/* Goal mode options:
  0. kRandom:               randomly generate a new goal.
  1. kOrientationSequence:  keep position goal the same, and cycle through a
                            sequence of orientations.
  2. kFixedGoal:            keep the same goal.
*/
enum GoalMode {
  kRandom,
  kOrientationSequence,
  kFixedGoal
};

struct SamplingC3GoalParams {
  GoalMode goal_mode;

  /// Parameters used for multiple goal modes.
  /// Success thresholds for position and orientation.
  double position_success_threshold;
  double orientation_success_threshold;

  double resting_object_height;             // in world frame
  std::vector<double> resting_object_heights;
  double ee_target_z_offset_above_object;   // defines EE goal wrt object height

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
  Eigen::VectorXd fixed_target_position;
  Eigen::VectorXd fixed_target_orientation;

  std::vector<Eigen::VectorXd> fixed_target_positions;
  std::vector<Eigen::VectorXd> fixed_target_orientations;

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
    a->Visit(DRAKE_NVP(resting_object_height));
    a->Visit(DRAKE_NVP(resting_object_heights));
    a->Visit(DRAKE_NVP(ee_target_z_offset_above_object));
    a->Visit(DRAKE_NVP(lookahead_step_size));
    a->Visit(DRAKE_NVP(lookahead_angle));
    a->Visit(DRAKE_NVP(angle_hysteresis));
    a->Visit(DRAKE_NVP(angle_err_to_vel_factor));
    a->Visit(DRAKE_NVP(fixed_target_position));
    a->Visit(DRAKE_NVP(fixed_target_orientation));
    a->Visit(DRAKE_NVP(fixed_target_positions));
    a->Visit(DRAKE_NVP(fixed_target_orientations));
    a->Visit(DRAKE_NVP(random_goal_x_limits));
    a->Visit(DRAKE_NVP(random_goal_y_limits));
    a->Visit(DRAKE_NVP(random_goal_radius_limits));
    a->Visit(DRAKE_NVP(random_goal_gen_max_attempts));
    a->Visit(DRAKE_NVP(pairwise_goal_distance));
  }
};
