#pragma once

#include <numeric>

#include "common/file_utils.h"
#include "common/quaternion_axis_alignment.h"

#include "drake/common/drake_assert.h"
#include "drake/common/yaml/yaml_read_archive.h"

/* Goal mode options:
  0. kRandom:               randomly generate a new goal.
  1. kOrientationSequence:  keep position goal the same, and cycle through a
                            sequence of orientations.
  2. kFixedGoal:            keep the same goal.
  3. kFixedGoalSequence:    step through a sequence of fixed (position,
                            orientation) goals, advancing to the next goal
                            once the current one is reached, and holding at
                            the last goal once reached.
*/
enum GoalMode { kRandom, kOrientationSequence, kFixedGoal, kFixedGoalSequence };

struct SamplingC3GoalParams {
  GoalMode goal_mode;

  /// Parameters used for multiple goal modes.
  /// Success thresholds for position and orientation.
  double position_success_threshold;
  double orientation_success_threshold;
  bool only_use_xy_position;

  std::vector<double> resting_object_heights;  // in world frame for each object
  double ee_target_z_offset_above_object;  // defines EE goal wrt object height

  /// End-effector position the robot moves to (and holds) to get out of the way
  /// once every fixed goal has been reached.  Used for kFixedGoal and for the
  /// terminal goal of a kFixedGoalSequence.  Must lie within the workspace
  /// limits so the robot can actually reach it.
  Eigen::Vector3d ee_parked_position;

  // Per-object body-frame axis to align with the target's world-frame body axis
  // direction, tracking twist about that axis freely. A zero vector (the
  // default) means "track full orientation" for that object.
  std::vector<Eigen::Vector3d> tracked_orientation_axis;

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

  /// Sequence of (position, orientation) goals, used only when goal_mode ==
  /// kFixedGoalSequence. Outer index = step in the sequence; inner index =
  /// object.  Once the last step is reached, the goal holds there (no wrap-
  /// around).
  std::vector<std::vector<Eigen::Vector3d>> fixed_target_position_sequence;
  std::vector<std::vector<Eigen::Vector4d>> fixed_target_orientation_sequence;

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
    a->Visit(DRAKE_NVP(ee_parked_position));
    a->Visit(DRAKE_NVP(tracked_orientation_axis));
    a->Visit(DRAKE_NVP(lookahead_step_size));
    a->Visit(DRAKE_NVP(lookahead_angle));
    a->Visit(DRAKE_NVP(angle_hysteresis));
    a->Visit(DRAKE_NVP(angle_err_to_vel_factor));
    a->Visit(DRAKE_NVP(fixed_target_positions));
    a->Visit(DRAKE_NVP(fixed_target_orientations));
    a->Visit(DRAKE_NVP(fixed_target_position_sequence));
    a->Visit(DRAKE_NVP(fixed_target_orientation_sequence));
    a->Visit(DRAKE_NVP(random_goal_x_limits));
    a->Visit(DRAKE_NVP(random_goal_y_limits));
    a->Visit(DRAKE_NVP(random_goal_radius_limits));
    a->Visit(DRAKE_NVP(random_goal_gen_max_attempts));
    a->Visit(DRAKE_NVP(pairwise_goal_distance));
    a->Visit(DRAKE_NVP(only_use_xy_position));
    ComputeSamplingAreaYLimits();
    SetDefaultObjectIndexToSamplingAreaIndexMap();
    NormalizeAndValidateTrackedAxes();
    ValidateFixedGoalSequence();
  }

  bool HasTrackedAxis(int index) const {
    return tracked_orientation_axis.at(index).squaredNorm() > 1e-12;
  }

  /// The single definition of "object `index` has reached its goal", shared by
  /// the goal generator (which uses it to advance the goal) and the controller
  /// (which uses it to decide which objects still need samples and when to park
  /// at a terminal fixed goal).
  ///
  /// Position is compared in xy only when only_use_xy_position is set.
  /// Orientation is compared as the tracked axis's misalignment when this
  /// object tracks an axis, and as the full quaternion error otherwise.
  bool IsObjectOnTarget(int index, const Eigen::Vector3d& p_curr,
                        const Eigen::Quaterniond& q_curr,
                        const Eigen::Vector3d& p_des,
                        const Eigen::Quaterniond& q_des) const {
    double position_error = only_use_xy_position
                                ? (p_curr - p_des).head(2).norm()
                                : (p_curr - p_des).norm();
    double orientation_error =
        HasTrackedAxis(index)
            ? dairlib::ComputeAxisMisalignmentAngle(
                  q_curr, q_des, tracked_orientation_axis.at(index))
            : Eigen::AngleAxisd(q_des * q_curr.inverse()).angle();
    return (position_error < position_success_threshold) &&
           (orientation_error < orientation_success_threshold);
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

  void NormalizeAndValidateTrackedAxes() {
    DRAKE_DEMAND(tracked_orientation_axis.size() ==
                 fixed_target_positions.size());
    for (auto& axis : tracked_orientation_axis) {
      if (axis.squaredNorm() > 1e-12) {
        axis.normalize();
      }
    }
  }

  void ValidateFixedGoalSequence() const {
    if (goal_mode != GoalMode::kFixedGoalSequence) {
      return;
    }
    DRAKE_DEMAND(!fixed_target_position_sequence.empty());
    DRAKE_DEMAND(fixed_target_position_sequence.size() ==
                 fixed_target_orientation_sequence.size());
    int num_objects = fixed_target_positions.size();
    for (const auto& step_positions : fixed_target_position_sequence) {
      DRAKE_DEMAND(step_positions.size() == num_objects);
    }
    for (const auto& step_orientations : fixed_target_orientation_sequence) {
      DRAKE_DEMAND(step_orientations.size() == num_objects);
    }
  }
};
