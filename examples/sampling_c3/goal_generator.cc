#include "goal_generator.h"

#include <math.h>

#include <cmath>
#include <numeric>

#include <drake/common/yaml/yaml_io.h>

#include "common/quaternion_axis_alignment.h"
#include "examples/sampling_c3/parameter_headers/goal_params.h"
#include "lcm/lcm_trajectory.h"

using Eigen::AngleAxisd;
using Eigen::MatrixXd;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using std::vector;

namespace dairlib {
namespace systems {

SamplingC3GoalGenerator::SamplingC3GoalGenerator(
    const drake::multibody::MultibodyPlant<double>& object_plant,
    const SamplingC3GoalParams& goal_params,
    vector<vector<Quaterniond>> nominal_orientations,
    vector<drake::multibody::ModelInstanceIndex> object_indices)
    : goal_params_(goal_params),
      nominal_orientations_(nominal_orientations),
      object_indices_(object_indices) {
  // INPUT PORTS
  radio_port_ =
      this->DeclareAbstractInputPort("lcmt_radio_out",
                                     drake::Value<dairlib::lcmt_radio_out>{})
          .get_index();

  for (int i = 0; i < object_indices_.size(); i++) {
    std::string port_name = "x_object_" + std::to_string(i);
    object_state_ports_.push_back(
        this->DeclareVectorInputPort(port_name, StateVector<double>(7, 6))
            .get_index());
  }
  // OUTPUT PORTS
  end_effector_target_port_ =
      this->DeclareVectorOutputPort(
              "end_effector_target", BasicVector<double>(3),
              &SamplingC3GoalGenerator::CalcEndEffectorTarget)
          .get_index();

  for (int i = 0; i < object_indices_.size(); i++) {
    std::string port_name = "object_target_" + std::to_string(i);
    object_target_ports_.push_back(
        this->DeclareVectorOutputPort(
                port_name, BasicVector<double>(7),
                [this, i](const drake::systems::Context<double>& context,
                          BasicVector<double>* vector) {
                  this->CalcObjectTarget(context, vector, i);
                })
            .get_index());
  }

  for (int i = 0; i < object_indices_.size(); i++) {
    std::string port_name = "object_velocity_target_" + std::to_string(i);
    object_velocity_target_ports_.push_back(
        this->DeclareVectorOutputPort(
                port_name, BasicVector<double>(6),
                [this, i](const drake::systems::Context<double>& context,
                          BasicVector<double>* vector) {
                  this->CalcObjectVelocityTarget(context, vector, i);
                })
            .get_index());
  }

  for (int i = 0; i < object_indices_.size(); i++) {
    std::string port_name = "object_final_target_" + std::to_string(i);
    object_final_target_ports_.push_back(
        this->DeclareVectorOutputPort(
                port_name, BasicVector<double>(7),
                [this, i](const drake::systems::Context<double>& context,
                          BasicVector<double>* vector) {
                  this->OutputObjectFinalTarget(context, vector, i);
                })
            .get_index());
  }

  target_gen_info_port_ =
      this->DeclareAbstractOutputPort(
              "target_generator_info", dairlib::lcmt_timestamped_saved_traj(),
              &SamplingC3GoalGenerator::OutputGoalGeneratorInfo)
          .get_index();

  // Start with the fixed goal from the goal params, or step 0 of the fixed
  // goal sequence for kFixedGoalSequence.
  if (goal_params_.goal_mode == GoalMode::kFixedGoalSequence) {
    target_final_object_positions_ =
        goal_params_.fixed_target_position_sequence.at(0);
    target_final_object_orientations_ =
        goal_params_.fixed_target_orientation_sequence.at(0);
  } else {
    target_final_object_positions_ = goal_params_.fixed_target_positions;
    target_final_object_orientations_ = goal_params_.fixed_target_orientations;
  }

  // Initialize the object index to sampling area index map
  object_index_to_sampling_area_index_map_ =
      goal_params_.default_object_index_to_sampling_area_index_map;

  reached_goal_ = vector<bool>(object_indices.size(), false);
  last_rotation_axis_lookahead_ =
      vector<Vector3d>(object_indices.size(), Vector3d::Zero());
  last_axis_align_lookahead_ =
      vector<Vector3d>(object_indices.size(), Vector3d::Zero());
  last_axis_align_velocity_ =
      vector<Vector3d>(object_indices.size(), Vector3d::Zero());
  last_axis_align_final_target_ =
      vector<Vector3d>(object_indices.size(), Vector3d::Zero());
}

// Fixes the EE target to be a fixed offset above the object.
void SamplingC3GoalGenerator::CalcEndEffectorTarget(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* target) const {
  const StateVector<double>* object_state =
      (StateVector<double>*)this->EvalVectorInput(context,
                                                  object_state_ports_.at(0));

  VectorXd end_effector_position = object_state->GetPositions().tail(3);
  end_effector_position[2] += goal_params_.ee_target_z_offset_above_object;
  target->SetFromVector(end_effector_position);
}

void SamplingC3GoalGenerator::CalcObjectTarget(
    const drake::systems::Context<double>& context, BasicVector<double>* target,
    int index) const {
  const StateVector<double>* object_state =
      (StateVector<double>*)this->EvalVectorInput(
          context, object_state_ports_.at(index));
  Vector3d obj_curr_position = object_state->GetPositions().tail(3);
  VectorXd obj_curr_quat = object_state->GetPositions().head(4);
  Quaterniond curr_quat(obj_curr_quat(0), obj_curr_quat(1), obj_curr_quat(2),
                        obj_curr_quat(3));

  // First, ignore lookahead and use the final goal.
  VectorXd target_obj_position = target_final_object_positions_.at(index);
  Quaterniond target_obj_orientation(
      target_final_object_orientations_.at(index)[0],
      target_final_object_orientations_.at(index)[1],
      target_final_object_orientations_.at(index)[2],
      target_final_object_orientations_.at(index)[3]);

  // Check if success has been met. Update goal if necessary.
  reached_goal_[index] = goal_params_.IsObjectOnTarget(
      index, obj_curr_position, curr_quat,
      target_final_object_positions_.at(index), target_obj_orientation);

  bool all_reached = true;
  for (int i = 0; i < reached_goal_.size(); i++) {
    all_reached = all_reached && reached_goal_[i];
  }
  if (all_reached) {
    AssignObjectIndexToGoalSamplingArea();

    // Collect object indices ordered by their assigned sampling areas (left →
    // right)
    vector<int> object_indices_to_process(reached_goal_.size());
    for (int i = 0; i < reached_goal_.size(); i++) {
      int area_index = object_index_to_sampling_area_index_map_[i];
      object_indices_to_process[area_index] = i;
    }

    for (auto i : object_indices_to_process) {
      OnGoalReached(i);
      reached_goal_[i] = false;
    }

    // Reset the datum position for the next goal generation
    datum_position_ =
        VectorXd::Constant(3, std::numeric_limits<double>::quiet_NaN());
  }

  // Apply lookahead.
  std::tie(target_obj_orientation, target_obj_position) =
      GenerateLineTrajectoryWithLookahead(curr_quat, obj_curr_position, index);
  VectorXd target_obj_state = VectorXd::Zero(7);
  target_obj_state << target_obj_orientation.w(), target_obj_orientation.x(),
      target_obj_orientation.y(), target_obj_orientation.z(),
      target_obj_position;
  target->SetFromVector(target_obj_state);
}

// Command zero linear velocity, and command angular velocity that scales with
// orientation error, scaled by angle_err_to_vel_factor_.
void SamplingC3GoalGenerator::CalcObjectVelocityTarget(
    const drake::systems::Context<double>& context, BasicVector<double>* target,
    int index) const {
  const StateVector<double>* object_state =
      (StateVector<double>*)this->EvalVectorInput(context,
                                                  object_state_ports_[index]);
  // Get the final and current orientation.
  Quaterniond y_quat_des(target_final_object_orientations_.at(index)[0],
                         target_final_object_orientations_.at(index)[1],
                         target_final_object_orientations_.at(index)[2],
                         target_final_object_orientations_.at(index)[3]);
  const VectorX<double>& q = object_state->GetPositions().head(4);
  VectorXd normalized_q = q / q.norm();
  Quaterniond y_quat(normalized_q(0), normalized_q(1), normalized_q(2),
                     normalized_q(3));

  // If tracking axis alignment only, command the angular velocity straight from
  // the hysteresis-consistent swing (angle, axis) that aligns the tracked axis.
  // Going through PiecewiseQuaternionSlerp here (as the full-orientation path
  // below does) would re-flatten the swing to the <= pi geodesic and discard
  // the antipodal hysteresis choice, giving a sign-flipping velocity command
  // near the singularity.
  if (goal_params_.HasTrackedAxis(index)) {
    double swing_angle;
    Vector3d swing_axis;
    dairlib::ComputeAxisAlignedGoalQuaternion(
        y_quat, y_quat_des, goal_params_.tracked_orientation_axis.at(index),
        goal_params_.angle_hysteresis, &last_axis_align_velocity_.at(index),
        &swing_angle, &swing_axis);
    double lookahead_angle =
        std::min(swing_angle, goal_params_.lookahead_angle);
    VectorXd angle_error =
        lookahead_angle * swing_axis * goal_params_.angle_err_to_vel_factor;
    VectorXd target_obj_velocity = VectorXd::Zero(6);
    target_obj_velocity << angle_error, VectorXd::Zero(3);
    target->SetFromVector(target_obj_velocity);
    return;
  }

  // Compute the orientation error, apply the lookahead angle, and convert the
  // axis-angle error to an angular velocity command.
  AngleAxisd angle_axis_diff(y_quat_des * y_quat.inverse());
  auto orientation_trajectory =
      PiecewiseQuaternionSlerp<double>({0, 1}, {y_quat, y_quat_des});
  double lookahead_fraction =
      std::min(goal_params_.lookahead_angle / angle_axis_diff.angle(), 1.0);
  MatrixXd y_quat_lookahead = orientation_trajectory.value(lookahead_fraction);
  Quaterniond y_quat_lookahead_quat(y_quat_lookahead(0), y_quat_lookahead(1),
                                    y_quat_lookahead(2), y_quat_lookahead(3));

  AngleAxisd angle_axis_diff_to_lookahead(y_quat_lookahead_quat *
                                          y_quat.inverse());
  VectorXd angle_error = angle_axis_diff_to_lookahead.angle() *
                         angle_axis_diff_to_lookahead.axis();
  angle_error *= goal_params_.angle_err_to_vel_factor;

  VectorXd target_obj_velocity = VectorXd::Zero(6);
  target_obj_velocity << angle_error, VectorXd::Zero(3);
  target->SetFromVector(target_obj_velocity);
}

void SamplingC3GoalGenerator::OutputObjectFinalTarget(
    const drake::systems::Context<double>& context, BasicVector<double>* target,
    int index) const {
  Vector4d target_final_orientation =
      target_final_object_orientations_.at(index);

  // If tracking axis alignment only, publish the closest quaternion to the
  // object's current orientation that has the tracked axis aligned, so the
  // final target doesn't fight the object's current twist about that axis.
  if (goal_params_.HasTrackedAxis(index)) {
    const StateVector<double>* object_state =
        (StateVector<double>*)this->EvalVectorInput(
            context, object_state_ports_.at(index));
    VectorXd obj_curr_quat = object_state->GetPositions().head(4);
    Quaterniond curr_quat(obj_curr_quat(0), obj_curr_quat(1), obj_curr_quat(2),
                          obj_curr_quat(3));
    Quaterniond target_quat(
        target_final_orientation[0], target_final_orientation[1],
        target_final_orientation[2], target_final_orientation[3]);
    Quaterniond goal_quat = dairlib::ComputeAxisAlignedGoalQuaternion(
        curr_quat, target_quat, goal_params_.tracked_orientation_axis.at(index),
        goal_params_.angle_hysteresis,
        &last_axis_align_final_target_.at(index));
    target_final_orientation << goal_quat.w(), goal_quat.x(), goal_quat.y(),
        goal_quat.z();
  }

  VectorXd target_final_obj_state = VectorXd::Zero(7);
  target_final_obj_state << target_final_orientation,
      target_final_object_positions_.at(index);
  target->SetFromVector(target_final_obj_state);
}

// Assigns the goal sampling area to each object such that the assignment is
// different from the previous assignment for all objects.
// For example, if there are 3 areas, and the current assignment is [0, 1, 2],
// then the new assignment can be [1, 2, 0] or [2, 0, 1].
void SamplingC3GoalGenerator::AssignObjectIndexToGoalSamplingArea() const {
  if (object_index_to_sampling_area_index_map_.size() == 1) {
    return;
  }
  vector<int> previous_map = object_index_to_sampling_area_index_map_;
  int num_areas = goal_params_.sampling_area_y_limits.size();

  // Fill with 0..num_areas-1
  vector<int> new_map(num_areas);
  std::iota(new_map.begin(), new_map.end(), 0);

  std::mt19937 rng{std::random_device{}()};

  // Keep shuffling until we get a derangement
  do {
    std::shuffle(new_map.begin(), new_map.end(), rng);
  } while ([&] {
    for (int i = 0; i < num_areas; ++i) {
      if (new_map[i] == previous_map[i])
        return true;  // same at position i → not valid
    }
    return false;
  }());

  object_index_to_sampling_area_index_map_ = new_map;
}

void SamplingC3GoalGenerator::SetRandomizedTargetFinalObjectPosition(
    int index) const {
  double x = 0, y = 0;
  double x_lower_limit = goal_params_.random_goal_x_limits[0];
  double x_upper_limit = goal_params_.random_goal_x_limits[1];
  double y_lower_limit =
      goal_params_
          .sampling_area_y_limits
              [object_index_to_sampling_area_index_map_[index]]
          .first;
  double y_upper_limit =
      goal_params_
          .sampling_area_y_limits
              [object_index_to_sampling_area_index_map_[index]]
          .second;
  double angle_limit = std::abs(std::asin((x_upper_limit - x_lower_limit) /
                                          goal_params_.pairwise_goal_distance));

  for (int i = 0; i < goal_params_.random_goal_gen_max_attempts; i++) {
    if (datum_position_.hasNaN()) {
      x = RandomUniform(x_lower_limit, x_upper_limit);

      // For 1–2 objects, the leftmost goal doesn’t need to be placed on the
      // left edge. For 3+ objects, the leftmost goal must be on the left edge
      // to ensure enough space for the remaining goals.
      if (object_index_to_sampling_area_index_map_.size() <= 2) {
        y = RandomUniform(y_lower_limit, y_upper_limit);
      } else {
        y = y_lower_limit;
      }

      std::cout << "Object: " << index << " Sampled position at " << x << ", "
                << y << std::endl;
      break;
    }
    double random_angle = RandomUniform(-angle_limit, angle_limit);
    x = datum_position_[0] +
        goal_params_.pairwise_goal_distance * std::sin(random_angle);
    y = datum_position_[1] +
        goal_params_.pairwise_goal_distance * std::cos(random_angle);

    bool is_sampled_goal_in_designated_area =
        x >= x_lower_limit && x <= x_upper_limit && y >= y_lower_limit &&
        y <= y_upper_limit;

    if (is_sampled_goal_in_designated_area) {
      std::cout << "Object: " << index << " Found suitable goal after " << i
                << " attempts"
                << " at " << x << ", " << y << std::endl;
      break;
    }
  }
  target_final_object_positions_.at(index) << x, y,
      goal_params_.resting_object_heights.at(index);
  datum_position_ << x, y, goal_params_.resting_object_heights.at(index);
}

// Randomly generates final orientation from the set of valid orientations plus
// an imposed random yaw.  If no topples are required, the yaw is at least 90
// degrees away from the current orientation.
void SamplingC3GoalGenerator::SetRandomizedTargetFinalObjectOrientation(
    int index) const {
  const auto& valid_orientations = GetNominalOrientations(index);
  std::uniform_int_distribution<int> dis(0, valid_orientations.size() - 1);
  std::mt19937 rng{std::random_device{}()};
  int random_index = dis(rng);
  Quaterniond quat_nominal = valid_orientations.at(random_index);

  // Add random yaw in world frame.  Ensure at least 90 degrees away if no
  // topple is required.
  double min_yaw = 0;
  double max_yaw = 2 * M_PI;
  if (random_index == orientation_index_) {
    min_yaw = M_PI / 2;
    max_yaw = 3 * M_PI / 2;
    quat_nominal = Quaterniond(target_final_object_orientations_.at(index)[0],
                               target_final_object_orientations_.at(index)[1],
                               target_final_object_orientations_.at(index)[2],
                               target_final_object_orientations_.at(index)[3]);
  }
  double yaw = RandomUniform(min_yaw, max_yaw);
  Quaterniond quat_world_yaw(AngleAxisd(yaw, Vector3d::UnitZ()));
  Quaterniond quat_final = quat_world_yaw * quat_nominal;
  target_final_object_orientations_.at(index) << quat_final.w(), quat_final.x(),
      quat_final.y(), quat_final.z();
  orientation_index_ = random_index;
}

void SamplingC3GoalGenerator::CycleThroughOrientationSequence(int index) const {
  const auto& nominal_orientations = GetNominalOrientations(index);
  int num_nominal_orientations = nominal_orientations.size();
  Quaterniond next_quat =
      nominal_orientations.at(goal_counter_ % num_nominal_orientations);
  target_final_object_orientations_.at(index) << next_quat.w(), next_quat.x(),
      next_quat.y(), next_quat.z();
  orientation_index_ = goal_counter_ % num_nominal_orientations;
}

// Advances to the next (position, orientation) goal in the fixed goal
// sequence, clamping at the last step once reached (no wrap-around).
void SamplingC3GoalGenerator::CycleThroughFixedGoalSequence(int index) const {
  int num_steps = goal_params_.fixed_target_position_sequence.size();
  int step = std::min(goal_counter_, num_steps - 1);
  target_final_object_positions_.at(index) =
      goal_params_.fixed_target_position_sequence.at(step).at(index);
  target_final_object_orientations_.at(index) =
      goal_params_.fixed_target_orientation_sequence.at(step).at(index);
}

// Outputs the orientation index for debugging.
void SamplingC3GoalGenerator::OutputGoalGeneratorInfo(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* target) const {
  MatrixXd orientation_index_data = orientation_index_ * MatrixXd::Ones(1, 1);
  VectorXd timestamp = context.get_time() * VectorXd::Ones(1);

  LcmTrajectory::Trajectory orientation_index_traj;
  orientation_index_traj.traj_name = "orientation_index";
  orientation_index_traj.datatypes = vector<std::string>(1, "int");
  orientation_index_traj.datapoints = orientation_index_data;
  orientation_index_traj.time_vector = timestamp.cast<double>();

  LcmTrajectory orientation_index_lcm_traj(
      {orientation_index_traj}, {"orientation_index"}, "orientation_index",
      "orientation_index", false);

  target->saved_traj = orientation_index_lcm_traj.GenerateLcmObject();
  target->utime = context.get_time() * 1e6;
}

void SamplingC3GoalGenerator::OnGoalReached(int index) const {
  // Reset the target object orientation and position.
  if (goal_params_.goal_mode == GoalMode::kRandom) {
    SetRandomizedTargetFinalObjectPosition(index);
    SetRandomizedTargetFinalObjectOrientation(index);
  } else if (goal_params_.goal_mode == GoalMode::kOrientationSequence) {
    // Set the next orientation in the sequence.
    CycleThroughOrientationSequence(index);
  } else if (goal_params_.goal_mode == GoalMode::kFixedGoalSequence) {
    CycleThroughFixedGoalSequence(index);
  }
  // Otherwise kFixedGoal, nothing to update.
  goal_counter_++;
}

std::pair<Quaterniond, Vector3d>
SamplingC3GoalGenerator::GenerateLineTrajectoryWithLookahead(
    const Quaterniond& quat_curr_orientation, const Vector3d& obj_curr_position,
    int index) const {
  Vector3d target_obj_position = Vector3d::Zero(3);
  Quaterniond target_obj_orientation = Quaterniond::Identity();

  // First handle position lookahead.
  Vector3d start_point = obj_curr_position;
  Vector3d end_point = target_final_object_positions_.at(index);

  Vector3d distance_vector = end_point - start_point;
  Vector3d unit_vector = distance_vector.normalized();
  double step_size =
      std::min(goal_params_.lookahead_step_size, distance_vector.norm());
  target_obj_position = start_point + step_size * unit_vector;

  // Second handle orientation lookahead.
  Quaterniond y_quat_des(target_final_object_orientations_.at(index)[0],
                         target_final_object_orientations_.at(index)[1],
                         target_final_object_orientations_.at(index)[2],
                         target_final_object_orientations_.at(index)[3]);

  double angle;
  Vector3d axis;
  if (goal_params_.HasTrackedAxis(index)) {
    // The antipodal-singularity hysteresis has already been resolved once,
    // inside ComputeAxisAlignedGoalQuaternion against its own dedicated state;
    // do not re-run the full-quaternion hysteresis check below against a
    // different state array on top of it. Take the swing (angle, axis)
    // directly from that function -- re-extracting it from goal_quat via
    // AngleAxisd would canonicalize the angle to [0, pi] and flip the axis
    // sign, discarding the hysteresis choice and making the lookahead target
    // oscillate near the singularity.
    dairlib::ComputeAxisAlignedGoalQuaternion(
        quat_curr_orientation, y_quat_des,
        goal_params_.tracked_orientation_axis.at(index),
        goal_params_.angle_hysteresis, &last_axis_align_lookahead_.at(index),
        &angle, &axis);
  } else {
    AngleAxisd angle_axis_diff(y_quat_des * quat_curr_orientation.inverse());
    angle = angle_axis_diff.angle();
    axis = angle_axis_diff.axis();

    // Enforce consistency near 180 degrees.
    if ((axis.dot(last_rotation_axis_lookahead_.at(index)) < 0) &&
        (M_PI - angle < goal_params_.angle_hysteresis)) {
      angle = 2 * M_PI - angle;
      axis = -axis;
    }
    last_rotation_axis_lookahead_.at(index) = axis;
  }

  // Enforce the lookahead.
  angle = std::min(angle, goal_params_.lookahead_angle);

  // Apply the rotation.
  AngleAxisd angle_axis_relative(angle, axis);
  Quaterniond quat_relative = Quaterniond(angle_axis_relative);
  Quaterniond y_quat_lookahead_quat = quat_relative * quat_curr_orientation;
  target_obj_orientation = y_quat_lookahead_quat;

  return std::make_pair(target_obj_orientation, target_obj_position);
}

}  // namespace systems
}  // namespace dairlib
