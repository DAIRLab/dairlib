#include "goal_generator.h"

#include <math.h>

#include <cmath>
#include <numeric>

#include <drake/common/yaml/yaml_io.h>

#include "examples/sampling_c3/parameter_headers/goal_params.h"
#include "lcm/lcm_trajectory.h"

using Eigen::VectorXd;

namespace dairlib {
namespace systems {

SamplingC3GoalGenerator::SamplingC3GoalGenerator(
    const drake::multibody::MultibodyPlant<double>& object_plant,
    const SamplingC3GoalParams& goal_params,
    std::vector<std::vector<Eigen::Quaterniond>> nominal_orientations,
    std::vector<drake::multibody::ModelInstanceIndex> object_indices)
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

  // Start with the fixed goal from the goal params.
  target_final_object_positions_ = goal_params_.fixed_target_positions;
  target_final_object_orientations_ = goal_params_.fixed_target_orientations;

  // Initialize the object index to sampling area index map
  object_index_to_sampling_area_index_map_ =
      goal_params_.default_object_index_to_sampling_area_index_map;

  reached_goal_ = std::vector<bool>(object_indices.size(), false);
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
  Eigen::Vector3d obj_curr_position = object_state->GetPositions().tail(3);
  VectorXd obj_curr_quat = object_state->GetPositions().head(4);
  Eigen::Quaterniond curr_quat(obj_curr_quat(0), obj_curr_quat(1),
                               obj_curr_quat(2), obj_curr_quat(3));

  // First, ignore lookahead and use the final goal.
  VectorXd target_obj_position = target_final_object_positions_.at(index);
  Eigen::Quaterniond target_obj_orientation(
      target_final_object_orientations_.at(index)[0],
      target_final_object_orientations_.at(index)[1],
      target_final_object_orientations_.at(index)[2],
      target_final_object_orientations_.at(index)[3]);

  // Check if success has been met. Update goal if necessary.
  double object_position_error;
  if (goal_params_.only_use_xy_position) {
    object_position_error =
        (obj_curr_position.segment(0, 2) -
         target_final_object_positions_.at(index).segment(0, 2))
            .norm();
  } else {
    object_position_error =
        (obj_curr_position - target_final_object_positions_.at(index)).norm();
  }

  Eigen::AngleAxis<double> angle_axis_diff(target_obj_orientation *
                                           curr_quat.inverse());
  double object_angular_error = angle_axis_diff.angle();

  if ((object_position_error < goal_params_.position_success_threshold) &&
      (object_angular_error < goal_params_.orientation_success_threshold)) {
    // std::cout << "\nObject " << index << " Met pose goal!\n" << std::endl;
    reached_goal_[index] = true;
  } else {
    reached_goal_[index] = false;
  }

  bool all_reached = true;
  for (int i = 0; i < reached_goal_.size(); i++) {
    all_reached = all_reached && reached_goal_[i];
  }
  if (all_reached) {
    AssignObjectIndexToGoalSamplingArea();

    // Collect object indices ordered by their assigned sampling areas (left →
    // right)
    std::vector<int> object_indices_to_process(reached_goal_.size());
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
        Eigen::VectorXd::Constant(3, std::numeric_limits<double>::quiet_NaN());
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
  Eigen::Quaterniond y_quat_des(target_final_object_orientations_.at(index)[0],
                                target_final_object_orientations_.at(index)[1],
                                target_final_object_orientations_.at(index)[2],
                                target_final_object_orientations_.at(index)[3]);
  const VectorX<double>& q = object_state->GetPositions().head(4);
  Eigen::VectorXd normalized_q = q / q.norm();
  Eigen::Quaterniond y_quat(normalized_q(0), normalized_q(1), normalized_q(2),
                            normalized_q(3));

  // Compute the orientation error, apply the lookahead angle, and convert the
  // axis-angle error to an angular velocity command.
  Eigen::AngleAxis<double> angle_axis_diff(y_quat_des * y_quat.inverse());
  auto orientation_trajectory =
      PiecewiseQuaternionSlerp<double>({0, 1}, {y_quat, y_quat_des});
  double lookahead_fraction =
      std::min(goal_params_.lookahead_angle / angle_axis_diff.angle(), 1.0);
  Eigen::MatrixXd y_quat_lookahead =
      orientation_trajectory.value(lookahead_fraction);
  Eigen::Quaterniond y_quat_lookahead_quat(
      y_quat_lookahead(0), y_quat_lookahead(1), y_quat_lookahead(2),
      y_quat_lookahead(3));

  Eigen::AngleAxis<double> angle_axis_diff_to_lookahead(y_quat_lookahead_quat *
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
  VectorXd target_final_obj_state = VectorXd::Zero(7);
  target_final_obj_state << target_final_object_orientations_.at(index),
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
  std::vector<int> previous_map = object_index_to_sampling_area_index_map_;
  int num_areas = goal_params_.sampling_area_y_limits.size();

  // Fill with 0..num_areas-1
  std::vector<int> new_map(num_areas);
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
  Eigen::Quaterniond quat_nominal = valid_orientations.at(random_index);

  // Add random yaw in world frame.  Ensure at least 90 degrees away if no
  // topple is required.
  double min_yaw = 0;
  double max_yaw = 2 * M_PI;
  if (random_index == orientation_index_) {
    min_yaw = M_PI / 2;
    max_yaw = 3 * M_PI / 2;
    quat_nominal =
        Eigen::Quaterniond(target_final_object_orientations_.at(index)[0],
                           target_final_object_orientations_.at(index)[1],
                           target_final_object_orientations_.at(index)[2],
                           target_final_object_orientations_.at(index)[3]);
  }
  double yaw = RandomUniform(min_yaw, max_yaw);
  Eigen::Quaterniond quat_world_yaw(
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  Eigen::Quaterniond quat_final = quat_world_yaw * quat_nominal;
  target_final_object_orientations_.at(index) << quat_final.w(), quat_final.x(),
      quat_final.y(), quat_final.z();
  orientation_index_ = random_index;
}

void SamplingC3GoalGenerator::CycleThroughOrientationSequence(int index) const {
  const auto& nominal_orientations = GetNominalOrientations(index);
  int num_nominal_orientations = nominal_orientations.size();
  Eigen::Quaterniond next_quat =
      nominal_orientations.at(goal_counter_ % num_nominal_orientations);
  target_final_object_orientations_.at(index) << next_quat.w(), next_quat.x(),
      next_quat.y(), next_quat.z();
  orientation_index_ = goal_counter_ % num_nominal_orientations;
}

// Outputs the orientation index for debugging.
void SamplingC3GoalGenerator::OutputGoalGeneratorInfo(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* target) const {
  Eigen::MatrixXd orientation_index_data =
      orientation_index_ * Eigen::MatrixXd::Ones(1, 1);
  Eigen::VectorXd timestamp = context.get_time() * Eigen::VectorXd::Ones(1);

  LcmTrajectory::Trajectory orientation_index_traj;
  orientation_index_traj.traj_name = "orientation_index";
  orientation_index_traj.datatypes = std::vector<std::string>(1, "int");
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
  }
  // Otherwise kFixedGoal, nothing to update.
  goal_counter_++;
}

std::pair<Eigen::Quaterniond, Eigen::Vector3d>
SamplingC3GoalGenerator::GenerateLineTrajectoryWithLookahead(
    const Eigen::Quaterniond& quat_curr_orientation,
    const Eigen::Vector3d& obj_curr_position, int index) const {
  Eigen::Vector3d target_obj_position = Eigen::Vector3d::Zero(3);
  Eigen::Quaterniond target_obj_orientation = Eigen::Quaterniond::Identity();

  // First handle position lookahead.
  Eigen::Vector3d start_point = obj_curr_position;
  Eigen::Vector3d end_point = target_final_object_positions_.at(index);

  Eigen::Vector3d distance_vector = end_point - start_point;
  Eigen::Vector3d unit_vector = distance_vector.normalized();
  double step_size =
      std::min(goal_params_.lookahead_step_size, distance_vector.norm());
  target_obj_position = start_point + step_size * unit_vector;

  // Second handle orientation lookahead.
  Eigen::Quaterniond y_quat_des(target_final_object_orientations_.at(index)[0],
                                target_final_object_orientations_.at(index)[1],
                                target_final_object_orientations_.at(index)[2],
                                target_final_object_orientations_.at(index)[3]);
  Eigen::AngleAxis<double> angle_axis_diff(y_quat_des *
                                           quat_curr_orientation.inverse());
  double angle = angle_axis_diff.angle();
  Eigen::Vector3d axis = angle_axis_diff.axis();

  // Enforce consistency near 180 degrees.
  if ((axis.dot(last_rotation_axis_) < 0) &&
      (M_PI - angle < goal_params_.angle_hysteresis)) {
    angle = 2 * M_PI - angle;
    axis = -axis;
  }
  last_rotation_axis_ = axis;

  // Enforce the lookahead.
  angle = std::min(angle, goal_params_.lookahead_angle);

  // Apply the rotation.
  Eigen::AngleAxis<double> angle_axis_relative(angle, axis);
  Eigen::Quaterniond quat_relative = Eigen::Quaterniond(angle_axis_relative);
  Eigen::Quaterniond y_quat_lookahead_quat =
      quat_relative * quat_curr_orientation;
  target_obj_orientation = y_quat_lookahead_quat;

  return std::make_pair(target_obj_orientation, target_obj_position);
}

}  // namespace systems
}  // namespace dairlib
