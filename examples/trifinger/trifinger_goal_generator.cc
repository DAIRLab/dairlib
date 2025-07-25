#include "trifinger_goal_generator.h"

#include <math.h>
#include <drake/common/yaml/yaml_io.h>

#include "examples/sampling_c3/parameter_headers/goal_params.h"
#include "lcm/lcm_trajectory.h"

using Eigen::VectorXd;

namespace dairlib {
namespace systems {

SamplingC3GoalGeneratorTrifinger::SamplingC3GoalGeneratorTrifinger(
    const int end_effector_num_positions,
    const drake::multibody::MultibodyPlant<double>& object_plant,
    const SamplingC3GoalParams& goal_params,
    const std::vector<Eigen::Quaterniond>& nominal_orientations) :
  end_effector_num_positions_(end_effector_num_positions),
  goal_params_(goal_params),
  nominal_orientations_(nominal_orientations) {
  // INPUT PORTS
  radio_port_ = this->DeclareAbstractInputPort(
      "lcmt_radio_out",
      drake::Value<dairlib::lcmt_radio_out>{})
    .get_index();
  object_state_port_ = this->DeclareVectorInputPort(
      "x_object",
      StateVector<double>(object_plant.num_positions(),
        object_plant.num_velocities()))
    .get_index();

  // OUTPUT PORTS
  end_effector_target_port_ = this->DeclareVectorOutputPort(
      "end_effector_target",
      BasicVector<double>(end_effector_num_positions_),
      &SamplingC3GoalGeneratorTrifinger::CalcEndEffectorTarget)
    .get_index();
  object_target_port_ = this->DeclareVectorOutputPort(
      "object_target",
      BasicVector<double>(object_plant.num_positions()),
      &SamplingC3GoalGeneratorTrifinger::CalcObjectTarget)
    .get_index();
  object_velocity_target_port_ = this->DeclareVectorOutputPort(
      "object_velocity_target",
      BasicVector<double>(object_plant.num_velocities()),
      &SamplingC3GoalGeneratorTrifinger::CalcObjectVelocityTarget)
    .get_index();
  object_final_target_port_ = this->DeclareVectorOutputPort(
      "object_final_target",
      BasicVector<double>(object_plant.num_positions()),
      &SamplingC3GoalGeneratorTrifinger::OutputObjectFinalTarget)
    .get_index();
  target_gen_info_port_ = this->DeclareAbstractOutputPort(
      "target_generator_info",
      dairlib::lcmt_timestamped_saved_traj(),
      &SamplingC3GoalGeneratorTrifinger::OutputGoalGeneratorInfo)
    .get_index();

  // Start with the fixed goal from the goal params.
  target_final_object_position_ = goal_params_.fixed_target_position;
  target_final_object_orientation_ = goal_params_.fixed_target_orientation;
}

// Fixes the EE target to be a fixed offset above the object.
void SamplingC3GoalGeneratorTrifinger::CalcEndEffectorTarget(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* target) const {
  VectorXd end_effector_position(end_effector_num_positions_);
  end_effector_position << 0, 0.05, 0.0325, 0.05, 0, 0.0325, 0, -0.05, 0.0325;
  target->SetFromVector(end_effector_position);
}

void SamplingC3GoalGeneratorTrifinger::CalcObjectTarget(
    const drake::systems::Context<double>& context,
    BasicVector<double>* target) const {
  const StateVector<double>* object_state =
    (StateVector<double>*)this->EvalVectorInput(context, object_state_port_);
  Eigen::Vector3d obj_curr_position = object_state->GetPositions().tail(3);
  VectorXd obj_curr_quat = object_state->GetPositions().head(4);
  Eigen::Quaterniond curr_quat(obj_curr_quat(0), obj_curr_quat(1),
                               obj_curr_quat(2), obj_curr_quat(3));

  // First, ignore lookahead and use the final goal.
  VectorXd target_obj_position = target_final_object_position_;
  Eigen::Quaterniond target_obj_orientation(
    target_final_object_orientation_[0], target_final_object_orientation_[1],
    target_final_object_orientation_[2], target_final_object_orientation_[3]);

  // Check if success has been met. Update goal if necessary.
  double object_position_error =
    (obj_curr_position - target_final_object_position_).norm();
  Eigen::AngleAxis<double> angle_axis_diff(target_obj_orientation *
                                           curr_quat.inverse());
  double object_angular_error = angle_axis_diff.angle();

  if ((object_position_error < goal_params_.position_success_threshold) &&
      (object_angular_error < goal_params_.orientation_success_threshold)) {
    std::cout << "\nMet pose goal!\n" << std::endl;
    OnGoalReached();
  }

  // Apply lookahead.
  std::tie(target_obj_orientation, target_obj_position) =
    GenerateLineTrajectoryWithLookahead(curr_quat, obj_curr_position);
  VectorXd target_obj_state = VectorXd::Zero(7);
  target_obj_state << target_obj_orientation.w(), target_obj_orientation.x(),
    target_obj_orientation.y(), target_obj_orientation.z(), target_obj_position;
  target->SetFromVector(target_obj_state);
}

// Command zero linear velocity, and command angular velocity that scales with
// orientation error, scaled by angle_err_to_vel_factor_.
void SamplingC3GoalGeneratorTrifinger::CalcObjectVelocityTarget(
    const drake::systems::Context<double>& context,
    BasicVector<double>* target) const {
  const StateVector<double>* object_state =
    (StateVector<double>*)this->EvalVectorInput(context, object_state_port_);
  // Get the final and current orientation.
  Eigen::Quaterniond y_quat_des(
    target_final_object_orientation_[0], target_final_object_orientation_[1],
    target_final_object_orientation_[2], target_final_object_orientation_[3]);
  const VectorX<double>& q = object_state->GetPositions().head(4);
  Eigen::VectorXd normalized_q = q / q.norm();
  Eigen::Quaterniond y_quat(normalized_q(0), normalized_q(1),
                            normalized_q(2), normalized_q(3));

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
    y_quat_lookahead(0), y_quat_lookahead(1),
    y_quat_lookahead(2), y_quat_lookahead(3));

  Eigen::AngleAxis<double> angle_axis_diff_to_lookahead(y_quat_lookahead_quat *
                                                        y_quat.inverse());
  VectorXd angle_error = angle_axis_diff_to_lookahead.angle() *
                         angle_axis_diff_to_lookahead.axis();
  angle_error *= goal_params_.angle_err_to_vel_factor;

  VectorXd target_obj_velocity = VectorXd::Zero(6);
  target_obj_velocity << angle_error, VectorXd::Zero(3);
  target->SetFromVector(target_obj_velocity);
}

void SamplingC3GoalGeneratorTrifinger::OutputObjectFinalTarget(
    const drake::systems::Context<double>& context,
    BasicVector<double>* target) const {
  VectorXd target_final_obj_state = VectorXd::Zero(7);
  target_final_obj_state << target_final_object_orientation_,
    target_final_object_position_;
  target->SetFromVector(target_final_obj_state);
}

// Randomly generates final position within the specified goal limits in x/y/r.
void SamplingC3GoalGeneratorTrifinger::SetRandomizedTargetFinalObjectPosition() const {
  double x, y = 0;
  while ((sqrt(x * x + y * y) > goal_params_.random_goal_radius_limits[1]) ||
  (sqrt(x * x + y * y) < goal_params_.random_goal_radius_limits[0])) {
    x = RandomUniform(goal_params_.random_goal_x_limits[0],
                      goal_params_.random_goal_x_limits[1]);
    y = RandomUniform(goal_params_.random_goal_y_limits[0],
                      goal_params_.random_goal_y_limits[1]);
  }

  target_final_object_position_ << x, y, goal_params_.resting_object_height;
}

// Randomly generates final orientation from the set of valid orientations plus
// an imposed random yaw.  If no topples are required, the yaw is at least 90
// degrees away from the current orientation.
void SamplingC3GoalGeneratorTrifinger::SetRandomizedTargetFinalObjectOrientation() const {
  const auto& valid_orientations = GetNominalOrientations();
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
    quat_nominal = Eigen::Quaterniond(
      target_final_object_orientation_[0], target_final_object_orientation_[1],
      target_final_object_orientation_[2], target_final_object_orientation_[3]);
  }
  double yaw = RandomUniform(min_yaw, max_yaw);
  Eigen::Quaterniond quat_world_yaw(
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  Eigen::Quaterniond quat_final = quat_world_yaw * quat_nominal;
  target_final_object_orientation_ << quat_final.w(), quat_final.x(),
                                      quat_final.y(), quat_final.z();
  orientation_index_ = random_index;
}

void SamplingC3GoalGeneratorTrifinger::CycleThroughOrientationSequence() const {
  const auto& nominal_orientations = GetNominalOrientations();
  int num_nominal_orientations = nominal_orientations.size();
  Eigen::Quaterniond next_quat =
    nominal_orientations.at(goal_counter_ % num_nominal_orientations);
  target_final_object_orientation_ << next_quat.w(), next_quat.x(),
                                      next_quat.y(), next_quat.z();
  orientation_index_ = goal_counter_ % num_nominal_orientations;
}

// Outputs the orientation index for debugging.
void SamplingC3GoalGeneratorTrifinger::OutputGoalGeneratorInfo(
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

void SamplingC3GoalGeneratorTrifinger::OnGoalReached() const {
  // Reset the target object orientation and position.
  if (goal_params_.goal_mode == GoalMode::kRandom) {
    SetRandomizedTargetFinalObjectPosition();
    SetRandomizedTargetFinalObjectOrientation();
  } else if (goal_params_.goal_mode == GoalMode::kOrientationSequence) {
    // Set the next orientation in the sequence.
    CycleThroughOrientationSequence();
  } else {
    std::cout << "You have only specified a single goal." << std::endl;
  }
  goal_counter_++;
}

std::pair<Eigen::Quaterniond, Eigen::Vector3d>
SamplingC3GoalGeneratorTrifinger::GenerateLineTrajectoryWithLookahead(
    const Eigen::Quaterniond& quat_curr_orientation,
    const Eigen::Vector3d& obj_curr_position) const {
  Eigen::Vector3d target_obj_position = Eigen::Vector3d::Zero(3);
  Eigen::Quaterniond target_obj_orientation = Eigen::Quaterniond::Identity();

  // First handle position lookahead.
  Eigen::Vector3d start_point = obj_curr_position;
  Eigen::Vector3d end_point = target_final_object_position_;

  Eigen::Vector3d distance_vector = end_point - start_point;
  Eigen::Vector3d unit_vector = distance_vector.normalized();
  double step_size = std::min(goal_params_.lookahead_step_size,
                              distance_vector.norm());
  target_obj_position = start_point + step_size * unit_vector;

  // Second handle orientation lookahead.
  Eigen::Quaterniond y_quat_des(
    target_final_object_orientation_[0], target_final_object_orientation_[1],
    target_final_object_orientation_[2], target_final_object_orientation_[3]);
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
