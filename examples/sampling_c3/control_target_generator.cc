#include "control_target_generator.h"

#include <math.h>
#include <drake/common/yaml/yaml_io.h>

#include "examples/sampling_c3/parameter_headers/franka_c3_controller_params.h"
#include "lcm/lcm_trajectory.h"

using Eigen::VectorXd;

namespace dairlib {
namespace systems {

// TODO: @bibit this file deserves a closer look, parameter overhaul might
// affect this
TargetGenerator::TargetGenerator(
    const drake::multibody::MultibodyPlant<double>& object_plant) {
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
      BasicVector<double>(3),
      &TargetGenerator::CalcEndEffectorTarget)
    .get_index();
  object_target_port_ = this->DeclareVectorOutputPort(
      "object_target",
      BasicVector<double>(7),
      &TargetGenerator::CalcObjectTarget)
    .get_index();
  object_velocity_target_port_ = this->DeclareVectorOutputPort(
      "object_velocity_target",
      BasicVector<double>(6),
      &TargetGenerator::CalcObjectVelocityTarget)
    .get_index();
  object_final_target_port_ = this->DeclareVectorOutputPort(
      "object_final_target",
      BasicVector<double>(7),
      &TargetGenerator::OutputObjectFinalTarget)
    .get_index();
  target_gen_info_port_ = this->DeclareAbstractOutputPort(
      "target_generator_info",
      dairlib::lcmt_timestamped_saved_traj(),
      &TargetGenerator::OutputTargetGeneratorInfo)
    .get_index();
}

void TargetGenerator::SetRemoteControlParameters(
    const int& goal_mode,
    const Eigen::VectorXd& target_object_position,
    const Eigen::VectorXd& target_object_orientation,
    const double& lookahead_step_size,
    const double& lookahead_angle,
    const double& angle_hysteresis,
    const double& angle_err_to_vel_factor,
    const double& ee_target_z_offset_above_object,
    const double& position_success_threshold,
    const double& orientation_success_threshold,
    const Eigen::VectorXd& random_goal_x_limits,
    const Eigen::VectorXd& random_goal_y_limits,
    const Eigen::VectorXd& random_goal_radius_limits,
    const double& resting_object_height)
{
  goal_mode_ = static_cast<GoalMode>(goal_mode);
  target_final_object_position_ = target_object_position;
  target_final_object_orientation_ = target_object_orientation;
  lookahead_step_size_ = lookahead_step_size;
  lookahead_angle_ = lookahead_angle;
  angle_hysteresis_ = angle_hysteresis;
  angle_err_to_vel_factor_ = angle_err_to_vel_factor;
  ee_target_z_offset_above_object_ = ee_target_z_offset_above_object;
  position_success_threshold_ = position_success_threshold;
  orientation_success_threshold_ = orientation_success_threshold;
  random_goal_x_limits_ = random_goal_x_limits;
  random_goal_y_limits_ = random_goal_y_limits;
  random_goal_radius_limits_ = random_goal_radius_limits;
  resting_object_height_ = resting_object_height;
}

void TargetGenerator::CalcEndEffectorTarget(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* target) const {
  // Evaluate input port for object state
  const StateVector<double>* object_state =
    (StateVector<double>*)this->EvalVectorInput(context, object_state_port_);

  VectorXd end_effector_position = object_state->GetPositions().tail(3);
  // Fixing ee target to be at a fixed offset above the object.
  end_effector_position[2] += ee_target_z_offset_above_object_;

  target->SetFromVector(end_effector_position);
}

void TargetGenerator::CalcObjectTarget(
    const drake::systems::Context<double>& context,
    BasicVector<double>* target) const {
  const StateVector<double>* object_state =
    (StateVector<double>*)this->EvalVectorInput(context, object_state_port_);

  // Get current object states.
  Eigen::Vector3d obj_curr_position = object_state->GetPositions().tail(3);
  VectorXd obj_curr_quat = object_state->GetPositions().head(4);
  // Define the current orientation as a quaternion.
  Eigen::Quaterniond curr_quat(obj_curr_quat(0), obj_curr_quat(1),
                               obj_curr_quat(2), obj_curr_quat(3));

  // Desired pose initialized with the target final object position and
  // orientation.
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

  if ((object_position_error < position_success_threshold_) &&
      (object_angular_error < orientation_success_threshold_)) {
    std::cout << "\nMet pose goal!\n" << std::endl;
    OnGoalReached();
  }

  // Adaptive line trajectory with lookahead.
  std::tie(target_obj_orientation, target_obj_position) =
    GenerateLineTrajectoryWithLookahead(curr_quat, obj_curr_position);

  VectorXd target_obj_state = VectorXd::Zero(7);
  target_obj_state << target_obj_orientation.w(), target_obj_orientation.x(),
    target_obj_orientation.y(), target_obj_orientation.z(), target_obj_position;
  target->SetFromVector(target_obj_state);
}

// Command zero linear velocity, and command angular velocity that scales with
// orientation error, scaled by angle_err_to_vel_factor_.
void TargetGenerator::CalcObjectVelocityTarget(
    const drake::systems::Context<double>& context,
    BasicVector<double>* target) const {
  const StateVector<double>* object_state =
    (StateVector<double>*)this->EvalVectorInput(context, object_state_port_);
  // Get the final target orientation
  Eigen::Quaterniond y_quat_des(
    target_final_object_orientation_[0], target_final_object_orientation_[1],
    target_final_object_orientation_[2], target_final_object_orientation_[3]);

  // Get current orientation
  const VectorX<double>& q = object_state->GetPositions().head(4);
  Eigen::VectorXd normalized_q = q / q.norm();
  Eigen::Quaterniond y_quat(normalized_q(0), normalized_q(1), normalized_q(2),
                            normalized_q(3));

  // Compute the error.
  Eigen::AngleAxis<double> angle_axis_diff(y_quat_des * y_quat.inverse());

  // Generate spherically interpolated trajectory.
  auto orientation_trajectory =
    PiecewiseQuaternionSlerp<double>({0, 1}, {y_quat, y_quat_des});

  // Evaluate the trajectory at the lookahead time.
  // Scale time based on lookahead angle.
  double lookahead_fraction =
    std::min(lookahead_angle_ / angle_axis_diff.angle(), 1.0);
  Eigen::MatrixXd y_quat_lookahead =
    orientation_trajectory.value(lookahead_fraction);
  Eigen::Quaterniond y_quat_lookahead_quat(
    y_quat_lookahead(0), y_quat_lookahead(1), y_quat_lookahead(2),
    y_quat_lookahead(3));

  Eigen::AngleAxis<double> angle_axis_diff_to_lookahead(y_quat_lookahead_quat *
                                                        y_quat.inverse());
  VectorXd angle_error = angle_axis_diff_to_lookahead.angle() *
                         angle_axis_diff_to_lookahead.axis();
  angle_error *= angle_err_to_vel_factor_;

  VectorXd target_obj_velocity = VectorXd::Zero(6);
  target_obj_velocity << angle_error, VectorXd::Zero(3);
  target->SetFromVector(target_obj_velocity);
}

void TargetGenerator::OutputObjectFinalTarget(
    const drake::systems::Context<double>& context,
    BasicVector<double>* target) const {
  VectorXd target_final_obj_state = VectorXd::Zero(7);
  target_final_obj_state << target_final_object_orientation_,
    target_final_object_position_;
  target->SetFromVector(target_final_obj_state);
}

void TargetGenerator::SetRandomizedTargetFinalObjectPosition() const {
  // Obeys x and y limits (to stay on the table) as well as radius limits (to
  // more closely match robot limits).
  std::uniform_real_distribution<double> x_dis(random_goal_x_limits_[0],
                                               random_goal_x_limits_[1]);
  std::uniform_real_distribution<double> y_dis(random_goal_y_limits_[0],
                                               random_goal_y_limits_[1]);

  double x = x_dis(rng_);
  double y = y_dis(rng_);
  while ((sqrt(x * x + y * y) > random_goal_radius_limits_[1]) ||
         (sqrt(x * x + y * y) < random_goal_radius_limits_[0])) {
    x = x_dis(rng_);
    y = y_dis(rng_);
  }

  Eigen::VectorXd target_final_object_position(3);
  target_final_object_position_ << x, y, resting_object_height_;
}

void TargetGenerator::SetRandomizedTargetFinalObjectOrientation() const {
  const auto& valid_orientations = GetNominalOrientations();

  std::uniform_int_distribution<int> dis(0, valid_orientations.size() - 1);
  int random_index = dis(rng_);

  Eigen::Quaterniond quat_nominal = valid_orientations.at(random_index);

  // Add random yaw in world frame.  Ensure at least 90 degrees away if no
  // topple is required.
  double min_yaw = 0;
  double max_yaw = 2 * M_PI;
  if (random_index == orientation_index_) {
    min_yaw = M_PI / 2;
    max_yaw = 3 * M_PI / 2;
    quat_nominal = Eigen::Quaterniond(target_final_object_orientation_[0],
                                      target_final_object_orientation_[1],
                                      target_final_object_orientation_[2],
                                      target_final_object_orientation_[3]);
  }
  std::uniform_real_distribution<double> yaw_dis(min_yaw, max_yaw);
  double yaw = yaw_dis(rng_);
  Eigen::Quaterniond quat_world_yaw(
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  Eigen::Quaterniond quat_final = quat_world_yaw * quat_nominal;
  target_final_object_orientation_ << quat_final.w(), quat_final.x(),
                                      quat_final.y(), quat_final.z();
  orientation_index_ = random_index;
}

void TargetGenerator::CycleThroughOrientationSequence() const {
  const auto& nominal_orientations = GetNominalOrientations();
  int num_nominal_orientations = nominal_orientations.size();
  Eigen::Quaterniond next_quat =
    nominal_orientations.at(goal_counter_ % num_nominal_orientations);
  target_final_object_orientation_ << next_quat.w(), next_quat.x(),
                                      next_quat.y(), next_quat.z();
  orientation_index_ = goal_counter_ % num_nominal_orientations;
}

void TargetGenerator::OutputTargetGeneratorInfo(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* target) const {
  // Output the orientation index as an lcm message for debugging purposes.
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

  // Output the mode as an lcm message
  target->saved_traj = orientation_index_lcm_traj.GenerateLcmObject();
  target->utime = context.get_time() * 1e6;
}

void TargetGenerator::OnGoalReached() const {
  // Reset the target object orientation and position.
  if (goal_mode_ == GoalMode::kRandom) {
    SetRandomizedTargetFinalObjectPosition();
    SetRandomizedTargetFinalObjectOrientation();
  } else if (goal_mode_ == GoalMode::kOrientationSequence) {
    // Set the next orientation in the sequence.
    CycleThroughOrientationSequence();
  } else {
    std::cout << "You have only specified a single goal." << std::endl;
  }
  goal_counter_++;
}

std::pair<Eigen::Quaterniond, Eigen::Vector3d>
TargetGenerator::GenerateLineTrajectoryWithLookahead(
    const Eigen::Quaterniond& quat_curr_orientation,
    const Eigen::Vector3d& obj_curr_position) const {
  Eigen::Vector3d target_obj_position = Eigen::Vector3d::Zero(3);
  Eigen::Quaterniond target_obj_orientation = Eigen::Quaterniond::Identity();
  // First handle position lookahead.
  Eigen::Vector3d start_point = obj_curr_position;
  Eigen::Vector3d end_point = target_final_object_position_;

  // compute vector from start point to end point
  Eigen::Vector3d distance_vector = end_point - start_point;

  if (distance_vector.norm() < lookahead_step_size_) {
    target_obj_position(0) = end_point[0];
    target_obj_position(1) = end_point[1];
    target_obj_position(2) = end_point[2];
  } else {
    Eigen::Vector3d step_vector =
      lookahead_step_size_ * (distance_vector / distance_vector.norm());
    Eigen::Vector3d target_on_line_with_lookahead = start_point + step_vector;
    target_obj_position(0) = target_on_line_with_lookahead[0];
    target_obj_position(1) = target_on_line_with_lookahead[1];
    target_obj_position(2) = target_on_line_with_lookahead[2];
  }

  // Second handle orientation lookahead.
  // Get target orientation
  Eigen::Quaterniond y_quat_des(
    target_final_object_orientation_[0], target_final_object_orientation_[1],
    target_final_object_orientation_[2], target_final_object_orientation_[3]);

  // Compute the error.
  Eigen::AngleAxis<double> angle_axis_diff(y_quat_des *
                                           quat_curr_orientation.inverse());

  // Get the axis and angle of rotation.
  double angle = angle_axis_diff.angle();
  Eigen::Vector3d axis = angle_axis_diff.axis();

  // Enforce consistency near 180 degrees.
  if ((axis.dot(last_rotation_axis_) < 0) &&
      (M_PI - angle < angle_hysteresis_)) {
    angle = 2 * M_PI - angle;
    axis = -axis;
  }
  last_rotation_axis_ = axis;

  // Enforce the lookahead.
  angle = std::min(angle, lookahead_angle_);

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
