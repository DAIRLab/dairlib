#include "assembly_controller.h"

#include <cmath>

#include <Eigen/Dense>
#include <drake/math/roll_pitch_yaw.h>

#include "common/update_context.h"
#include "examples/magna/parameter_headers/assembly_c3_options.h"
#include "examples/magna/parameter_headers/target_poses.h"
#include "systems/framework/timestamped_vector.h"

#include "drake/lcmt_schunk_wsg_command.hpp"

#define POSITION_TOLERANCE 0.005
#define POSITION_TOLERANCE_CIRCULAR_ARC 0.01
#define ORIENTATION_TOLERANCE 0.1  // radians (approximately 5.7 degrees)

namespace dairlib {
namespace examples {
namespace magna {

using drake::SortedPair;
using drake::geometry::GeometryId;
using drake::math::RollPitchYaw;
using drake::multibody::MultibodyPlant;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteValues;
using Eigen::MatrixXd;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using solvers::C3Plus;
using solvers::LCS;
using solvers::LCSFactory;
using std::vector;
using systems::TimestampedVector;

AssemblyController::AssemblyController(
    drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context,
    drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad,
    drake::systems::Context<drake::AutoDiffXd>* context_ad,
    const std::vector<
        std::vector<drake::SortedPair<drake::geometry::GeometryId>>>&
        contact_geoms,
    const AssemblyC3Options& assembly_c3_options,
    const TargetPosesParams& target_poses_params, bool verbose)
    : plant_(plant),
      context_(context),
      plant_ad_(plant_ad),
      context_ad_(context_ad),
      contact_pairs_(contact_geoms),
      assembly_c3_options_(assembly_c3_options),
      N_(assembly_c3_options.N),
      dt_(assembly_c3_options.dt),
      verbose_(verbose) {
  this->set_name("assembly_controller");

  n_q_ = plant_.num_positions();
  n_v_ = plant_.num_velocities();
  n_u_ = plant_.num_actuators();
  n_x_ = n_q_ + n_v_;

  auto c3_options = assembly_c3_options_.GetC3Options();

  // Determine contact model and n_lambda_
  if (assembly_c3_options_.contact_model == "stewart_and_trinkle") {
    contact_model_ = solvers::ContactModel::kStewartAndTrinkle;
    n_lambda_ = 2 * assembly_c3_options_.num_contacts +
                2 * assembly_c3_options_.num_friction_directions *
                    assembly_c3_options_.num_contacts;
  } else if (assembly_c3_options_.contact_model == "anitescu") {
    contact_model_ = solvers::ContactModel::kAnitescu;
    n_lambda_ = 2 * assembly_c3_options_.num_friction_directions *
                assembly_c3_options_.num_contacts;
  } else {
    std::cerr << "Unknown or unsupported contact model: "
              << assembly_c3_options_.contact_model << std::endl;
    DRAKE_THROW_UNLESS(false);
  }
  if (assembly_c3_options_.contact_model == "stewart_and_trinkle") {
    contact_model_ = solvers::ContactModel::kStewartAndTrinkle;
    n_lambda_ = 2 * assembly_c3_options_.num_contacts +
                2 * assembly_c3_options_.num_friction_directions *
                    assembly_c3_options_.num_contacts;
  } else {
    contact_model_ = solvers::ContactModel::kAnitescu;
    n_lambda_ = 2 * assembly_c3_options_.num_friction_directions *
                assembly_c3_options_.num_contacts;
  }
  // Initialize cost matrices
  double discount_factor = 1;
  for (int i = 0; i < N_ + 1; ++i) {
    Q_.push_back(discount_factor * c3_options.Q);
    discount_factor *= assembly_c3_options_.gamma;
    if (i < N_) {
      R_.push_back(discount_factor * c3_options.R);
      G_.push_back(c3_options.G);
      U_.push_back(c3_options.U);
    }
  }

  // Create placeholder LCS for initializing C3Plus
  MatrixXd A = MatrixXd::Ones(n_x_, n_x_);
  MatrixXd B = MatrixXd::Zero(n_x_, n_u_);
  VectorXd d = VectorXd::Zero(n_x_);
  MatrixXd D = MatrixXd::Ones(n_x_, n_lambda_);
  MatrixXd E = MatrixXd::Zero(n_lambda_, n_x_);
  MatrixXd F = MatrixXd::Zero(n_lambda_, n_lambda_);
  MatrixXd H = MatrixXd::Zero(n_lambda_, n_u_);
  VectorXd c = VectorXd::Zero(n_lambda_);
  LCS lcs_placeholder(A, B, D, d, E, F, H, c, N_, dt_);

  auto x_desired_placeholder =
      std::vector<VectorXd>(N_ + 1, VectorXd::Zero(n_x_));
  c3_mpc_ = std::make_shared<C3Plus>(
      lcs_placeholder, solvers::C3Base::CostMatrices(Q_, R_, G_, U_),
      x_desired_placeholder, c3_options);

  // Input ports
  lcs_state_input_port_ =
      this->DeclareVectorInputPort("x_lcs", TimestampedVector<double>(n_x_))
          .get_index();
  target_input_port_ =
      this->DeclareVectorInputPort("x_lcs_des", n_x_).get_index();

  // Output ports
  traj_execute_port_ =
      this->DeclareAbstractOutputPort("traj_execute",
                                      dairlib::lcmt_timestamped_saved_traj(),
                                      &AssemblyController::OutputTrajExecute)
          .get_index();

  traj_planned_keypoints_port_ =
      this->DeclareAbstractOutputPort(
              "traj_planned_keypoints", dairlib::lcmt_timestamped_saved_traj(),
              &AssemblyController::OutputTrajPlannedKeypoints)
          .get_index();

  gripper_pos_command_port_ =
      this->DeclareAbstractOutputPort(
              "gripper_pos_command", drake::lcmt_schunk_wsg_command(),
              &AssemblyController::OutputGripperPosCommand)
          .get_index();

  // Discrete state for phase tracking
  phase_index_ = this->DeclareDiscreteState(1);
  plan_start_time_index_ = this->DeclareDiscreteState(1);
  current_target_index_ = this->DeclareDiscreteState(1);
  // Initialize target_reached_time to -1 (not reached yet) - negative means not
  // reached
  target_reached_time_index_ =
      this->DeclareDiscreteState(Eigen::VectorXd::Constant(1, -1.0));

  this->DeclareForcedDiscreteUpdateEvent(&AssemblyController::ComputePlan);

  // Load target poses from YAML parameters
  target_poses_ = target_poses_params.ToTargetPoses();
}

drake::systems::EventStatus AssemblyController::ComputePlan(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  // Evaluate input ports
  const TimestampedVector<double>* lcs_x_curr =
      (TimestampedVector<double>*)this->EvalVectorInput(context,
                                                        lcs_state_input_port_);
  const BasicVector<double>& lcs_x_des =
      *this->template EvalVectorInput<BasicVector>(context, target_input_port_);
  drake::VectorX<double> x_lcs_des = lcs_x_des.get_value();
  drake::VectorX<double> x_lcs_curr = lcs_x_curr->get_data();
  double t_context = lcs_x_curr->get_timestamp();

  discrete_state->get_mutable_value(plan_start_time_index_)[0] = t_context;

  // Get current phase (could be updated based on conditions)
  int phase_int = static_cast<int>(discrete_state->get_value(phase_index_)[0]);
  current_phase_ = static_cast<AssemblyPhase>(phase_int);

  // Get current target index
  int current_target_idx =
      static_cast<int>(discrete_state->get_value(current_target_index_)[0]);

  // Safety check: ensure target index is valid
  if (!target_poses_.empty()) {
    if (current_target_idx < 0 ||
        current_target_idx >= static_cast<int>(target_poses_.size())) {
      // Reset to first target if invalid
      current_target_idx = 0;
      discrete_state->get_mutable_value(current_target_index_)[0] = 0.0;
    }
  }

  // Check if we have targets and if current target is valid
  if (!target_poses_.empty() && current_target_idx >= 0 &&
      current_target_idx < static_cast<int>(target_poses_.size()) &&
      current_phase_ == AssemblyPhase::kMoveToTarget) {
    const TargetPose& current_target_pose = target_poses_[current_target_idx];
    double target_reached_time =
        discrete_state->get_value(target_reached_time_index_)[0];

    // Check if current target is reached
    if (IsTargetReached(x_lcs_curr, current_target_idx)) {
      // If this is the first time we've reached the target, record the time
      if (target_reached_time < 0.0) {
        target_reached_time = t_context;
        discrete_state->get_mutable_value(target_reached_time_index_)[0] =
            target_reached_time;
        std::cout << "Target " << current_target_idx
                  << " reached at t=" << target_reached_time << std::endl;
      }

      // Check if dwell time has elapsed (if dwell is specified)
      double time_at_target = t_context - target_reached_time;
      if (current_target_pose.dwell_seconds > 0.0 &&
          time_at_target < current_target_pose.dwell_seconds) {
        // Still dwelling - don't advance yet
        // Will generate constant trajectory at target in
        // GenerateMoveToTargetTrajectory
      } else {
        // Dwell complete (or no dwell required), move to next target
        current_target_idx++;
        discrete_state->get_mutable_value(current_target_index_)[0] =
            static_cast<double>(current_target_idx);
        // Reset target reached time for next target
        discrete_state->get_mutable_value(target_reached_time_index_)[0] = -1.0;

        // Check if we've completed all targets
        if (current_target_idx >= static_cast<int>(target_poses_.size())) {
          std::cout << "All targets completed! Now switching to MPC phase."
                    << std::endl;
          // Could transition to a completion phase here if needed
          // For now, just stay at the last target
          // current_target_idx = static_cast<int>(target_poses_.size()) - 1;
          // discrete_state->get_mutable_value(current_target_index_)[0] =
          //     static_cast<double>(current_target_idx);
          current_phase_ = AssemblyPhase::kMPC;
          discrete_state->get_mutable_value(phase_index_)[0] =
              static_cast<double>(current_phase_);
        } else {
          std::cout << "Moving to target " << current_target_idx << std::endl;
        }
      }
    } else {
      // Not at target yet - reset target reached time
      if (target_reached_time >= 0.0) {
        discrete_state->get_mutable_value(target_reached_time_index_)[0] = -1.0;
      }
    }
  }
  switch (current_phase_) {
    case AssemblyPhase::kMoveToTarget:
      if (!target_poses_.empty() && current_target_idx >= 0 &&
          current_target_idx < static_cast<int>(target_poses_.size())) {
        GenerateMoveToTargetTrajectory(
            x_lcs_curr, t_context, &execution_lcm_traj_, current_target_idx);
      }
      break;
    case AssemblyPhase::kMPC:
      GenerateMPCTrajectory(x_lcs_curr, x_lcs_des, t_context,
                            &execution_lcm_traj_, &planned_keypoints_lcm_traj_);
      break;
  }

  return drake::systems::EventStatus::Succeeded();
}

bool AssemblyController::IsTargetReached(const Eigen::VectorXd& x_lcs_curr,
                                         int target_index) const {
  if (target_index < 0 ||
      target_index >= static_cast<int>(target_poses_.size())) {
    return false;
  }

  const TargetPose& target_pose = target_poses_[target_index];

  // Check position
  Eigen::Vector3d current_pos = x_lcs_curr.head(3);
  Eigen::Vector3d target_pos = target_pose.position;
  double distance = (current_pos - target_pos).norm();
  bool position_reached =
      distance < (target_pose.radius > 0.0 ? POSITION_TOLERANCE_CIRCULAR_ARC
                                           : POSITION_TOLERANCE);

  // Check orientation
  // Extract current orientation from x_lcs_curr (RPY at indices 3, 4, 5)
  Eigen::Vector3d rpy = x_lcs_curr.segment(3, 3);
  RollPitchYaw<double> rpy_obj(rpy);
  Quaterniond current_quat = rpy_obj.ToQuaternion();
  current_quat.normalize();

  // Get target orientation
  Eigen::Vector4d target_orientation_normalized = target_pose.orientation;
  target_orientation_normalized.normalize();
  Quaterniond target_quat(
      target_orientation_normalized(0), target_orientation_normalized(1),
      target_orientation_normalized(2), target_orientation_normalized(3));
  target_quat.normalize();

  // Compute angle between quaternions (taking shorter path)
  double dot_product = std::abs(current_quat.dot(target_quat));
  // Clamp to [-1, 1] to avoid numerical issues with acos
  dot_product = std::min(1.0, std::max(-1.0, dot_product));
  double angle = 2.0 * std::acos(dot_product);
  bool orientation_reached = angle < ORIENTATION_TOLERANCE;

  return position_reached && orientation_reached;
}

void AssemblyController::AddEETrajectoriesToLcm(
    const Eigen::MatrixXd& position_knots,
    const Eigen::MatrixXd& orientation_knots,
    const Eigen::MatrixXd& force_knots, const Eigen::VectorXd& timestamps,
    LcmTrajectory* traj) const {
  traj->ClearTrajectories();

  // Add end effector position trajectory
  LcmTrajectory::Trajectory ee_traj;
  ee_traj.traj_name = "end_effector_position_target";
  ee_traj.datatypes = std::vector<std::string>(position_knots.rows(), "double");
  ee_traj.datapoints = position_knots;
  ee_traj.time_vector = timestamps;
  traj->AddTrajectory(ee_traj.traj_name, ee_traj);

  // Add end effector orientation trajectory
  LcmTrajectory::Trajectory ee_orientation_traj;
  ee_orientation_traj.traj_name = "end_effector_orientation_target";
  ee_orientation_traj.datatypes =
      std::vector<std::string>(orientation_knots.rows(), "double");
  ee_orientation_traj.datapoints = orientation_knots;
  ee_orientation_traj.time_vector = timestamps;
  traj->AddTrajectory(ee_orientation_traj.traj_name, ee_orientation_traj);

  // Add end effector force trajectory
  LcmTrajectory::Trajectory force_traj;
  force_traj.traj_name = "end_effector_force_target";
  force_traj.datatypes = std::vector<std::string>(force_knots.rows(), "double");
  force_traj.datapoints = force_knots;
  force_traj.time_vector = timestamps;
  traj->AddTrajectory(force_traj.traj_name, force_traj);
}

void AssemblyController::GenerateMoveToTargetTrajectory(
    const Eigen::VectorXd& x_lcs_curr, double t_context, LcmTrajectory* traj,
    int target_index) const {
  if (target_index < 0 ||
      target_index >= static_cast<int>(target_poses_.size())) {
    std::cerr << "Warning: Invalid target index: " << target_index << std::endl;
    return;
  }

  // Get target pose from target_poses_ vector
  const TargetPose& target_pose = target_poses_[target_index];
  Eigen::Vector3d target_pos = target_pose.position;
  Eigen::Vector4d target_orientation = target_pose.orientation;

  // Get current position
  Eigen::Vector3d current_pos = x_lcs_curr.head(3);
  Eigen::Vector3d displacement = target_pos - current_pos;
  double distance = displacement.norm();

  if (verbose_) {
    std::cout << "Current EE position: " << current_pos.transpose()
              << std::endl;
    std::cout << "Target EE position: " << target_pos.transpose() << std::endl;
    std::cout << "Distance to target: " << distance << std::endl;
  }

  // Already at target, create a two-point trajectory at target position (OSC
  // requires at least two points)
  if (IsTargetReached(x_lcs_curr, target_index)) {
    gripper_pos_command_ = target_pose.gripper_pos_command;
    Eigen::MatrixXd knots = Eigen::MatrixXd::Zero(3, 2);
    Eigen::VectorXd timestamps = Eigen::VectorXd::Zero(2);
    knots.col(0) = target_pos;
    knots.col(1) = target_pos;
    timestamps[0] = t_context;
    timestamps[1] = t_context + dt_;

    // Create minimal trajectory
    Eigen::MatrixXd orientation_single = Eigen::MatrixXd::Zero(4, 2);
    orientation_single.col(0) = target_orientation;
    orientation_single.col(1) = target_orientation;
    Eigen::MatrixXd force_samples = Eigen::MatrixXd::Zero(3, 2);
    force_samples.col(0) = Eigen::Vector3d::Zero();
    force_samples.col(1) = Eigen::Vector3d::Zero();

    AddEETrajectoriesToLcm(knots, orientation_single, force_samples, timestamps,
                           traj);
    return;
  }

  // Generate trajectory - handle both position and orientation interpolation
  int n_knots = N_;
  Eigen::MatrixXd knots = Eigen::MatrixXd::Zero(3, n_knots);
  Eigen::VectorXd timestamps = Eigen::VectorXd::Zero(n_knots);

  bool position_reached =
      distance < (target_pose.radius > 0.0 ? POSITION_TOLERANCE_CIRCULAR_ARC
                                           : POSITION_TOLERANCE);
  if (position_reached) {
    for (int i = 0; i < n_knots; i++) {
      knots.col(i) = target_pos;
      timestamps[i] = t_context + i * dt_;
    }
  } else {
    // Check if circular arc interpolation should be used
    bool use_circular_arc = (target_pose.radius > 0.0);

    if (use_circular_arc) {
      // Circular arc interpolation using center and radius
      Eigen::Vector3d center = target_pose.center;
      double radius = target_pose.radius;

      // Compute vectors from center to current and target positions
      Eigen::Vector3d vec_current = current_pos - center;
      Eigen::Vector3d vec_target = target_pos - center;
      double dist_current = vec_current.norm();
      double dist_target = vec_target.norm();

      // Tolerance for checking if points are on the circle
      const double radius_tolerance = 0.01;

      // Verify that both points are approximately on the circle
      if (std::abs(dist_current - radius) > radius_tolerance ||
          std::abs(dist_target - radius) > radius_tolerance) {
        if (verbose_) {
          std::cout << "Warning: Current or target position not on circle. "
                    << "dist_current: " << dist_current
                    << ", dist_target: " << dist_target
                    << ", radius: " << radius
                    << ". Falling back to linear interpolation." << std::endl;
        }
        use_circular_arc = false;
      }

      if (use_circular_arc) {
        // Normalize the vectors
        Eigen::Vector3d u_current = vec_current / dist_current;
        Eigen::Vector3d u_target = vec_target / dist_target;

        // Compute the plane normal (normalized cross product)
        Eigen::Vector3d normal = u_current.cross(u_target);
        double normal_norm = normal.norm();

        // Check if vectors are collinear (would make cross product zero)
        if (normal_norm < 1e-6) {
          if (verbose_) {
            std::cout << "Warning: Current and target positions are collinear "
                         "with center. "
                      << "Falling back to linear interpolation." << std::endl;
          }
          use_circular_arc = false;
        } else {
          normal /= normal_norm;

          // Compute the angle between the two vectors
          double dot_product = u_current.dot(u_target);
          // Clamp to [-1, 1] to avoid numerical issues
          dot_product = std::min(1.0, std::max(-1.0, dot_product));
          double angle = std::acos(dot_product);
          double sin_angle = std::sin(angle);

          // Handle the case when angle is very small (near 0 or π)
          if (sin_angle < 1e-6) {
            // Vectors are nearly parallel, use linear interpolation as fallback
            if (verbose_) {
              std::cout << "Warning: Angle between vectors is too small for "
                           "circular interpolation. "
                        << "Falling back to linear interpolation." << std::endl;
            }
            use_circular_arc = false;
          } else {
            // Compute a vector perpendicular to u_current that points towards
            // u_target u_target = u_current * cos(angle) + u_perp * sin(angle)
            // Therefore: u_perp = (u_target - u_current * cos(angle)) /
            // sin(angle)
            Eigen::Vector3d u_perp =
                (u_target - u_current * dot_product) / sin_angle;
            // u_perp should be approximately unit length (within numerical
            // precision)

            // Interpolate along the circular arc
            for (int i = 0; i < n_knots; i++) {
              double t =
                  static_cast<double>(i) / static_cast<double>(n_knots - 1);
              double interp_angle = t * angle;
              double cos_interp = std::cos(interp_angle);
              double sin_interp = std::sin(interp_angle);

              // Rotate u_current towards u_target by interp_angle
              // rotated_vec = u_current * cos(interp_angle) + u_perp *
              // sin(interp_angle)
              Eigen::Vector3d rotated_vec =
                  u_current * cos_interp + u_perp * sin_interp;

              // Scale to radius and add center
              knots.col(i) = center + radius * rotated_vec;
              timestamps[i] = t_context + i * dt_;
            }

            if (verbose_) {
              const double pi = 3.14159265358979323846;
              std::cout << "Using circular arc interpolation. Center: "
                        << center.transpose() << ", Radius: " << radius
                        << ", Angle: " << angle << " radians ("
                        << angle * 180.0 / pi << " degrees)" << std::endl;
            }
          }
        }
      }
    }

    // Fall back to linear interpolation if circular arc is not used
    if (!use_circular_arc) {
      const double avg_speed = 0.1;
      double step_size = avg_speed * dt_;
      for (int i = 0; i < n_knots; i++) {
        knots.col(i) = current_pos + i * step_size * displacement.normalized();
        timestamps[i] = t_context + i * dt_;
      }
    }
  }

  // Interpolate orientation from current to target using spherical linear
  // interpolation (Slerp)
  // Extract current orientation from x_lcs_curr (RPY at indices 3, 4, 5)
  Eigen::Vector3d rpy = x_lcs_curr.segment(3, 3);
  RollPitchYaw<double> rpy_obj(rpy);
  Quaterniond q_current = rpy_obj.ToQuaternion();
  q_current.normalize();

  // Get target orientation
  Eigen::Vector4d target_orientation_normalized = target_orientation;
  target_orientation_normalized.normalize();
  Quaterniond q_target(
      target_orientation_normalized(0), target_orientation_normalized(1),
      target_orientation_normalized(2), target_orientation_normalized(3));
  q_target.normalize();

  // Ensure we take the shorter path (if dot product is negative, negate one
  // quaternion)
  if (q_current.dot(q_target) < 0.0) {
    q_target =
        Quaterniond(-q_target.w(), -q_target.x(), -q_target.y(), -q_target.z());
  }

  Eigen::MatrixXd ee_orientations = Eigen::MatrixXd::Zero(4, n_knots);

  // Interpolate orientation during movement phase using Slerp
  for (int i = 0; i < n_knots; i++) {
    double t = static_cast<double>(i) / static_cast<double>(n_knots - 1);
    Quaterniond q_interp = q_current.slerp(t, q_target);
    q_interp.normalize();
    ee_orientations.col(i) =
        Eigen::Vector4d(q_interp.w(), q_interp.x(), q_interp.y(), q_interp.z());
  }

  // Zero forces (or could be set based on target pose requirements)
  Eigen::MatrixXd force_samples = Eigen::MatrixXd::Zero(3, n_knots);

  // Add all trajectories to LCM
  AddEETrajectoriesToLcm(knots, ee_orientations, force_samples, timestamps,
                         traj);
}

void AssemblyController::GenerateMPCTrajectory(
    const Eigen::VectorXd& x_lcs_curr, const Eigen::VectorXd& x_lcs_des,
    double t_context, LcmTrajectory* traj,
    LcmTrajectory* planned_keypoints_traj) const {
  if (abs(x_lcs_curr.head(3).norm() - x_lcs_des.head(3).norm()) < 0.001) {
    gripper_pos_command_ = 0.04;
    return;
  }
  // Update context to current state
  UpdateContext(n_q_, n_v_, n_u_, plant_, context_, plant_ad_, context_ad_,
                x_lcs_curr);

  // Resolve contact pairs and create LCS
  vector<SortedPair<GeometryId>> resolved_contact_pairs =
      LCSFactory::PreProcessor(plant_, *context_, contact_pairs_,
                               assembly_c3_options_.resolve_contacts_to,
                               assembly_c3_options_.num_friction_directions,
                               false);

  // Create LCS object
  // Using simplified parameters - for more complex scenarios, these would
  // come from configuration options
  solvers::LCS lcs_object = LCSFactory::LinearizePlantToLCS(
      plant_, *context_, plant_ad_, *context_ad_, resolved_contact_pairs,
      assembly_c3_options_.mu, dt_, N_,
      assembly_c3_options_.n_lambda_with_tangential,
      assembly_c3_options_.num_friction_directions_per_contact,
      assembly_c3_options_.starting_index_per_contact_in_lambda_t_vector,
      contact_model_);

  // Update LCS model, desired state, and then solve C3
  std::vector<VectorXd> x_desired = std::vector<VectorXd>(N_ + 1, x_lcs_des);
  c3_mpc_->UpdateLCS(lcs_object);
  c3_mpc_->UpdateTarget(x_desired);
  c3_mpc_->Solve(x_lcs_curr, false);

  // Get solution
  vector<VectorXd> u_sol = c3_mpc_->GetInputSolution();
  vector<VectorXd> x_sol = c3_mpc_->GetStateSolution();

  // Set up trajectory
  Eigen::MatrixXd knots = Eigen::MatrixXd::Zero(3, N_);
  Eigen::VectorXd timestamps = Eigen::VectorXd::Zero(N_);

  for (int i = 0; i < N_; i++) {
    knots.col(i) = x_sol[i].head(3);
    timestamps[i] = t_context + i * dt_;
  }

  // End effector orientation (from RPY in x_sol)
  Eigen::MatrixXd ee_orientations = Eigen::MatrixXd::Zero(4, N_);
  for (int i = 0; i < N_; i++) {
    Eigen::Vector3d rpy = x_sol[i].segment(3, 3);
    RollPitchYaw<double> rpy_obj(rpy);
    Quaterniond quat = rpy_obj.ToQuaternion();
    quat.normalize();
    Eigen::Vector4d q_vec;
    q_vec << quat.w(), quat.x(), quat.y(), quat.z();
    ee_orientations.col(i) = q_vec;
  }

  // Create trajectory
  traj->ClearTrajectories();
  LcmTrajectory::Trajectory ee_traj;
  ee_traj.traj_name = "end_effector_position_target";
  ee_traj.datatypes = std::vector<std::string>(3, "double");
  ee_traj.datapoints = knots;
  ee_traj.time_vector = timestamps;

  LcmTrajectory::Trajectory ee_orientation_traj;
  ee_orientation_traj.traj_name = "end_effector_orientation_target";
  ee_orientation_traj.datatypes = std::vector<std::string>(4, "double");
  ee_orientation_traj.datapoints = ee_orientations;
  ee_orientation_traj.time_vector = timestamps;

  traj->AddTrajectory(ee_traj.traj_name, ee_traj);
  traj->AddTrajectory(ee_orientation_traj.traj_name, ee_orientation_traj);

  // Add force trajectory from C3 solution
  Eigen::MatrixXd force_samples = Eigen::MatrixXd::Zero(3, N_);
  for (int i = 0; i < N_; i++) {
    force_samples.col(i) = u_sol[i];
  }
  LcmTrajectory::Trajectory force_traj;
  force_traj.traj_name = "end_effector_force_target";
  force_traj.datatypes = std::vector<std::string>(3, "double");
  force_traj.datapoints = force_samples;
  force_traj.time_vector = timestamps;
  traj->AddTrajectory(force_traj.traj_name, force_traj);

  // Add planned keypoints trajectory
  planned_keypoints_traj->ClearTrajectories();
  LcmTrajectory::Trajectory keypoints_traj;
  keypoints_traj.traj_name = "planned_keypoints";
  keypoints_traj.datatypes = std::vector<std::string>(3, "double");
  Eigen::MatrixXd keypoint_knots = Eigen::MatrixXd::Zero(3, N_);
  for (int i = 0; i < N_; i++) {
    keypoint_knots.col(i) = x_sol[i].segment(6, 3);
  }
  keypoints_traj.datapoints = keypoint_knots;
  keypoints_traj.time_vector = timestamps;
  planned_keypoints_traj->AddTrajectory(keypoints_traj.traj_name,
                                        keypoints_traj);
}

void AssemblyController::OutputTrajExecute(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* output) const {
  output->saved_traj = execution_lcm_traj_.GenerateLcmObject();
  output->utime = context.get_time() * 1e6;
}

void AssemblyController::OutputTrajPlannedKeypoints(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* output) const {
  output->saved_traj = planned_keypoints_lcm_traj_.GenerateLcmObject();
  output->utime = context.get_time() * 1e6;
}

void AssemblyController::OutputGripperPosCommand(
    const drake::systems::Context<double>& context,
    drake::lcmt_schunk_wsg_command* output) const {
  output->utime = context.get_time() * 1e6;
  output->target_position_mm = gripper_pos_command_ * 1000.0;
}

}  // namespace magna
}  // namespace examples
}  // namespace dairlib
