#include "assembly_controller.h"

#include <Eigen/Dense>

#include "common/update_context.h"
#include "systems/framework/timestamped_vector.h"

#define POSITION_TOLERANCE 0.005

namespace dairlib {
namespace magna {

using drake::SortedPair;
using drake::geometry::GeometryId;
using drake::multibody::MultibodyPlant;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteValues;
using Eigen::MatrixXd;
using Eigen::Vector3d;
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
    const C3Options& c3_options)
    : plant_(plant),
      context_(context),
      plant_ad_(plant_ad),
      context_ad_(context_ad),
      contact_pairs_(contact_geoms),
      c3_options_(c3_options),
      N_(c3_options.N),
      dt_(c3_options.dt) {
  this->set_name("assembly_controller");

  n_q_ = plant_.num_positions();
  n_v_ = plant_.num_velocities();
  n_u_ = plant_.num_actuators();
  n_x_ = n_q_ + n_v_;

  N_ = 10;
  dt_ = 0.01;

  // Determine contact model and n_lambda_
  // if (c3_options_.contact_model == "stewart_and_trinkle") {
  //   contact_model_ = solvers::ContactModel::kStewartAndTrinkle;
  //   n_lambda_ =
  //       2 * c3_options_.num_contacts +
  //       2 * c3_options_.num_friction_directions * c3_options_.num_contacts;
  // } else if (c3_options_.contact_model == "anitescu") {
  //   contact_model_ = solvers::ContactModel::kAnitescu;
  //   n_lambda_ =
  //       2 * c3_options_.num_friction_directions * c3_options_.num_contacts;
  // } else {
  //   std::cerr << "Unknown or unsupported contact model: "
  //             << c3_options_.contact_model << std::endl;
  //   DRAKE_THROW_UNLESS(false);
  // }
  // if (c3_options_.contact_model == "stewart_and_trinkle") {
  //   contact_model_ = solvers::ContactModel::kStewartAndTrinkle;
  //   n_lambda_ =
  //       2 * c3_options_.num_contacts +
  //       2 * c3_options_.num_friction_directions * c3_options_.num_contacts;
  // } else {
  //   contact_model_ = solvers::ContactModel::kAnitescu;
  //   n_lambda_ =
  //       2 * c3_options_.num_friction_directions * c3_options_.num_contacts;
  // }
  // // Initialize cost matrices
  // double discount_factor = 1;
  // for (int i = 0; i < N_ + 1; ++i) {
  //   Q_.push_back(discount_factor * c3_options_.Q);
  //   discount_factor *= c3_options_.gamma;
  //   if (i < N_) {
  //     R_.push_back(discount_factor * c3_options_.R);
  //     G_.push_back(c3_options_.G);
  //     U_.push_back(c3_options_.U);
  //   }
  // }

  // // Create placeholder LCS for initializing C3Plus
  // MatrixXd A = MatrixXd::Ones(n_x_, n_x_);
  // MatrixXd B = MatrixXd::Zero(n_x_, n_u_);
  // VectorXd d = VectorXd::Zero(n_x_);
  // MatrixXd D = MatrixXd::Ones(n_x_, n_lambda_);
  // MatrixXd E = MatrixXd::Zero(n_lambda_, n_x_);
  // MatrixXd F = MatrixXd::Zero(n_lambda_, n_lambda_);
  // MatrixXd H = MatrixXd::Zero(n_lambda_, n_u_);
  // VectorXd c = VectorXd::Zero(n_lambda_);
  // LCS lcs_placeholder(A, B, D, d, E, F, H, c, N_, dt_);

  // auto x_desired_placeholder =
  //     std::vector<VectorXd>(N_ + 1, VectorXd::Zero(n_x_));
  // c3_mpc_ = std::make_shared<C3Plus>(
  //     lcs_placeholder, solvers::C3Base::CostMatrices(Q_, R_, G_, U_),
  //     x_desired_placeholder, c3_options_);

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

  gripper_pos_command_port_ =
      this->DeclareAbstractOutputPort(
              "gripper_pos_command",
              dairlib::lcmt_franka_hand_target_position(),
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

  target_poses_.push_back(TargetPose{Eigen::Vector3d(0.5, 0.27, -0.01),
                                     Eigen::Vector4d(1.0, 0.0, 0.0, 0.0), -0.1,
                                     2.0});
  target_poses_.push_back(TargetPose{Eigen::Vector3d(0.5, 0.15, 0.25),
                                     Eigen::Vector4d(1.0, 0.0, 0.0, 0.0), -0.1,
                                     0.0});
  target_poses_.push_back(TargetPose{Eigen::Vector3d(0.5, 0.047, 0.25),
                                     Eigen::Vector4d(1.0, 0.0, 0.0, 0.0), -0.1,
                                     0.0});
  target_poses_.push_back(TargetPose{Eigen::Vector3d(0.5, 0.047, 0.198),
                                     Eigen::Vector4d(1.0, 0.0, 0.0, 0.0), -0.1,
                                     0.5});
  target_poses_.push_back(TargetPose{Eigen::Vector3d(0.5, 0.12, 0.15),
                                     Eigen::Vector4d(1.0, 0.0, 0.0, 0.0), -0.1,
                                     0.0});
  target_poses_.push_back(TargetPose{Eigen::Vector3d(0.5, 0.16, 0.05),
                                     Eigen::Vector4d(1.0, 0.0, 0.0, 0.0), -0.1,
                                     0.0});
}

drake::systems::EventStatus AssemblyController::ComputePlan(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  // Evaluate input ports
  const TimestampedVector<double>* lcs_x_curr =
      (TimestampedVector<double>*)this->EvalVectorInput(context,
                                                        lcs_state_input_port_);

  drake::VectorX<double> x_lcs_curr = lcs_x_curr->get_value();
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
      current_target_idx < static_cast<int>(target_poses_.size())) {
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
          std::cout << "All targets completed!" << std::endl;
          // Could transition to a completion phase here if needed
          // For now, just stay at the last target
          current_target_idx = static_cast<int>(target_poses_.size()) - 1;
          discrete_state->get_mutable_value(current_target_index_)[0] =
              static_cast<double>(current_target_idx);
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

  // Generate trajectory based on current phase
  execution_lcm_traj_.ClearTrajectories();

  switch (current_phase_) {
    case AssemblyPhase::kMoveToTarget:
      if (!target_poses_.empty() && current_target_idx >= 0 &&
          current_target_idx < static_cast<int>(target_poses_.size())) {
        GenerateMoveToTargetTrajectory(
            x_lcs_curr, t_context, &execution_lcm_traj_, current_target_idx);
      }
      break;
      // case AssemblyPhase::kMPC:
      //   GenerateMPCTrajectory(x_lcs_curr, x_lcs_des.get_value(), t_context,
      //                         &execution_lcm_traj_);
      //   break;
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
  Eigen::Vector3d current_pos = x_lcs_curr.head(3);
  Eigen::Vector3d target_pos = target_pose.position;
  double distance = (current_pos - target_pos).norm();

  return distance < POSITION_TOLERANCE;
}

void AssemblyController::AddEETrajectoriesToLcm(
    const Eigen::MatrixXd& position_knots,
    const Eigen::MatrixXd& orientation_knots,
    const Eigen::MatrixXd& force_knots, const Eigen::VectorXd& timestamps,
    LcmTrajectory* traj) const {
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
  // Validate target index
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

  std::cout << "current_pos: " << current_pos.transpose() << std::endl;
  std::cout << "target_pos: " << target_pos.transpose() << std::endl;
  std::cout << "distance: " << distance << std::endl;

  // Check if already at target (within tolerance)
  // NOTE: Since this function is called every control loop, we generate a
  // trajectory from the current position. If we're at the target, we generate a
  // constant trajectory that holds the target position. The dwell time is
  // handled by ComputePlan() which delays advancing to the next target.
  if (distance < POSITION_TOLERANCE) {
    gripper_pos_command_ = target_pose.gripper_pos_command;
    // Already at target, create a single-point trajectory at target position
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

  // We generate a straight line trajectory from current position to target
  const double avg_speed = 0.1;
  double step_size = avg_speed * dt_;

  int n_knots = N_;
  Eigen::MatrixXd knots = Eigen::MatrixXd::Zero(3, n_knots);
  Eigen::VectorXd timestamps = Eigen::VectorXd::Zero(n_knots);

  // Generate movement trajectory with smooth interpolation
  for (int i = 0; i < n_knots; i++) {
    knots.col(i) = current_pos + i * step_size * displacement.normalized();
    timestamps[i] = t_context + i * dt_;
  }

  // Interpolate orientation from current to target using spherical linear
  // interpolation (Slerp) For simplicity, we'll use linear interpolation for
  // quaternion components (For production code, proper quaternion slerp would
  // be better)
  Eigen::Vector4d current_orientation(1.0, 0.0, 0.0,
                                      0.0);  // Default to identity
  // If x_lcs_curr has orientation information, extract it (assuming it's in
  // positions) For now, we'll interpolate from identity to target

  Eigen::MatrixXd ee_orientations = Eigen::MatrixXd::Zero(4, n_knots);

  // Interpolate orientation during movement phase
  for (int i = 0; i < n_knots; i++) {
    // double t_normalized = static_cast<double>(i) / (n_move_knots - 1);
    // double smooth_alpha =
    //     t_normalized * t_normalized * (3.0 - 2.0 * t_normalized);

    // Simple linear interpolation for quaternion (normalized)
    Eigen::Vector4d q_interp = target_orientation;
    q_interp.normalize();
    ee_orientations.col(i) = q_interp;
  }

  // Zero forces (or could be set based on target pose requirements)
  Eigen::MatrixXd force_samples = Eigen::MatrixXd::Zero(3, n_knots);

  // Add all trajectories to LCM
  AddEETrajectoriesToLcm(knots, ee_orientations, force_samples, timestamps,
                         traj);
}

// void AssemblyController::GenerateMPCTrajectory(
//     const Eigen::VectorXd& x_lcs_curr, const Eigen::VectorXd& x_lcs_des,
//     double t_context, LcmTrajectory* traj) const {
//   // Update context to current state
//   UpdateContext(n_q_, n_v_, n_u_, plant_, context_, plant_ad_, context_ad_,
//                 x_lcs_curr);

//   // Resolve contact pairs and create LCS
//   vector<SortedPair<GeometryId>> resolved_contact_pairs =
//       LCSFactory::PreProcessor(plant_, *context_, contact_pairs_,
//                                "actor",  // resolve_contacts_to
//                                c3_options_.num_friction_directions, false);

//   // Create LCS object
//   // Using simplified parameters - for more complex scenarios, these would
//   come
//   // from configuration options
//   solvers::LCS lcs_object = LCSFactory::LinearizePlantToLCS(
//       plant_, *context_, plant_ad_, *context_ad_, resolved_contact_pairs,
//       c3_options_.mu, dt_, N_, 0, {}, {}, contact_model_);

//   // Update cost matrices
//   std::vector<VectorXd> x_desired = std::vector<VectorXd>(N_ + 1, x_lcs_des);

//   // Update C3Plus with new LCS
//   c3_mpc_->UpdateLCS(lcs_object);
//   c3_mpc_->UpdateCostLCS(lcs_object);

//   // Update desired state
//   c3_mpc_->UpdateTarget(x_desired);

//   // Solve C3
//   c3_mpc_->Solve(x_lcs_curr, false);

//   // Get solution
//   vector<VectorXd> u_sol = c3_mpc_->GetInputSolution();
//   vector<VectorXd> x_sol = c3_mpc_->GetStateSolution();

//   // Set up trajectory
//   Eigen::MatrixXd knots = Eigen::MatrixXd::Zero(3, N_);
//   Eigen::VectorXd timestamps = Eigen::VectorXd::Zero(N_);

//   for (int i = 0; i < N_; i++) {
//     knots.col(i) = x_sol[i].head(3);
//     timestamps[i] = t_context + i * dt_;
//   }

//   // End effector orientation (keep vertical)
//   Eigen::MatrixXd ee_orientations = Eigen::MatrixXd::Zero(4, N_);
//   Eigen::Vector4d q_vertical(1.0, 0.0, 0.0, 0.0);
//   for (int i = 0; i < N_; i++) {
//     ee_orientations.col(i) = q_vertical;
//   }

//   // Create trajectory
//   LcmTrajectory::Trajectory ee_traj;
//   ee_traj.traj_name = "end_effector_position_target";
//   ee_traj.datatypes = std::vector<std::string>(3, "double");
//   ee_traj.datapoints = knots;
//   ee_traj.time_vector = timestamps;

//   LcmTrajectory::Trajectory ee_orientation_traj;
//   ee_orientation_traj.traj_name = "end_effector_orientation_target";
//   ee_orientation_traj.datatypes = std::vector<std::string>(4, "double");
//   ee_orientation_traj.datapoints = ee_orientations;
//   ee_orientation_traj.time_vector = timestamps;

//   traj->AddTrajectory(ee_traj.traj_name, ee_traj);
//   traj->AddTrajectory(ee_orientation_traj.traj_name, ee_orientation_traj);

//   // Add force trajectory from C3 solution
//   Eigen::MatrixXd force_samples = Eigen::MatrixXd::Zero(3, N_);
//   for (int i = 0; i < N_; i++) {
//     force_samples.col(i) = u_sol[i];
//   }
//   LcmTrajectory::Trajectory force_traj;
//   force_traj.traj_name = "end_effector_force_target";
//   force_traj.datatypes = std::vector<std::string>(3, "double");
//   force_traj.datapoints = force_samples;
//   force_traj.time_vector = timestamps;
//   traj->AddTrajectory(force_traj.traj_name, force_traj);
// }

void AssemblyController::OutputTrajExecute(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* output) const {
  output->saved_traj = execution_lcm_traj_.GenerateLcmObject();
  output->utime = context.get_time() * 1e6;
}

void AssemblyController::OutputGripperPosCommand(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_franka_hand_target_position* output) const {
  output->utime = context.get_time() * 1e6;
  output->hand_target_position[0] = gripper_pos_command_;
}

}  // namespace magna
}  // namespace dairlib
