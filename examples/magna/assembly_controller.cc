#include "assembly_controller.h"

#include <cmath>

#include <Eigen/Dense>
#include <drake/math/roll_pitch_yaw.h>

#include "common/update_context.h"
#include "dairlib/lcmt_c3_forces.hpp"
#include "dairlib/lcmt_force.hpp"
#include "dairlib/lcmt_round_belt_state.hpp"
#include "examples/magna/parameter_headers/assembly_c3_options.h"
#include "examples/magna/parameter_headers/osc_target_poses.h"
#include "solvers/c3_output.h"
#include "systems/framework/timestamped_vector.h"

#include "drake/lcmt_schunk_wsg_command.hpp"

namespace dairlib {
namespace examples {
namespace magna {

using dairlib::C3Output;
using drake::SortedPair;
using drake::geometry::GeometryId;
using drake::math::RollPitchYaw;
using drake::multibody::MultibodyPlant;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteValues;
using Eigen::MatrixXd;
using Eigen::MatrixXf;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using Eigen::VectorXf;
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
    const RoundBeltControllerParams& round_belt_controller_params, bool verbose)
    : plant_(plant),
      context_(context),
      plant_ad_(plant_ad),
      context_ad_(context_ad),
      contact_pairs_(contact_geoms),
      round_belt_controller_params_(round_belt_controller_params),
      assembly_c3_options_(round_belt_controller_params.assembly_c3_options),
      N_(round_belt_controller_params.assembly_c3_options.N),
      dt_(round_belt_controller_params.assembly_c3_options.dt),
      mpc_target_lcs_states_(round_belt_controller_params.target_lcs_states),
      osc_target_poses_pre_mpc_(
          round_belt_controller_params.osc_target_poses_pre_mpc),
      osc_target_poses_post_mpc_(
          round_belt_controller_params.osc_target_poses_post_mpc),
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
  n_z_ = c3_mpc_->GetZSize();

  // Add linear constraints for horizontal force components (x, y)
  for (int i = 0; i < 2; ++i) {
    Eigen::RowVectorXd A = VectorXd::Zero(n_u_);
    A(i) = 1.0;
    c3_mpc_->AddLinearConstraint(A, c3_options.u_horizontal_limits[0],
                                 c3_options.u_horizontal_limits[1], 2);
  }

  // Add linear constraint for vertical force component (z)
  Eigen::RowVectorXd A_vertical = VectorXd::Zero(n_u_);
  A_vertical(2) = 1.0;
  c3_mpc_->AddLinearConstraint(A_vertical, c3_options.u_vertical_limits[0],
                               c3_options.u_vertical_limits[1], 2);

  // Input ports
  lcs_state_input_port_ =
      this->DeclareVectorInputPort("x_lcs", TimestampedVector<double>(n_x_))
          .get_index();
  task_relevant_keypoints_input_port_ =
      this->DeclareAbstractInputPort(
              "task_relevant_keypoints",
              drake::Value<dairlib::lcmt_round_belt_state>{})
          .get_index();

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

  traj_planned_port_ =
      this->DeclareAbstractOutputPort("traj_planned",
                                      dairlib::lcmt_timestamped_saved_traj(),
                                      &AssemblyController::OutputTrajPlanned)
          .get_index();

  gripper_pos_command_port_ =
      this->DeclareAbstractOutputPort(
              "gripper_pos_command", drake::lcmt_schunk_wsg_command(),
              &AssemblyController::OutputGripperPosCommand)
          .get_index();

  c3_forces_port_ =
      this->DeclareAbstractOutputPort("c3_forces", dairlib::lcmt_c3_forces(),
                                      &AssemblyController::OutputC3Forces)
          .get_index();

  current_target_lcs_state_port_ =
      this->DeclareVectorOutputPort(
              "current_target_lcs_state", n_x_,
              &AssemblyController::OutputCurrentTargetLcsState)
          .get_index();

  // C3 intermediates output port
  auto c3_intermediates = C3Output::C3Intermediates();
  c3_intermediates.z_ = MatrixXf::Zero(n_z_, N_);
  c3_intermediates.w_ = MatrixXf::Zero(n_z_, N_);
  c3_intermediates.delta_ = MatrixXf::Zero(n_z_, N_);
  c3_intermediates.time_vector_ = VectorXf::Zero(N_);
  c3_intermediates_port_ = this->DeclareAbstractOutputPort(
                                   "c3_intermediates", c3_intermediates,
                                   &AssemblyController::OutputC3Intermediates)
                               .get_index();

  // C3 solution output port
  auto c3_solution = C3Output::C3Solution();
  c3_solution.x_sol_ = MatrixXf::Zero(n_x_, N_);
  c3_solution.lambda_sol_ = MatrixXf::Zero(n_lambda_, N_);
  c3_solution.u_sol_ = MatrixXf::Zero(n_u_, N_);
  c3_solution.time_vector_ = VectorXf::Zero(N_);
  c3_solution_port_ =
      this->DeclareAbstractOutputPort("c3_solution", c3_solution,
                                      &AssemblyController::OutputC3Solution)
          .get_index();

  // Discrete state for phase tracking
  // Initialize phase based on whether we have pre-MPC targets
  AssemblyPhase initial_phase = osc_target_poses_pre_mpc_.empty()
                                    ? AssemblyPhase::kMPC
                                    : AssemblyPhase::kMoveToTargetPreMPC;
  phase_index_ = this->DeclareDiscreteState(
      Eigen::VectorXd::Constant(1, static_cast<double>(initial_phase)));
  plan_start_time_index_ = this->DeclareDiscreteState(1);
  current_target_index_ = this->DeclareDiscreteState(1);
  // Initialize target_reached_time to -1 (not reached yet) - negative means not
  // reached
  target_reached_time_index_ =
      this->DeclareDiscreteState(Eigen::VectorXd::Constant(1, -1.0));
  // Initialize MPC target index to 0
  mpc_current_target_index_ = this->DeclareDiscreteState(1);
  // Initialize post-MPC target tracking
  post_mpc_target_index_ = this->DeclareDiscreteState(1);
  post_mpc_target_reached_time_index_ =
      this->DeclareDiscreteState(Eigen::VectorXd::Constant(1, -1.0));

  this->DeclareForcedDiscreteUpdateEvent(&AssemblyController::ComputePlan);

  // Initialize values of output ports
  execution_lcm_traj_ = dairlib::LcmTrajectory();
  planned_keypoints_lcm_traj_ = dairlib::LcmTrajectory();
  planned_lcm_traj_ = dairlib::LcmTrajectory();
  c3_forces_output_ = dairlib::lcmt_c3_forces();

  if (osc_target_poses_pre_mpc_.empty()) {
    std::cout << "No pre-MPC targets found. Initial phase set to MPC."
              << std::endl;
  } else {
    std::cout << "Found " << osc_target_poses_pre_mpc_.size()
              << " pre-MPC target(s). Initial phase set to MoveToTargetPreMPC."
              << std::endl;
  }
  if (!osc_target_poses_post_mpc_.empty()) {
    std::cout << "Found " << osc_target_poses_post_mpc_.size()
              << " post-MPC target(s)." << std::endl;
  }

  // Retrieve the height of the small pulley's groove
  // TODO: Remove hardcoded value 0.0248
  groove_height_ =
      plant_.GetBodyByName("board").EvalPoseInWorld(*context_).translation()(
          2) +
      0.0248;
}

AssemblyController::DiscreteStateData AssemblyController::ExtractDiscreteState(
    const drake::systems::DiscreteValues<double>& discrete_state) const {
  DiscreteStateData data;
  data.plan_start_time = discrete_state.get_value(plan_start_time_index_)[0];
  data.current_phase = static_cast<AssemblyPhase>(
      static_cast<int>(discrete_state.get_value(phase_index_)[0]));
  data.current_target_idx =
      static_cast<int>(discrete_state.get_value(current_target_index_)[0]);
  data.target_reached_time =
      discrete_state.get_value(target_reached_time_index_)[0];
  data.mpc_current_target_idx =
      static_cast<int>(discrete_state.get_value(mpc_current_target_index_)[0]);
  data.post_mpc_target_idx =
      static_cast<int>(discrete_state.get_value(post_mpc_target_index_)[0]);
  data.post_mpc_target_reached_time =
      discrete_state.get_value(post_mpc_target_reached_time_index_)[0];
  return data;
}

void AssemblyController::UpdateDiscreteState(
    const DiscreteStateData& data,
    drake::systems::DiscreteValues<double>* discrete_state) const {
  discrete_state->get_mutable_value(plan_start_time_index_)[0] =
      data.plan_start_time;
  discrete_state->get_mutable_value(phase_index_)[0] =
      static_cast<double>(data.current_phase);
  discrete_state->get_mutable_value(current_target_index_)[0] =
      static_cast<double>(data.current_target_idx);
  discrete_state->get_mutable_value(target_reached_time_index_)[0] =
      data.target_reached_time;
  discrete_state->get_mutable_value(mpc_current_target_index_)[0] =
      static_cast<double>(data.mpc_current_target_idx);
  discrete_state->get_mutable_value(post_mpc_target_index_)[0] =
      static_cast<double>(data.post_mpc_target_idx);
  discrete_state->get_mutable_value(post_mpc_target_reached_time_index_)[0] =
      data.post_mpc_target_reached_time;
}

bool AssemblyController::ShouldAdvanceToNextOSCTarget(
    const Eigen::VectorXd& x_lcs_curr, double current_time, int target_index,
    double target_reached_time,
    const std::vector<SingleOSCTargetPose>& target_poses) const {
  // Validate target index
  if (target_index < 0 ||
      target_index >= static_cast<int>(target_poses.size())) {
    return false;
  }

  // Check if target is reached
  if (!IsTargetReached(x_lcs_curr, target_index, target_poses)) {
    return false;
  }

  // Target is reached - check dwell time
  const SingleOSCTargetPose& target_pose = target_poses[target_index];

  // If no dwell time required, can advance immediately
  if (target_pose.dwell_seconds <= 0.0) {
    return true;
  }

  // If dwell time required but we just reached it, don't advance yet
  if (target_reached_time < 0.0) {
    return false;
  }

  // Check if dwell time has elapsed
  double time_at_target = current_time - target_reached_time;
  return time_at_target >= target_pose.dwell_seconds;
}

drake::systems::EventStatus AssemblyController::ComputePlan(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  // Extract input data
  const TimestampedVector<double>* lcs_x_curr =
      static_cast<const TimestampedVector<double>*>(
          this->EvalVectorInput(context, lcs_state_input_port_));

  const drake::VectorX<double> x_lcs_curr = lcs_x_curr->get_data();
  const double t_context = lcs_x_curr->get_timestamp();

  // Extract task-relevant keypoints from input port
  const auto& task_relevant_keypoints_msg =
      this->EvalInputValue<dairlib::lcmt_round_belt_state>(
          context, task_relevant_keypoints_input_port_);
  if (task_relevant_keypoints_msg != nullptr) {
    const int num_points = task_relevant_keypoints_msg->num_points;
    task_relevant_keypoints_.clear();
    task_relevant_keypoints_.resize(num_points);
    double z_sum = 0.0;
    for (int i = 0; i < num_points; ++i) {
      task_relevant_keypoints_[i] = {
          task_relevant_keypoints_msg->point_positions[i][0],
          task_relevant_keypoints_msg->point_positions[i][1],
          task_relevant_keypoints_msg->point_positions[i][2]};
      z_sum += task_relevant_keypoints_msg->point_positions[i][2];
    }
    task_relevant_keypoints_avg_z_ =
        (num_points > 0) ? z_sum / num_points : 0.03023;  // default value
    target_ee_height_ =
        groove_height_ + x_lcs_curr[2] - task_relevant_keypoints_avg_z_;
  }

  // Extract and validate discrete state
  DiscreteStateData state_data = ExtractDiscreteState(*discrete_state);
  state_data.plan_start_time = t_context;

  // If no pre-MPC targets exist and we're in pre-MPC phase, switch to MPC
  if (osc_target_poses_pre_mpc_.empty()) {
    if (state_data.current_phase == AssemblyPhase::kMoveToTargetPreMPC) {
      std::cout << "No pre-MPC targets available. Switching to MPC phase."
                << std::endl;
      state_data.current_phase = AssemblyPhase::kMPC;
    }
  } else {
    // Validate and correct pre-MPC target index if we have targets
    if (state_data.current_phase == AssemblyPhase::kMoveToTargetPreMPC &&
        (state_data.current_target_idx < 0 ||
         state_data.current_target_idx >=
             static_cast<int>(osc_target_poses_pre_mpc_.size()))) {
      state_data.current_target_idx = 0;
      state_data.target_reached_time = -1.0;
    }
  }

  // Validate post-MPC target index if we have targets
  if (state_data.current_phase == AssemblyPhase::kMoveToTargetPostMPC &&
      !osc_target_poses_post_mpc_.empty()) {
    if (state_data.post_mpc_target_idx < 0 ||
        state_data.post_mpc_target_idx >=
            static_cast<int>(osc_target_poses_post_mpc_.size())) {
      state_data.post_mpc_target_idx = 0;
      state_data.post_mpc_target_reached_time = -1.0;
    }
  }

  // Handle target progression in kMoveToTargetPreMPC phase
  if (!osc_target_poses_pre_mpc_.empty() &&
      state_data.current_phase == AssemblyPhase::kMoveToTargetPreMPC &&
      state_data.current_target_idx >= 0 &&
      state_data.current_target_idx <
          static_cast<int>(osc_target_poses_pre_mpc_.size())) {
    // Check if we just reached the target
    if (IsTargetReached(x_lcs_curr, state_data.current_target_idx,
                        osc_target_poses_pre_mpc_)) {
      // Record the time when target was first reached
      if (state_data.target_reached_time < 0.0) {
        state_data.target_reached_time = t_context;
        std::cout << "Pre-MPC target " << state_data.current_target_idx
                  << " reached at t=" << state_data.target_reached_time
                  << std::endl;
      }

      // Check if we should advance to the next target
      if (ShouldAdvanceToNextOSCTarget(
              x_lcs_curr, t_context, state_data.current_target_idx,
              state_data.target_reached_time, osc_target_poses_pre_mpc_)) {
        state_data.current_target_idx++;
        state_data.target_reached_time = -1.0;

        // Check if all pre-MPC targets are completed
        if (state_data.current_target_idx >=
            static_cast<int>(osc_target_poses_pre_mpc_.size())) {
          std::cout << "All pre-MPC targets completed! Switching to MPC phase."
                    << std::endl;
          state_data.current_phase = AssemblyPhase::kMPC;

          // Store the last OSC target pose for warm-up holding
          const auto& last_osc_target =
              osc_target_poses_pre_mpc_[osc_target_poses_pre_mpc_.size() - 1];
          warmup_hold_position_ = last_osc_target.position;
          warmup_hold_orientation_ = last_osc_target.orientation;
          warmup_hold_orientation_.normalize();
          warmup_pose_initialized_ = true;
        } else {
          std::cout << "Moving to pre-MPC target "
                    << state_data.current_target_idx << std::endl;
        }
      }
    } else {
      // Not at target yet - reset target reached time if it was set
      if (state_data.target_reached_time >= 0.0) {
        state_data.target_reached_time = -1.0;
      }
    }
  }

  // Handle target progression in kMoveToTargetPostMPC phase
  if (!osc_target_poses_post_mpc_.empty() &&
      state_data.current_phase == AssemblyPhase::kMoveToTargetPostMPC &&
      state_data.post_mpc_target_idx >= 0 &&
      state_data.post_mpc_target_idx <
          static_cast<int>(osc_target_poses_post_mpc_.size())) {
    // Check if we just reached the target
    if (IsTargetReached(x_lcs_curr, state_data.post_mpc_target_idx,
                        osc_target_poses_post_mpc_)) {
      // Record the time when target was first reached
      if (state_data.post_mpc_target_reached_time < 0.0) {
        state_data.post_mpc_target_reached_time = t_context;
        std::cout << "Post-MPC target " << state_data.post_mpc_target_idx
                  << " reached at t=" << state_data.post_mpc_target_reached_time
                  << std::endl;
      }

      // Check if we should advance to the next target
      if (ShouldAdvanceToNextOSCTarget(x_lcs_curr, t_context,
                                       state_data.post_mpc_target_idx,
                                       state_data.post_mpc_target_reached_time,
                                       osc_target_poses_post_mpc_)) {
        state_data.post_mpc_target_idx++;
        state_data.post_mpc_target_reached_time = -1.0;

        // Check if all post-MPC targets are completed
        if (state_data.post_mpc_target_idx >=
            static_cast<int>(osc_target_poses_post_mpc_.size())) {
          std::cout << "All post-MPC targets completed!" << std::endl;
          // Stay in post-MPC phase but at the last target
          state_data.post_mpc_target_idx =
              static_cast<int>(osc_target_poses_post_mpc_.size()) - 1;
        } else {
          std::cout << "Moving to post-MPC target "
                    << state_data.post_mpc_target_idx << std::endl;
        }
      }
    } else {
      // Not at target yet - reset target reached time if it was set
      if (state_data.post_mpc_target_reached_time >= 0.0) {
        state_data.post_mpc_target_reached_time = -1.0;
      }
    }
  }

  // Generate trajectory based on current phase
  switch (state_data.current_phase) {
    case AssemblyPhase::kMoveToTargetPreMPC:
      if (!osc_target_poses_pre_mpc_.empty() &&
          state_data.current_target_idx >= 0 &&
          state_data.current_target_idx <
              static_cast<int>(osc_target_poses_pre_mpc_.size())) {
        // Adjust z position of the last pre-MPC OSC target to align the
        // keypoints with the small pulley's groove
        if (state_data.current_target_idx ==
            static_cast<int>(osc_target_poses_pre_mpc_.size()) - 1) {
          osc_target_poses_pre_mpc_[state_data.current_target_idx].position[2] =
              target_ee_height_;
        }
        GenerateMoveToTargetTrajectory(
            x_lcs_curr, t_context, &execution_lcm_traj_, &planned_lcm_traj_,
            state_data.current_target_idx, osc_target_poses_pre_mpc_);
      }
      break;

    case AssemblyPhase::kMPC:
      if (!mpc_target_lcs_states_.empty() &&
          state_data.mpc_current_target_idx >= 0 &&
          state_data.mpc_current_target_idx <
              static_cast<int>(mpc_target_lcs_states_.size())) {
        const VectorXd& x_lcs_des =
            mpc_target_lcs_states_[state_data.mpc_current_target_idx];
        GenerateMPCTrajectory(x_lcs_curr, x_lcs_des, t_context,
                              &execution_lcm_traj_,
                              &planned_keypoints_lcm_traj_, &state_data);

        // Check if MPC reached its final target and transition to post-MPC
        if (mpc_reached_target_ &&
            state_data.mpc_current_target_idx >=
                static_cast<int>(mpc_target_lcs_states_.size()) - 1) {
          if (!osc_target_poses_post_mpc_.empty()) {
            std::cout << "MPC completed! Switching to post-MPC phase."
                      << std::endl;
            state_data.current_phase = AssemblyPhase::kMoveToTargetPostMPC;
            state_data.post_mpc_target_idx = 0;
            state_data.post_mpc_target_reached_time = -1.0;
          }
        }
      }
      break;

    case AssemblyPhase::kMoveToTargetPostMPC:
      if (!osc_target_poses_post_mpc_.empty() &&
          state_data.post_mpc_target_idx >= 0 &&
          state_data.post_mpc_target_idx <
              static_cast<int>(osc_target_poses_post_mpc_.size())) {
        GenerateMoveToTargetTrajectory(
            x_lcs_curr, t_context, &execution_lcm_traj_, &planned_lcm_traj_,
            state_data.post_mpc_target_idx, osc_target_poses_post_mpc_);
      }
      break;
  }

  // Update discrete state
  UpdateDiscreteState(state_data, discrete_state);

  return drake::systems::EventStatus::Succeeded();
}

bool AssemblyController::IsTargetReached(
    const Eigen::VectorXd& x_lcs_curr, int target_index,
    const std::vector<SingleOSCTargetPose>& target_poses) const {
  if (target_index < 0 ||
      target_index >= static_cast<int>(target_poses.size())) {
    return false;
  }

  const SingleOSCTargetPose& target_pose = target_poses[target_index];

  // Check position
  Eigen::Vector3d current_pos = x_lcs_curr.head(3);
  Eigen::Vector3d target_pos = target_pose.position;
  double distance = (current_pos - target_pos).norm();
  bool position_reached =
      distance <
      (target_pose.radius > 0.0
           ? round_belt_controller_params_.predefined_motion_position_tolerance
           : round_belt_controller_params_
                 .predefined_motion_position_tolerance);

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
  bool orientation_reached =
      angle <
      round_belt_controller_params_.predefined_motion_orientation_tolerance;

  return position_reached && orientation_reached;
}

bool AssemblyController::IsMpcTargetReached(
    const Eigen::VectorXd& x_lcs_curr, const Eigen::VectorXd& x_lcs_des) const {
  // Check position
  bool position_reached = (x_lcs_curr.head(3) - x_lcs_des.head(3)).norm() <
                          round_belt_controller_params_.mpc_position_tolerance;

  // Check orientation (RPY at indices 3, 4, 5)
  Eigen::Vector3d current_rpy = x_lcs_curr.segment(3, 3);
  Eigen::Vector3d target_rpy = x_lcs_des.segment(3, 3);
  RollPitchYaw<double> current_rpy_obj(current_rpy);
  RollPitchYaw<double> target_rpy_obj(target_rpy);
  Quaterniond current_quat = current_rpy_obj.ToQuaternion();
  Quaterniond target_quat = target_rpy_obj.ToQuaternion();
  current_quat.normalize();
  target_quat.normalize();

  // Compute angle between quaternions (taking shorter path)
  double dot_product = std::abs(current_quat.dot(target_quat));
  dot_product = std::min(1.0, std::max(-1.0, dot_product));
  double angle = 2.0 * std::acos(dot_product);
  bool orientation_reached =
      angle < round_belt_controller_params_.mpc_orientation_tolerance;

  return position_reached && orientation_reached;
}

void AssemblyController::AddEETrajectoriesToLcm(
    const Eigen::MatrixXd& position_knots,
    const Eigen::MatrixXd& orientation_knots,
    const Eigen::MatrixXd& force_knots, const Eigen::VectorXd& timestamps,
    LcmTrajectory* execution_traj, LcmTrajectory* planned_traj) const {
  execution_traj->ClearTrajectories();

  // Add end effector position trajectory
  LcmTrajectory::Trajectory ee_traj;
  ee_traj.traj_name = "end_effector_position_target";
  ee_traj.datatypes = std::vector<std::string>(position_knots.rows(), "double");
  ee_traj.datapoints = position_knots;
  ee_traj.time_vector = timestamps;
  execution_traj->AddTrajectory(ee_traj.traj_name, ee_traj);

  // Add end effector orientation trajectory
  LcmTrajectory::Trajectory ee_orientation_traj;
  ee_orientation_traj.traj_name = "end_effector_orientation_target";
  ee_orientation_traj.datatypes =
      std::vector<std::string>(orientation_knots.rows(), "double");
  ee_orientation_traj.datapoints = orientation_knots;
  ee_orientation_traj.time_vector = timestamps;
  execution_traj->AddTrajectory(ee_orientation_traj.traj_name,
                                ee_orientation_traj);

  // Add end effector force trajectory
  LcmTrajectory::Trajectory force_traj;
  force_traj.traj_name = "end_effector_force_target";
  force_traj.datatypes = std::vector<std::string>(force_knots.rows(), "double");
  force_traj.datapoints = force_knots;
  force_traj.time_vector = timestamps;
  execution_traj->AddTrajectory(force_traj.traj_name, force_traj);

  // Copy to planned trajectory for visualization
  planned_traj->ClearTrajectories();
  planned_traj->AddTrajectory(ee_traj.traj_name, ee_traj);
  planned_traj->AddTrajectory(ee_orientation_traj.traj_name,
                              ee_orientation_traj);
  planned_traj->AddTrajectory(force_traj.traj_name, force_traj);
}

void AssemblyController::GenerateMoveToTargetTrajectory(
    const Eigen::VectorXd& x_lcs_curr, double t_context,
    LcmTrajectory* execution_traj, LcmTrajectory* planned_traj,
    int target_index,
    const std::vector<SingleOSCTargetPose>& target_poses) const {
  if (target_index < 0 ||
      target_index >= static_cast<int>(target_poses.size())) {
    std::cerr << "Warning: Invalid target index: " << target_index << std::endl;
    return;
  }

  // Get target pose from target_poses vector
  const SingleOSCTargetPose& target_pose = target_poses[target_index];
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
  if (IsTargetReached(x_lcs_curr, target_index, target_poses)) {
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
                           execution_traj, planned_traj);
    return;
  }

  // Generate trajectory - handle both position and orientation interpolation
  int n_knots = N_;
  Eigen::MatrixXd knots = Eigen::MatrixXd::Zero(3, n_knots);
  Eigen::VectorXd timestamps = Eigen::VectorXd::Zero(n_knots);

  bool position_reached =
      distance <
      (target_pose.radius > 0.0
           ? round_belt_controller_params_.predefined_motion_position_tolerance
           : round_belt_controller_params_
                 .predefined_motion_position_tolerance);
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
                         execution_traj, planned_traj);
}

void AssemblyController::GenerateMPCTrajectory(
    const Eigen::VectorXd& x_lcs_curr, const Eigen::VectorXd& x_lcs_des,
    double t_context, LcmTrajectory* traj,
    LcmTrajectory* planned_keypoints_traj,
    DiscreteStateData* state_data) const {
  // Check if current MPC target is reached (both position and orientation)
  if (IsMpcTargetReached(x_lcs_curr, x_lcs_des)) {
    std::cout << "MPC reached target " << state_data->mpc_current_target_idx
              << std::endl;
    state_data->mpc_current_target_idx++;
    if (state_data->mpc_current_target_idx >=
        static_cast<int>(mpc_target_lcs_states_.size())) {
      std::cout << "All MPC targets completed!" << std::endl;
      mpc_reached_target_ = true;
      return;
    }
    std::cout << "Moving to MPC target " << state_data->mpc_current_target_idx
              << std::endl;
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

  try {
    c3_mpc_->Solve(x_lcs_curr, false);
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    is_solve_succeeded_ = false;
  }

  // Handle warm-up: check if we've completed enough C3 solve iterations
  if (!is_warmup_complete_) {
    mpc_warmup_count_++;
    if (mpc_warmup_count_ >=
        round_belt_controller_params_.mpc_warmup_iterations) {
      is_warmup_complete_ = true;
      std::cout << "C3+ warm-up complete after " << mpc_warmup_count_
                << " iterations" << std::endl;
    } else if (mpc_warmup_count_ % 100 == 0) {
      std::cout << "C3+ warm-up iteration " << mpc_warmup_count_ << " of "
                << round_belt_controller_params_.mpc_warmup_iterations
                << std::endl;
    }
  }

  // Get solution
  vector<VectorXd> u_sol = c3_mpc_->GetInputSolution();
  vector<VectorXd> x_sol = c3_mpc_->GetStateSolution();
  vector<VectorXd> lambda_sol = c3_mpc_->GetForceSolution();

  // Set up trajectory
  Eigen::MatrixXd knots = Eigen::MatrixXd::Zero(3, N_);
  Eigen::VectorXd timestamps = Eigen::VectorXd::Zero(N_);

  // During warm-up, hold last OSC pose; otherwise use C3 solution
  for (int i = 0; i < N_; i++) {
    if (!is_warmup_complete_ && warmup_pose_initialized_) {
      // Warm-up: hold last OSC target position
      knots.col(i) = warmup_hold_position_;
    } else if (is_solve_succeeded_) {
      knots.col(i) = x_sol[i].segment(0, 3);
    } else {
      knots.col(i) = x_desired[i].segment(0, 3);
    }
    timestamps[i] = t_context + i * dt_;
  }

  // End effector orientation (from RPY in x_sol)
  Eigen::MatrixXd ee_orientations = Eigen::MatrixXd::Zero(4, N_);
  Eigen::Vector3d rpy_curr = x_lcs_curr.segment(3, 3);
  Eigen::Vector3d rpy_desired = x_desired[0].segment(3, 3);
  Eigen::Vector3d step_angle = (rpy_desired - rpy_curr) / N_;

  for (int i = 0; i < N_; i++) {
    if (!is_warmup_complete_ && warmup_pose_initialized_) {
      // Warm-up: hold last OSC target orientation
      ee_orientations.col(i) = warmup_hold_orientation_;
    } else {
      Eigen::Vector3d rpy = rpy_curr + i * step_angle;
      RollPitchYaw<double> rpy_obj(rpy);
      Quaterniond quat = rpy_obj.ToQuaternion();
      quat.normalize();
      Eigen::Vector4d q_vec;
      q_vec << quat.w(), quat.x(), quat.y(), quat.z();
      ee_orientations.col(i) = q_vec;
    }
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
  // During warm-up, use zero force; otherwise use C3 solution
  Eigen::MatrixXd force_samples = Eigen::MatrixXd::Zero(3, N_);

  if (is_warmup_complete_ && !mpc_reached_target_ && is_solve_succeeded_) {
    for (int i = 0; i < N_; i++) {
      force_samples.col(i) = u_sol[i].segment(0, 3);
    }
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

  // Add planned trajectory (C3 solution) for visualization
  // This shows the actual C3 plan, regardless of warm-up state
  planned_lcm_traj_.ClearTrajectories();

  // Planned positions from C3 solution
  Eigen::MatrixXd planned_knots = Eigen::MatrixXd::Zero(3, N_);
  for (int i = 0; i < N_; i++) {
    if (is_solve_succeeded_) {
      planned_knots.col(i) = x_sol[i].segment(0, 3);
    } else {
      planned_knots.col(i) = x_desired[i].segment(0, 3);
    }
  }
  LcmTrajectory::Trajectory planned_ee_traj;
  planned_ee_traj.traj_name = "end_effector_position_target";
  planned_ee_traj.datatypes = std::vector<std::string>(3, "double");
  planned_ee_traj.datapoints = planned_knots;
  planned_ee_traj.time_vector = timestamps;
  planned_lcm_traj_.AddTrajectory(planned_ee_traj.traj_name, planned_ee_traj);

  // Planned orientations (interpolated from current to desired)
  Eigen::MatrixXd planned_orientations = Eigen::MatrixXd::Zero(4, N_);
  for (int i = 0; i < N_; i++) {
    Eigen::Vector3d rpy = rpy_curr + i * step_angle;
    RollPitchYaw<double> rpy_obj(rpy);
    Quaterniond quat = rpy_obj.ToQuaternion();
    quat.normalize();
    planned_orientations.col(i) =
        Eigen::Vector4d(quat.w(), quat.x(), quat.y(), quat.z());
  }
  LcmTrajectory::Trajectory planned_orientation_traj;
  planned_orientation_traj.traj_name = "end_effector_orientation_target";
  planned_orientation_traj.datatypes = std::vector<std::string>(4, "double");
  planned_orientation_traj.datapoints = planned_orientations;
  planned_orientation_traj.time_vector = timestamps;
  planned_lcm_traj_.AddTrajectory(planned_orientation_traj.traj_name,
                                  planned_orientation_traj);

  // Planned forces from C3 solution
  Eigen::MatrixXd planned_forces = Eigen::MatrixXd::Zero(3, N_);
  if (!mpc_reached_target_ && is_solve_succeeded_) {
    for (int i = 0; i < N_; i++) {
      planned_forces.col(i) = u_sol[i].segment(0, 3);
    }
  }
  LcmTrajectory::Trajectory planned_force_traj;
  planned_force_traj.traj_name = "end_effector_force_target";
  planned_force_traj.datatypes = std::vector<std::string>(3, "double");
  planned_force_traj.datapoints = planned_forces;
  planned_force_traj.time_vector = timestamps;
  planned_lcm_traj_.AddTrajectory(planned_force_traj.traj_name,
                                  planned_force_traj);

  // Convert C3 planned forces from contact frame to world frame
  ConvertForcesToWorldFrame(resolved_contact_pairs, lambda_sol[0]);
}

void AssemblyController::ConvertForcesToWorldFrame(
    const std::vector<drake::SortedPair<drake::geometry::GeometryId>>&
        resolved_contact_pairs,
    const Eigen::VectorXd& lcs_forces) const {
  int num_contacts = resolved_contact_pairs.size();
  int cur_lambda_index = 0;

  c3_forces_output_ = dairlib::lcmt_c3_forces();

  c3_forces_output_.num_forces =
      std::accumulate(
          assembly_c3_options_.num_friction_directions_per_contact.begin(),
          assembly_c3_options_.num_friction_directions_per_contact.end(), 0) *
      2;
  c3_forces_output_.forces.resize(c3_forces_output_.num_forces);

  DRAKE_DEMAND(contact_model_ == solvers::ContactModel::kAnitescu);

  for (int i = 0; i < num_contacts; i++) {
    multibody::GeomGeomCollider collider(plant_, resolved_contact_pairs[i]);
    int num_force_basis =
        2 * assembly_c3_options_.num_friction_directions_per_contact[i];
    bool is_planar_contact = num_force_basis == 2;
    auto [p_WCa, force_basis] =
        collider.CalcWitnessPointsAndForceBasisInWorldFrame(*context_,
                                                            is_planar_contact);
    // reduce force_basis to Anitescu force basis
    Eigen::Matrix<double, 4, 3> anitescu_force_basis;
    for (int j = 1; j < 5; j++) {
      anitescu_force_basis.row(j - 1) =
          force_basis.row(0) + assembly_c3_options_.mu[i] * force_basis.row(j);
    }
    for (int j = 0; j < 4; j++) {
      auto lcm_force = dairlib::lcmt_force();
      auto force = anitescu_force_basis.row(j) * lcs_forces[cur_lambda_index];
      lcm_force.contact_point[0] = p_WCa[0];
      lcm_force.contact_point[1] = p_WCa[1];
      lcm_force.contact_point[2] = p_WCa[2];
      lcm_force.contact_force[0] = force[0];
      lcm_force.contact_force[1] = force[1];
      lcm_force.contact_force[2] = force[2];
      c3_forces_output_.forces[i * 4 + j] = lcm_force;
      cur_lambda_index += 1;
    }
  }
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

void AssemblyController::OutputTrajPlanned(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* output) const {
  output->saved_traj = planned_lcm_traj_.GenerateLcmObject();
  output->utime = context.get_time() * 1e6;
}

void AssemblyController::OutputGripperPosCommand(
    const drake::systems::Context<double>& context,
    drake::lcmt_schunk_wsg_command* output) const {
  output->utime = context.get_time() * 1e6;
  output->target_position_mm = gripper_pos_command_ * 2000.0;
}

void AssemblyController::OutputC3Forces(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_c3_forces* output) const {
  *output = c3_forces_output_;
  output->utime = context.get_time() * 1e6;
}

void AssemblyController::OutputCurrentTargetLcsState(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output) const {
  // Get current MPC target index from discrete state
  int mpc_target_idx = static_cast<int>(
      context.get_discrete_state(mpc_current_target_index_).get_value()[0]);

  // Clamp index to valid range
  if (mpc_target_idx < 0) {
    mpc_target_idx = 0;
  } else if (mpc_target_idx >=
             static_cast<int>(mpc_target_lcs_states_.size())) {
    mpc_target_idx = static_cast<int>(mpc_target_lcs_states_.size()) - 1;
  }

  // Output the current target LCS state
  if (!mpc_target_lcs_states_.empty()) {
    output->set_value(mpc_target_lcs_states_[mpc_target_idx]);
  } else {
    output->set_value(VectorXd::Zero(n_x_));
  }
}

void AssemblyController::OutputC3Intermediates(
    const drake::systems::Context<double>& context,
    C3Output::C3Intermediates* c3_intermediates) const {
  double t = context.get_discrete_state(plan_start_time_index_)[0];
  auto z_sol = c3_mpc_->GetFullSolution();
  auto delta_sol = c3_mpc_->GetDualDeltaSolution();
  auto w_sol = c3_mpc_->GetDualWSolution();

  for (int i = 0; i < N_; i++) {
    c3_intermediates->time_vector_(i) = t + i * dt_;
    c3_intermediates->z_.col(i) = z_sol[i].cast<float>();
    c3_intermediates->w_.col(i) = w_sol[i].cast<float>();
    c3_intermediates->delta_.col(i) = delta_sol[i].cast<float>();
  }
}

void AssemblyController::OutputC3Solution(
    const drake::systems::Context<double>& context,
    C3Output::C3Solution* c3_solution) const {
  double t = context.get_discrete_state(plan_start_time_index_)[0];
  auto x_sol = c3_mpc_->GetStateSolution();
  auto lambda_sol = c3_mpc_->GetForceSolution();
  auto u_sol = c3_mpc_->GetInputSolution();

  for (int i = 0; i < N_; i++) {
    c3_solution->time_vector_(i) = t + i * dt_;
    c3_solution->x_sol_.col(i) = x_sol[i].cast<float>();
    c3_solution->lambda_sol_.col(i) = lambda_sol[i].cast<float>();
    c3_solution->u_sol_.col(i) = u_sol[i].cast<float>();
  }
}
}  // namespace magna
}  // namespace examples
}  // namespace dairlib
