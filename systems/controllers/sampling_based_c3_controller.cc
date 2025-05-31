#include "sampling_based_c3_controller.h"

#include <ctime>
#include <iostream>
#include <thread>
#include <utility>

#include <Eigen/Dense>
#include <omp.h>

#include "common/quaternion_error_hessian.h"
#include "dairlib/lcmt_radio_out.hpp"
#include "external/drake/tools/install/libdrake/_virtual_includes/drake_shared_library/drake/multibody/plant/multibody_plant.h"
#include "generate_samples.h"
#include "multibody/multibody_utils.h"
#include "solvers/c3_miqp.h"
#include "solvers/c3_options.h"
#include "solvers/c3_qp.h"
#include "solvers/lcs.h"

#include "drake/common/trajectories/piecewise_polynomial.h"

namespace dairlib {

using drake::SortedPair;
using drake::geometry::GeometryId;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteValues;
using drake::trajectories::PiecewisePolynomial;
using Eigen::MatrixXd;
using Eigen::MatrixXf;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::VectorXf;
using solvers::C3;
using solvers::C3MIQP;
using solvers::C3QP;
using solvers::LCS;
using solvers::LCSFactory;
using std::vector;
using systems::TimestampedVector;

namespace systems {

SamplingC3Controller::SamplingC3Controller(
    drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context,
    drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad,
    drake::systems::Context<drake::AutoDiffXd>* context_ad,
    const std::vector<
        std::vector<drake::SortedPair<drake::geometry::GeometryId>>>&
        contact_geoms,
    SamplingC3Options sampling_c3_options,
    SamplingC3SamplingParams sampling_params, bool verbose)
    : plant_(plant),
      context_(context),
      plant_ad_(plant_ad),
      context_ad_(context_ad),
      contact_pairs_(contact_geoms),
      sampling_c3_options_(std::move(sampling_c3_options)),
      sampling_params_(std::move(sampling_params)),
      G_(std::vector<MatrixXd>(sampling_c3_options_.N,
                               sampling_c3_options_.G_position_tracking)),
      G_for_curr_location_(std::vector<MatrixXd>(
          sampling_c3_options_.N,
          sampling_c3_options_.G_position_tracking_for_curr_location)),
      U_(std::vector<MatrixXd>(sampling_c3_options_.N,
                               sampling_c3_options_.U_position_tracking)),
      U_for_curr_location_(std::vector<MatrixXd>(
          sampling_c3_options_.N,
          sampling_c3_options_.U_position_tracking_for_curr_location)),
      N_(sampling_c3_options_.N),
      verbose_(verbose) {
  this->set_name("sampling_c3_controller");

  double discount_factor = 1;
  // Build C3Options from SamplingC3Options.
  // crossed_cost_switching_threshold_ is false by default, so these are
  // position tracking options to begin with.
  C3Options c3_options = sampling_c3_options_.GetC3Options(
      crossed_cost_switching_threshold_,
      sampling_c3_options_.num_contacts_index);
  C3Options c3_options_for_curr_location = sampling_c3_options_.GetC3Options(
      crossed_cost_switching_threshold_,
      sampling_c3_options_.num_contacts_index_for_curr_location);

  for (int i = 0; i < N_; ++i) {
    Q_.push_back(discount_factor * c3_options.Q);
    R_.push_back(discount_factor * c3_options.R);
    discount_factor *= c3_options.gamma;
  }
  Q_.push_back(discount_factor * c3_options.Q);
  DRAKE_DEMAND(Q_.size() == N_ + 1);
  DRAKE_DEMAND(R_.size() == N_);

  n_q_ = plant_.num_positions();
  n_v_ = plant_.num_velocities();
  n_u_ = plant_.num_actuators();

  n_x_ = n_q_ + n_v_;

  solve_time_filter_constant_ = sampling_c3_options_.solve_time_filter_alpha;
  if (sampling_c3_options_.contact_model == "stewart_and_trinkle") {
    contact_model_ = solvers::ContactModel::kStewartAndTrinkle;
    n_lambda_ =
        2 * sampling_c3_options_
                .num_contacts_list[sampling_c3_options_.num_contacts_index] +
        2 * sampling_c3_options_.num_friction_directions *
            sampling_c3_options_
                .num_contacts_list[sampling_c3_options_.num_contacts_index];
  } else if (sampling_c3_options_.contact_model == "anitescu") {
    contact_model_ = solvers::ContactModel::kAnitescu;
    n_lambda_ = 2 * sampling_c3_options_.num_friction_directions *
                sampling_c3_options_
                    .num_contacts_list[sampling_c3_options_.num_contacts_index];
  } else {
    std::cerr << ("Unknown or unsupported contact model") << std::endl;
    DRAKE_THROW_UNLESS(false);
  }

  // Creates placeholder lcs to construct base C3 problem
  // Placeholder LCS will have correct size as it's already determined by the
  // contact model
  auto lcs_placeholder = CreatePlaceholderLCS();
  auto x_desired_placeholder =
      std::vector<VectorXd>(N_ + 1, VectorXd::Zero(n_x_));
  if (sampling_c3_options_.projection_type == "MIQP") {
    c3_curr_plan_ = std::make_unique<C3MIQP>(
        lcs_placeholder, C3::CostMatrices(Q_, R_, G_, U_),
        x_desired_placeholder, c3_options_for_curr_location);
    c3_best_plan_ = std::make_unique<C3MIQP>(lcs_placeholder,
                                             C3::CostMatrices(Q_, R_, G_, U_),
                                             x_desired_placeholder, c3_options);
    c3_buffer_plan_ = std::make_unique<C3MIQP>(
        lcs_placeholder, C3::CostMatrices(Q_, R_, G_, U_),
        x_desired_placeholder, c3_options);
  } else if (sampling_c3_options_.projection_type == "QP") {
    c3_curr_plan_ = std::make_unique<C3QP>(
        lcs_placeholder, C3::CostMatrices(Q_, R_, G_, U_),
        x_desired_placeholder, c3_options_for_curr_location);
    c3_best_plan_ = std::make_unique<C3QP>(lcs_placeholder,
                                           C3::CostMatrices(Q_, R_, G_, U_),
                                           x_desired_placeholder, c3_options);
    c3_buffer_plan_ = std::make_unique<C3QP>(lcs_placeholder,
                                             C3::CostMatrices(Q_, R_, G_, U_),
                                             x_desired_placeholder, c3_options);
  } else {
    std::cerr << ("Unknown projection type") << std::endl;
    DRAKE_THROW_UNLESS(false);
  }

  c3_curr_plan_->SetOsqpSolverOptions(solver_options_);
  c3_best_plan_->SetOsqpSolverOptions(solver_options_);
  c3_buffer_plan_->SetOsqpSolverOptions(solver_options_);

  // Set actor bounds
  for (int i = 0; i < sampling_c3_options_.workspace_limits.size(); ++i) {
    Eigen::RowVectorXd A = VectorXd::Zero(n_x_);
    A.segment(0, 3) = sampling_c3_options_.workspace_limits[i].segment(0, 3);
    // TODO @sharanyashastry: For the T example, the z constraint is an equality
    // constraint. This will be reflected in the params but need to make sure to
    // put a comment here when the T example is added.
    // The fourth parameter decides which optimization variable the constraint
    // is applied to. 1 = x, 2 = u, 3 = lambda.

    // These are hard constraints set in the C3 problem for the end effector
    // position.
    c3_curr_plan_->AddLinearConstraint(
        A, c3_options_for_curr_location.workspace_limits[i][3],
        c3_options_for_curr_location.workspace_limits[i][4], 1);
    c3_best_plan_->AddLinearConstraint(A, c3_options.workspace_limits[i][3],
                                       c3_options.workspace_limits[i][4], 1);
    c3_buffer_plan_->AddLinearConstraint(A, c3_options.workspace_limits[i][3],
                                         c3_options.workspace_limits[i][4], 1);
  }
  for (int i : vector<int>({0, 1})) {
    Eigen::RowVectorXd A = VectorXd::Zero(n_u_);
    A(i) = 1.0;
    c3_curr_plan_->AddLinearConstraint(
        A, c3_options_for_curr_location.u_horizontal_limits[0],
        c3_options_for_curr_location.u_horizontal_limits[1], 2);
    c3_best_plan_->AddLinearConstraint(A, c3_options.u_horizontal_limits[0],
                                       c3_options.u_horizontal_limits[1], 2);
    c3_buffer_plan_->AddLinearConstraint(A, c3_options.u_horizontal_limits[0],
                                         c3_options.u_horizontal_limits[1], 2);
  }
  for (int i : vector<int>({2})) {
    Eigen::RowVectorXd A = VectorXd::Zero(n_u_);
    A(i) = 1.0;
    c3_curr_plan_->AddLinearConstraint(
        A, c3_options_for_curr_location.u_vertical_limits[0],
        c3_options_for_curr_location.u_vertical_limits[1], 2);
    c3_best_plan_->AddLinearConstraint(A, c3_options.u_vertical_limits[0],
                                       c3_options.u_vertical_limits[1], 2);
    c3_buffer_plan_->AddLinearConstraint(A, c3_options.u_vertical_limits[0],
                                         c3_options.u_vertical_limits[1], 2);
  }

  // Input ports.
  radio_port_ =
      this->DeclareAbstractInputPort("lcmt_radio_out",
                                     drake::Value<dairlib::lcmt_radio_out>{})
          .get_index();
  lcs_state_input_port_ =
      this->DeclareVectorInputPort("x_lcs", TimestampedVector<double>(n_x_))
          .get_index();
  target_input_port_ =
      this->DeclareVectorInputPort("x_lcs_des", n_x_).get_index();
  final_target_input_port_ =
      this->DeclareVectorInputPort("x_lcs_final_des", n_x_).get_index();

  // Output ports.
  auto c3_solution = C3Output::C3Solution();
  c3_solution.x_sol_ = MatrixXf::Zero(n_q_ + n_v_, N_);
  c3_solution.lambda_sol_ = MatrixXf::Zero(n_lambda_, N_);
  c3_solution.u_sol_ = MatrixXf::Zero(n_u_, N_);
  c3_solution.time_vector_ = VectorXf::Zero(N_);
  auto c3_intermediates = C3Output::C3Intermediates();
  c3_intermediates.z_ = MatrixXf::Zero(n_x_ + n_lambda_ + n_u_, N_);
  c3_intermediates.w_ = MatrixXf::Zero(n_x_ + n_lambda_ + n_u_, N_);
  c3_intermediates.delta_ = MatrixXf::Zero(n_x_ + n_lambda_ + n_u_, N_);
  c3_intermediates.time_vector_ = VectorXf::Zero(N_);
  auto lcs_contact_jacobian = std::pair(Eigen::MatrixXd(n_x_, n_lambda_),
                                        std::vector<Eigen::VectorXd>());

  // Since the num_additional_samples_repos means the additional samples
  // to generate in addition to the prev_repositioning_target_, add 1.
  // Additionally add 1 to C3 if considering samples from the buffer.
  int from_buffer = 0;
  if (sampling_params_.consider_best_buffer_sample_when_leaving_c3) {
    from_buffer = 1;
  }
  max_num_samples_ =
      std::max(sampling_params_.num_additional_samples_repos + 1,
               sampling_params_.num_additional_samples_c3 + from_buffer);
  // The +1 here is to account for the current location.
  all_sample_locations_ =
      vector<Vector3d>(max_num_samples_ + 1, Vector3d::Zero());
  LcmTrajectory lcm_traj = LcmTrajectory();

  // Current location plan output ports.
  // This output port is being kept so it can go into a C3outputSender which is
  // what we use to grab downstream forces for visualization.
  c3_solution_curr_plan_port_ =
      this->DeclareAbstractOutputPort(
              "c3_solution_curr_plan", c3_solution,
              &SamplingC3Controller::OutputC3SolutionCurrPlan)
          .get_index();
  c3_solution_curr_plan_actor_port_ =
      this->DeclareAbstractOutputPort(
              "c3_solution_curr_plan_actor",
              dairlib::lcmt_timestamped_saved_traj(),
              &SamplingC3Controller::OutputC3SolutionCurrPlanActor)
          .get_index();
  c3_solution_curr_plan_object_port_ =
      this->DeclareAbstractOutputPort(
              "c3_solution_curr_plan_object",
              dairlib::lcmt_timestamped_saved_traj(),
              &SamplingC3Controller::OutputC3SolutionCurrPlanObject)
          .get_index();
  c3_intermediates_curr_plan_port_ =
      this->DeclareAbstractOutputPort(
              "c3_intermediates_curr_plan", c3_intermediates,
              &SamplingC3Controller::OutputC3IntermediatesCurrPlan)
          .get_index();
  lcs_contact_jacobian_curr_plan_port_ =
      this->DeclareAbstractOutputPort(
              "J_lcs_curr_plan, p_lcs_curr_plan", lcs_contact_jacobian,
              &SamplingC3Controller::OutputLCSContactJacobianCurrPlan)
          .get_index();

  // Best sample plan output ports.
  // This output port is being kept so it can go into a C3outputSender which is
  // what we use to grab downstream forces for visualization.
  c3_solution_best_plan_port_ =
      this->DeclareAbstractOutputPort(
              "c3_solution_best_plan", c3_solution,
              &SamplingC3Controller::OutputC3SolutionBestPlan)
          .get_index();
  c3_solution_best_plan_actor_port_ =
      this->DeclareAbstractOutputPort(
              "c3_solution_best_plan_actor",
              dairlib::lcmt_timestamped_saved_traj(),
              &SamplingC3Controller::OutputC3SolutionBestPlanActor)
          .get_index();
  c3_solution_best_plan_object_port_ =
      this->DeclareAbstractOutputPort(
              "c3_solution_best_plan_object",
              dairlib::lcmt_timestamped_saved_traj(),
              &SamplingC3Controller::OutputC3SolutionBestPlanObject)
          .get_index();
  c3_intermediates_best_plan_port_ =
      this->DeclareAbstractOutputPort(
              "c3_intermediates_best_plan", c3_intermediates,
              &SamplingC3Controller::OutputC3IntermediatesBestPlan)
          .get_index();
  lcs_contact_jacobian_best_plan_port_ =
      this->DeclareAbstractOutputPort(
              "J_lcs_best_plan, p_lcs_best_plan", lcs_contact_jacobian,
              &SamplingC3Controller::OutputLCSContactJacobianBestPlan)
          .get_index();

  // Execution trajectory output ports.
  c3_traj_execute_actor_port_ =
      this->DeclareAbstractOutputPort(
              "c3_traj_execute_actor", dairlib::lcmt_timestamped_saved_traj(),
              &SamplingC3Controller::OutputC3TrajExecuteActor)
          .get_index();
  c3_traj_execute_object_port_ =
      this->DeclareAbstractOutputPort(
              "c3_traj_execute_object", dairlib::lcmt_timestamped_saved_traj(),
              &SamplingC3Controller::OutputC3TrajExecuteObject)
          .get_index();
  repos_traj_execute_actor_port_ =
      this->DeclareAbstractOutputPort(
              "repos_traj_execute_actor",
              dairlib::lcmt_timestamped_saved_traj(),
              &SamplingC3Controller::OutputReposTrajExecuteActor)
          .get_index();
  repos_traj_execute_object_port_ =
      this->DeclareAbstractOutputPort(
              "repos_traj_execute_object",
              dairlib::lcmt_timestamped_saved_traj(),
              &SamplingC3Controller::OutputReposTrajExecuteObject)
          .get_index();
  // NOTE: We reuse lcm_traj to set the right type for the port.
  traj_execute_actor_port_ =
      this->DeclareAbstractOutputPort(
              "traj_execute_actor", dairlib::lcmt_timestamped_saved_traj(),
              &SamplingC3Controller::OutputTrajExecuteActor)
          .get_index();
  traj_execute_object_port_ =
      this->DeclareAbstractOutputPort(
              "traj_execute_object", dairlib::lcmt_timestamped_saved_traj(),
              &SamplingC3Controller::OutputTrajExecuteObject)
          .get_index();
  is_c3_mode_port_ =
      this->DeclareAbstractOutputPort("is_c3_mode",
                                      dairlib::lcmt_timestamped_saved_traj(),
                                      &SamplingC3Controller::OutputIsC3Mode)
          .get_index();

  // Output ports for dynamically feasible plans used for cost computation and
  // visualization.
  dynamically_feasible_curr_plan_actor_port_ =
      this->DeclareAbstractOutputPort(
              "dynamically_feasible_curr_plan_actor",
              dairlib::lcmt_timestamped_saved_traj(),
              &SamplingC3Controller::OutputDynamicallyFeasibleCurrPlanActor)
          .get_index();
  dynamically_feasible_curr_plan_object_port_ =
      this->DeclareAbstractOutputPort(
              "dynamically_feasible_curr_plan_object",
              dairlib::lcmt_timestamped_saved_traj(),
              &SamplingC3Controller::OutputDynamicallyFeasibleCurrPlanObject)
          .get_index();
  dynamically_feasible_best_plan_actor_port_ =
      this->DeclareAbstractOutputPort(
              "dynamically_feasible_best_plan_actor",
              dairlib::lcmt_timestamped_saved_traj(),
              &SamplingC3Controller::OutputDynamicallyFeasibleBestPlanActor)
          .get_index();
  dynamically_feasible_best_plan_object_port_ =
      this->DeclareAbstractOutputPort(
              "dynamically_feasible_best_plan_object",
              dairlib::lcmt_timestamped_saved_traj(),
              &SamplingC3Controller::OutputDynamicallyFeasibleBestPlanObject)
          .get_index();

  // Sample location related output ports.
  // This port will output all samples except the current location.
  // all_sample_locations_port_ does not include the current location. So
  // index 0 is the first sample.
  all_sample_locations_port_ =
      this->DeclareAbstractOutputPort(
              "all_sample_locations", dairlib::lcmt_timestamped_saved_traj(),
              &SamplingC3Controller::OutputAllSampleLocations)
          .get_index();
  // all_sample_costs_port_ does include the current location. So index 0 is
  // the current location cost.
  all_sample_costs_port_ =
      this->DeclareAbstractOutputPort(
              "all_sample_costs", dairlib::lcmt_timestamped_saved_traj(),
              &SamplingC3Controller::OutputAllSampleCosts)
          .get_index();

  curr_and_best_sample_costs_port_ =
      this->DeclareAbstractOutputPort(
              "current_and_best_sample_cost",
              dairlib::lcmt_timestamped_saved_traj(),
              &SamplingC3Controller::OutputCurrAndBestSampleCost)
          .get_index();

  // A debug output port to publish information about the internals of the
  // sampling-based controller.
  debug_lcmt_port_ =
      this->DeclareAbstractOutputPort("sampling_c3_debug",
                                      dairlib::lcmt_sampling_c3_debug(),
                                      &SamplingC3Controller::OutputDebug)
          .get_index();

  // Sample buffer related ouput ports.
  sample_buffer_ = MatrixXd::Zero(sampling_params_.N_sample_buffer, n_q_);
  sample_costs_buffer_ = -1 * VectorXd::Ones(sampling_params_.N_sample_buffer);
  sample_buffer_configurations_port_ =
      this->DeclareAbstractOutputPort(
              "sample_buffer_configurations", sample_buffer_,
              &SamplingC3Controller::OutputSampleBufferConfigurations)
          .get_index();

  sample_buffer_costs_port_ =
      this->DeclareAbstractOutputPort(
              "sample_buffer_costs", sample_costs_buffer_,
              &SamplingC3Controller::OutputSampleBufferCosts)
          .get_index();

  plan_start_time_index_ = DeclareDiscreteState(1);
  x_pred_curr_plan_ = VectorXd::Zero(n_x_);
  x_from_last_control_loop_ = VectorXd::Zero(n_x_);
  x_pred_from_last_control_loop_ = VectorXd::Zero(n_x_);
  x_final_target_ = VectorXd::Zero(n_x_);
  best_progress_steps_ago_ = 0;
  while (!progress_cost_buffer_.empty()) {
    progress_cost_buffer_.pop();
  }
  lowest_cost_ = -1.0;
  lowest_pos_and_rot_current_cost_ = -1.0;
  lowest_position_error_ = -1.0;
  lowest_orientation_error_ = -1.0;

  DeclareForcedDiscreteUpdateEvent(&SamplingC3Controller::ComputePlan);

  // Set parallelization settings.
  omp_set_dynamic(0);  // Explicitly disable dynamic teams.
  omp_set_nested(1);   // Enable nested threading.
  if (sampling_c3_options_.num_outer_threads == 0) {
    // Interpret setting number of threads to zero as a request to use all
    // machine's threads.
    num_threads_to_use_ = omp_get_max_threads();
  } else {
    num_threads_to_use_ = sampling_c3_options_.num_outer_threads;
  }

  if (verbose_) {
    std::cout << "Initial filtered_solve_time_: " << filtered_solve_time_
              << std::endl;
  }
}

LCS SamplingC3Controller::CreatePlaceholderLCS() const {
  MatrixXd A = MatrixXd::Ones(n_x_, n_x_);
  MatrixXd B = MatrixXd::Zero(n_x_, n_u_);
  VectorXd d = VectorXd::Zero(n_x_);
  MatrixXd D = MatrixXd::Ones(n_x_, n_lambda_);
  MatrixXd E = MatrixXd::Zero(n_lambda_, n_x_);
  MatrixXd F = MatrixXd::Zero(n_lambda_, n_lambda_);
  MatrixXd H = MatrixXd::Zero(n_lambda_, n_u_);
  VectorXd c = VectorXd::Zero(n_lambda_);
  return LCS(A, B, D, d, E, F, H, c, sampling_c3_options_.N,
             sampling_c3_options_.planning_dt_position_tracking);
}

drake::systems::EventStatus SamplingC3Controller::ComputePlan(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  auto start = std::chrono::high_resolution_clock::now();

  // Evaluate input ports.
  const auto& radio_out =
      this->EvalInputValue<dairlib::lcmt_radio_out>(context, radio_port_);
  // Not sure why x_lcs_des is a vector while lcs_x_curr is a timestamped
  // vector.
  const BasicVector<double>& x_lcs_des =
      *this->template EvalVectorInput<BasicVector>(context, target_input_port_);
  const BasicVector<double>& x_lcs_final_des =
      *this->template EvalVectorInput<BasicVector>(context,
                                                   final_target_input_port_);
  const TimestampedVector<double>* lcs_x_curr =
      (TimestampedVector<double>*)this->EvalVectorInput(context,
                                                        lcs_state_input_port_);

  // Store the current LCS state.
  drake::VectorX<double> x_lcs_curr = lcs_x_curr->get_data();

  if (verbose_) {
    std::cout << "x_lcs_curr: " << x_lcs_curr.transpose() << std::endl;
    std::cout << "x_lcs_des: " << x_lcs_des.get_value().transpose()
              << std::endl;
    std::cout << "x_lcs_final_des: " << x_lcs_final_des.get_value().transpose()
              << std::endl;
    std::cout << "x_pred_curr_plan_: " << x_pred_curr_plan_.transpose()
              << std::endl;
  }

  // If not tele-opping and if generating samples about a predicted lcs state,
  // clamp the predicted next state to be not too far away from the current
  // state.
  // Only use predicted state for c3 if use_predicted_x0_c3 is true and
  // is_doing_c3_. Only use predicted state for repositioning if
  // use_predicted_x0_repos is true and !is_doing_c3_.
  Eigen::Vector3d curr_ee = x_lcs_curr.head(3);
  Eigen::Vector3d last_ee = x_from_last_control_loop_.head(3);
  Eigen::Vector3d pred_ee = x_pred_from_last_control_loop_.head(3);
  // Store the current actual state before applying prediction in preparation
  // for next control loop.
  x_from_last_control_loop_ = x_lcs_curr;
  if (!radio_out->channel[14] && !x_pred_curr_plan_.isZero() &&
      sampling_c3_options_.use_predicted_x0_c3 && is_doing_c3_) {
    // First detect if we should use predicted state or not:  x_pred reset
    // mechanism is used if the prediction from last control loop is further
    // from the current state than last state was.
    if (((curr_ee - last_ee).norm() < (curr_ee - pred_ee).norm()) &&
        (curr_ee - pred_ee).norm() > 0.01 &&
        !x_pred_from_last_control_loop_.isZero() &&
        sampling_c3_options_.use_predicted_x0_reset_mechanism) {
      // Skip using the predicted state.
      if (verbose_) {
        std::cout << "RESET x_pred in C3 mode. ";
        std::cout << "curr_ee-last_ee is " << (curr_ee - last_ee).norm()
                  << " and curr_ee-pred_ee is " << (curr_ee - pred_ee).norm()
                  << std::endl;
        std::cout << "x_lcs_curr without clamping: " << x_lcs_curr.transpose()
                  << std::endl;
      }
    } else {
      // Do the clamping.
      ClampEndEffectorAcceleration(x_lcs_curr);
      if (verbose_) {
        std::cout << "x_lcs_curr after clamping in C3 mode: "
                  << x_lcs_curr.transpose() << std::endl;
      }
    }
  } else if (!radio_out->channel[14] && !x_pred_curr_plan_.isZero() &&
             sampling_c3_options_.use_predicted_x0_repos && !is_doing_c3_) {
    // First detect if we should use predicted state or not:  x_pred reset
    // mechanism is used if the prediction from last control loop is further
    // from the current state than last state was.
    if (((curr_ee - last_ee).norm() < (curr_ee - pred_ee).norm()) &&
        (curr_ee - pred_ee).norm() > 0.01 &&
        !x_pred_from_last_control_loop_.isZero() &&
        sampling_c3_options_.use_predicted_x0_reset_mechanism) {
      // Skip using the predicted state.
      if (verbose_) {
        std::cout << "RESET x_pred in repositioning mode. ";
        std::cout << "curr_ee-last_ee is " << (curr_ee - last_ee).norm()
                  << " and curr_ee-pred_ee is " << (curr_ee - pred_ee).norm()
                  << std::endl;
        std::cout << "x_lcs_curr without clamping: " << x_lcs_curr.transpose()
                  << std::endl;
      }
    } else {
      // Do the clamping.
      ClampEndEffectorAcceleration(x_lcs_curr);
      if (verbose_) {
        std::cout << "x_lcs_curr after clamping in repositioning mode: "
                  << x_lcs_curr.transpose() << std::endl;
      }
    }
  } else if (verbose_) {
    std::cout << "Not clamping x_lcs_curr." << std::endl;
  }
  // Store the predicted actual state in preparation for next control loop.
  x_pred_from_last_control_loop_ = x_lcs_curr;

  discrete_state->get_mutable_value(plan_start_time_index_)[0] =
      lcs_x_curr->get_timestamp();

  std::vector<VectorXd> x_desired =
      std::vector<VectorXd>(N_ + 1, x_lcs_des.value());

  // Force Checking of Workspace Limits
  // These are soft safety constraints checked within our controller
  // to ensure the robot is within the workspace limits.
  for (int i = 0; i < sampling_c3_options_.workspace_limits.size(); ++i) {
    DRAKE_DEMAND(lcs_x_curr->get_data().segment(0, 3).transpose() *
                     sampling_c3_options_.workspace_limits[i].segment(0, 3) >
                 sampling_c3_options_.workspace_limits[i][3] -
                     sampling_c3_options_.workspace_margins);
    DRAKE_DEMAND(lcs_x_curr->get_data().segment(0, 3).transpose() *
                     sampling_c3_options_.workspace_limits[i].segment(0, 3) <
                 sampling_c3_options_.workspace_limits[i][4] +
                     sampling_c3_options_.workspace_margins);
  }
  DRAKE_DEMAND(std::pow(lcs_x_curr->get_data()[0], 2) +
                   std::pow(lcs_x_curr->get_data()[1], 2) >
               std::pow(sampling_c3_options_.robot_radius_limits[0] +
                            sampling_c3_options_.workspace_margins,
                        2));
  DRAKE_DEMAND(std::pow(lcs_x_curr->get_data()[0], 2) +
                   std::pow(lcs_x_curr->get_data()[1], 2) <
               std::pow(sampling_c3_options_.robot_radius_limits[1] -
                            sampling_c3_options_.workspace_margins,
                        2));

  // Compute the current position and orientation errors.
  current_position_error_ =
      (x_lcs_curr.segment(7, 3) - x_lcs_final_des.get_value().segment(7, 3))
          .norm();
  Eigen::Quaterniond curr_quat(x_lcs_curr[3], x_lcs_curr[4], x_lcs_curr[5],
                               x_lcs_curr[6]);
  Eigen::Quaterniond des_quat(
      x_lcs_final_des.get_value()[3], x_lcs_final_des.get_value()[4],
      x_lcs_final_des.get_value()[5], x_lcs_final_des.get_value()[6]);
  Eigen::AngleAxis<double> angle_axis_diff(des_quat * curr_quat.inverse());
  current_orientation_error_ = angle_axis_diff.angle();

  // Detect if the final target has changed, in which case return to caring only
  // about position until the switching threshold has been crossed again.
  // Exclude the EE goal from the comparison, since that always changes to be
  // above the current jack location.
  if (!x_final_target_.segment(3, n_x_ - 3)
           .isApprox(x_lcs_final_des.value().segment(3, n_x_ - 3), 1e-5)) {
    std::cout << "Detected goal change!" << std::endl;
    if (verbose_) {
      std::cout << "  Last goal: " << x_final_target_.transpose() << std::endl;
      std::cout << "  New goal:  " << x_lcs_final_des.value().transpose()
                << std::endl;
      std::cout << "  --> Error:  "
                << (x_final_target_.segment(3, n_x_ - 3) -
                    x_lcs_final_des.value().segment(3, n_x_ - 3))
                       .norm()
                << std::endl;
    }
    crossed_cost_switching_threshold_ = false;
    x_final_target_ = x_lcs_final_des.value();
    detected_goal_changes_++;

    // Reset the sample buffer now that the costs have changed.
    sample_buffer_ = MatrixXd::Zero(sampling_params_.N_sample_buffer, n_q_);
    sample_costs_buffer_ =
        -1 * VectorXd::Ones(sampling_params_.N_sample_buffer);
  }

  // If object is within a fixed radius of the desired location, enter pose
  // tracking mode.
  if (!crossed_cost_switching_threshold_) {
    if ((x_lcs_curr.segment(7, 2) - x_lcs_final_des.value().segment(7, 2))
            .norm() < sampling_params_.cost_switching_threshold_distance) {
      crossed_cost_switching_threshold_ = true;
      std::cout << "Crossed cost switching threshold." << std::endl;

      // Reset the sample buffer now that the costs have changed.
      sample_buffer_ = MatrixXd::Zero(sampling_params_.N_sample_buffer, n_q_);
      sample_costs_buffer_ =
          -1 * VectorXd::Ones(sampling_params_.N_sample_buffer);

      // If also in C3 mode, reset the lowest cost seen in this mode.
      if (is_doing_c3_) {
        lowest_cost_ = -1.0;
        lowest_pos_and_rot_current_cost_ = -1.0;
        lowest_position_error_ = -1.0;
        lowest_orientation_error_ = -1.0;
        best_progress_steps_ago_ = 0;
        // Clear the progress cost buffer.
        while (!progress_cost_buffer_.empty()) {
          progress_cost_buffer_.pop();
        }
      }
    }
  }

  // Cost switching based on threshold to start using pose based cost.
  Q_.clear();
  G_.clear();
  U_.clear();
  G_for_curr_location_.clear();
  U_for_curr_location_.clear();
  double discount_factor = 1;

  // Build C3Options from SamplingC3Options.
  C3Options c3_options;
  C3Options c3_options_for_curr_location;

  // Load the pose/position tracking C3Options based on the
  // crossed_cost_switching_threshold_ flag.
  if (crossed_cost_switching_threshold_) {
    c3_options = sampling_c3_options_.GetC3Options(
        crossed_cost_switching_threshold_,
        sampling_c3_options_.num_contacts_index);
    c3_options_for_curr_location = sampling_c3_options_.GetC3Options(
        crossed_cost_switching_threshold_,
        sampling_c3_options_.num_contacts_index_for_curr_location);

    dt_ = c3_options.dt;
    // clear the Q_ values and replace with costs for position and orientation.
    for (int i = 0; i < N_ + 1; ++i) {
      Q_.push_back(discount_factor * c3_options.Q);
      discount_factor *= c3_options.gamma;
      if (i < N_) {
        R_.push_back(discount_factor * c3_options.R);
        G_.push_back(c3_options.G);
        U_.push_back(c3_options.U);
        G_for_curr_location_.push_back(c3_options_for_curr_location.G);
        U_for_curr_location_.push_back(c3_options_for_curr_location.U);
      }
    }
  } else {
    c3_options = sampling_c3_options_.GetC3Options(
        crossed_cost_switching_threshold_,
        sampling_c3_options_.num_contacts_index);
    c3_options_for_curr_location = sampling_c3_options_.GetC3Options(
        crossed_cost_switching_threshold_,
        sampling_c3_options_.num_contacts_index_for_curr_location);
    dt_ = c3_options.dt;
    // clear the Q_ values and replace with costs for position only.
    for (int i = 0; i < N_ + 1; ++i) {
      Q_.push_back(discount_factor * c3_options.Q);
      discount_factor *= c3_options.gamma;
      if (i < N_) {
        R_.push_back(discount_factor * c3_options.R);
        G_.push_back(c3_options.G);
        U_.push_back(c3_options.U);
        G_for_curr_location_.push_back(c3_options_for_curr_location.G);
        U_for_curr_location_.push_back(c3_options_for_curr_location.U);
      }
    }
  }

  if (sampling_c3_options_.use_quaternion_dependent_cost &&
      crossed_cost_switching_threshold_) {
    Eigen::VectorXd quat = x_lcs_curr.segment(3, 4);
    Eigen::VectorXd quat_desired = x_lcs_des.get_value().segment(3, 4);
    Eigen::MatrixXd Q_quaternion_dependent_cost =
        hessian_of_squared_quaternion_angle_difference(quat, quat_desired);

    // Get the eigenvalues of the hessian to regularize so the Q matrix is
    // always PSD.
    double min_eigval =
        Q_quaternion_dependent_cost.eigenvalues().real().minCoeff();
    Eigen::MatrixXd Q_quaternion_dependent_regularizer_part_1 =
        std::max(0.0, -min_eigval) * Eigen::MatrixXd::Identity(4, 4);

    Eigen::MatrixXd Q_quaternion_dependent_regularizer_part_2 =
        quat_desired * quat_desired.transpose();
    DRAKE_ASSERT(Q_quaternion_dependent_cost.rows() ==
                 Q_quaternion_dependent_cost.cols() ==
                 Q_quaternion_dependent_regularizer_part_2.rows() ==
                 Q_quaternion_dependent_regularizer_part_2.cols() == 4);
    double discount_factor = 1;
    for (int i = 0; i < N_ + 1; ++i) {
      Q_[i].block(3, 3, 4, 4) =
          discount_factor * sampling_c3_options_.q_quaternion_dependent_weight *
          (Q_quaternion_dependent_cost +
           Q_quaternion_dependent_regularizer_part_1 +
           sampling_c3_options_.q_quaternion_dependent_regularizer_fraction *
               Q_quaternion_dependent_regularizer_part_2);
      discount_factor *= sampling_c3_options_.gamma;
    }
  }

  if (verbose_) {
    std::cout << "Q_[0] with gamma " << sampling_c3_options_.gamma << ":"
              << std::endl;
    std::cout << Q_[0] << std::endl;
    std::cout << "R_[0] with gamma " << sampling_c3_options_.gamma << ":"
              << std::endl;
    std::cout << R_[0] << std::endl;
  }

  // Generate multiple samples and include current location as first item.
  std::vector<Eigen::VectorXd> candidate_states =
      generate_sample_states(n_q_, n_v_, n_u_, x_lcs_curr, is_doing_c3_,
                             sampling_params_, sampling_c3_options_, plant_,
                             context_, plant_ad_, context_ad_, contact_pairs_);

  // Add the previous best repositioning target to the candidate states at the
  // index 1 always. (Index 0 will become the current state.)
  if (!is_doing_c3_) {
    // Add the prev best repositioning target to the candidate states.
    Eigen::VectorXd repositioning_target_state = x_lcs_curr;
    repositioning_target_state.head(3) = prev_repositioning_target_;
    candidate_states.insert(candidate_states.begin(),
                            repositioning_target_state);
  }
  // Insert the current location at the beginning of the candidate states.
  candidate_states.insert(candidate_states.begin(), x_lcs_curr);
  int num_total_samples = candidate_states.size();

  if (verbose_) {
    std::cout << "num_total_samples: " << num_total_samples << std::endl;
  }

  // Update the set of sample locations under consideration.
  all_sample_locations_.clear();
  for (int i = 0; i < num_total_samples; i++) {
    all_sample_locations_.push_back(candidate_states[i].head(3));
  }

  // Make LCS objects for each sample.
  std::vector<solvers::LCS> candidate_lcs_objects;
  // These objects will be linearized the same as candidate_lcs_objects but will
  // have all contact pairs resolved.  These will not be used to solve c3 but
  // will be used to compute more realistic costs.
  std::vector<solvers::LCS> lcs_objects_for_cost_simulation;

  for (int i = 0; i < num_total_samples; i++) {
    // Context needs to be updated to create the LCS objects.
    UpdateContext(n_q_, n_v_, n_u_, plant_, context_, plant_ad_, context_ad_,
                  candidate_states[i]);

    // Create an LCS object.
    // Preprocessing the contact pairs
    vector<SortedPair<GeometryId>> resolved_contact_pairs;
    solvers::LCS lcs_object_sample = CreatePlaceholderLCS();

    if (i == 0) {
      resolved_contact_pairs = LCSFactory::PreProcessor(
          plant_, *context_, contact_pairs_,
          sampling_c3_options_.resolve_contacts_to_list
              [sampling_c3_options_.num_contacts_index_for_curr_location],
          c3_options_for_curr_location.num_friction_directions,
          c3_options_for_curr_location.num_contacts, verbose_);
      lcs_object_sample = solvers::LCSFactory::LinearizePlantToLCS(
          plant_, *context_, plant_ad_, *context_ad_, resolved_contact_pairs,
          c3_options_for_curr_location.num_friction_directions,
          c3_options_for_curr_location.mu, dt_, N_, contact_model_);
    } else {
      resolved_contact_pairs = LCSFactory::PreProcessor(
          plant_, *context_, contact_pairs_,
          sampling_c3_options_.resolve_contacts_to_list
              [sampling_c3_options_.num_contacts_index],
          c3_options.num_friction_directions, c3_options.num_contacts,
          verbose_);
      lcs_object_sample = solvers::LCSFactory::LinearizePlantToLCS(
          plant_, *context_, plant_ad_, *context_ad_, resolved_contact_pairs,
          c3_options.num_friction_directions, c3_options.mu, dt_, N_,
          contact_model_);
    }

    // Store LCS object.
    candidate_lcs_objects.push_back(lcs_object_sample);

    if (sampling_params_.use_different_contacts_to_compute_cost) {
      // Create an LCS object with all contact pairs resolved.  In order, that
      // is 3 EE-capsule contacts, and 6 ground-capsule_tip contacts.

      // Preprocessing the contact pairs.
      if (verbose_) {
        std::cout << "Using different number of contacts to compute cost."
                  << std::endl;
      }
      vector<SortedPair<GeometryId>> resolved_contact_pairs_for_cost_simulation;
      resolved_contact_pairs_for_cost_simulation = LCSFactory::PreProcessor(
          plant_, *context_, contact_pairs_,
          sampling_c3_options_.resolve_contacts_to_list
              [sampling_c3_options_.num_contacts_index_for_cost],
          sampling_c3_options_.num_friction_directions,
          sampling_c3_options_.num_contacts_list
              [sampling_c3_options_.num_contacts_index_for_cost],
          verbose_);
      solvers::LCS lcs_object_sample_for_cost_simulation =
          solvers::LCSFactory::LinearizePlantToLCS(
              plant_, *context_, plant_ad_, *context_ad_,
              resolved_contact_pairs_for_cost_simulation,
              sampling_c3_options_.num_friction_directions,
              sampling_c3_options_
                  .mu_list[sampling_c3_options_.num_contacts_index_for_cost],
              dt_, N_, contact_model_);
      lcs_objects_for_cost_simulation.push_back(
          lcs_object_sample_for_cost_simulation);
    }
  }
  // Reset the context to the current lcs state.
  UpdateContext(n_q_, n_v_, n_u_, plant_, context_, plant_ad_, context_ad_,
                x_lcs_curr);

  if (verbose_) {
    // Print the LCS matrices for verbose inspection.
    solvers::LCS verbose_lcs = candidate_lcs_objects.at(CURRENT_LOCATION_INDEX);
    std::cout << "A: " << std::endl;
    std::cout << verbose_lcs.A_[0] << std::endl;
    std::cout << "B: " << std::endl;
    std::cout << verbose_lcs.B_[0] << std::endl;
    std::cout << "D: " << std::endl;
    std::cout << verbose_lcs.D_[0] << std::endl;
    std::cout << "d: " << std::endl;
    std::cout << verbose_lcs.d_[0] << std::endl;
    std::cout << "E: " << std::endl;
    std::cout << verbose_lcs.E_[0] << std::endl;
    std::cout << "F: " << std::endl;
    std::cout << verbose_lcs.F_[0] << std::endl;
    std::cout << "H: " << std::endl;
    std::cout << verbose_lcs.H_[0] << std::endl;
    std::cout << "c: " << std::endl;
    std::cout << verbose_lcs.c_[0] << std::endl;
  }

  // Preparation for parallelization.
  // This size is adapting to the number of samples based on mode.
  all_sample_costs_ = std::vector<double>(num_total_samples, -1);
  all_sample_dynamically_feasible_plans_ =
      std::vector<std::vector<Eigen::VectorXd>>(
          num_total_samples,
          std::vector<Eigen::VectorXd>(N_ + 1, VectorXd::Zero(n_x_)));
  std::vector<std::shared_ptr<solvers::C3>> c3_objects(num_total_samples,
                                                       nullptr);
  std::vector<std::vector<Eigen::VectorXd>> deltas(
      num_total_samples, std::vector<Eigen::VectorXd>(
                             N_, VectorXd::Zero(n_x_ + n_lambda_ + n_u_)));
  std::vector<std::vector<Eigen::VectorXd>> ws(
      num_total_samples, std::vector<Eigen::VectorXd>(
                             N_, VectorXd::Zero(n_x_ + n_lambda_ + n_u_)));

// Parallelize over computing C3 costs for each sample.
#pragma omp parallel for num_threads(num_threads_to_use_)
  for (int i = 0; i < num_total_samples; i++) {
    // Get the candidate state, its LCS representation.
    Eigen::VectorXd test_state = candidate_states.at(i);
    solvers::LCS test_system = candidate_lcs_objects.at(i);
    std::vector<Eigen::MatrixXd> G = G_;
    std::vector<Eigen::MatrixXd> U = U_;
    std::vector<Eigen::VectorXd> delta = deltas.at(i);
    std::vector<Eigen::VectorXd> w = ws.at(i);

    // Optionally the current location can use a different number of contacts
    // than the other samples.
    if ((i == 0) &&
        (sampling_c3_options_.num_contacts_index_for_curr_location !=
         sampling_c3_options_.num_contacts_index)) {
      vector<SortedPair<GeometryId>> resolved_contact_pairs_for_curr_location;
      resolved_contact_pairs_for_curr_location = LCSFactory::PreProcessor(
          plant_, *context_, contact_pairs_,
          sampling_c3_options_.resolve_contacts_to_list
              [sampling_c3_options_.num_contacts_index_for_curr_location],
          c3_options_for_curr_location.num_friction_directions,
          c3_options_for_curr_location.num_contacts, verbose_);
      test_system = solvers::LCSFactory::LinearizePlantToLCS(
          plant_, *context_, plant_ad_, *context_ad_,
          resolved_contact_pairs_for_curr_location,
          c3_options.num_friction_directions, c3_options.mu, dt_, N_,
          contact_model_);

      G = G_for_curr_location_;
      U = U_for_curr_location_;
      int n_lambda = 2 * c3_options_for_curr_location.num_friction_directions *
                     c3_options_for_curr_location.num_contacts;
      delta = std::vector<Eigen::VectorXd>(
          N_, VectorXd::Zero(n_x_ + n_lambda + n_u_));
      w = std::vector<Eigen::VectorXd>(N_,
                                       VectorXd::Zero(n_x_ + n_lambda + n_u_));
    }

    // Set up C3 MIQP.
    std::shared_ptr<solvers::C3> test_c3_object;

    if (c3_options.projection_type == "MIQP") {
      test_c3_object = std::make_shared<C3MIQP>(
          test_system, C3::CostMatrices(Q_, R_, G, U), x_desired, c3_options);
    } else if (c3_options.projection_type == "QP") {
      test_c3_object = std::make_shared<C3QP>(
          test_system, C3::CostMatrices(Q_, R_, G, U), x_desired, c3_options);
    } else {
      std::cerr << ("Unknown projection type") << std::endl;
      DRAKE_THROW_UNLESS(false);
    }

    if (sampling_params_.use_different_contacts_to_compute_cost) {
      // Compute the cost using the lcs object with more contacts.
      test_c3_object->UpdateCostLCS(lcs_objects_for_cost_simulation.at(i));
    }

    // Solve C3, store resulting object and cost.
    test_c3_object->SetOsqpSolverOptions(solver_options_);
    // test_c3_object->Solve(test_state, delta, w, verbose_);
    test_c3_object->Solve(test_state, verbose_);
    // Get the state solution and calculate the cost.
    // This is taking in the xbox input to change the way we calculate cost
    // type 3 based on if force tracking is on or off.
    std::pair<double, std::vector<Eigen::VectorXd>> cost_trajectory_pair;
    if (!crossed_cost_switching_threshold_) {
      if (i == 0) {
        cost_trajectory_pair = test_c3_object->CalcCost(
            sampling_params_.cost_type_position_tracking,
            sampling_c3_options_.Kp_for_cost_type_3,
            sampling_c3_options_.Kd_for_cost_type_3, radio_out->channel[11],
            radio_out->channel[7], verbose_);
      } else {
        cost_trajectory_pair = test_c3_object->CalcCost(
            sampling_params_.cost_type_position_tracking,
            sampling_c3_options_.Kp_for_cost_type_3,
            sampling_c3_options_.Kd_for_cost_type_3, radio_out->channel[11],
            false, verbose_);
      }
    } else {
      if (i == 0) {
        cost_trajectory_pair = test_c3_object->CalcCost(
            sampling_params_.cost_type, sampling_c3_options_.Kp_for_cost_type_3,
            sampling_c3_options_.Kd_for_cost_type_3, radio_out->channel[11],
            radio_out->channel[7], verbose_);
      } else {
        cost_trajectory_pair = test_c3_object->CalcCost(
            sampling_params_.cost_type, sampling_c3_options_.Kp_for_cost_type_3,
            sampling_c3_options_.Kd_for_cost_type_3, radio_out->channel[11],
            false, verbose_);
      }
    }

    double c3_cost = cost_trajectory_pair.first;
    all_sample_dynamically_feasible_plans_.at(i) = cost_trajectory_pair.second;

#pragma omp critical
    {
      c3_objects.at(i) = test_c3_object;
    }
    // Add travel cost.  Ignore differences in z.
    double xy_travel_distance =
        (test_state.head(2) - x_lcs_curr.head(2)).norm();
    all_sample_costs_[i] =
        c3_cost + sampling_params_.travel_cost_per_meter * xy_travel_distance;
    // Add additional costs based on repositioning progress.
    if ((i == CURRENT_REPOSITION_INDEX) &&
        (finished_reposition_flag_ == true)) {
      all_sample_costs_[i] += sampling_params_.finished_reposition_cost;
      finished_reposition_flag_ = false;
    }
  }
  // End of parallelization

  // Set up hysteresis values based on if the cost switching threshold has been
  // crossed.
  double c3_to_repos_hysteresis;
  double repos_to_c3_hysteresis;
  double hysteresis_between_repos_targets;

  double c3_to_repos_cost_fraction;
  double repos_to_c3_cost_fraction;
  double repos_to_repos_cost_fraction;

  // Set hysteresis values based on in position- or pose-tracking mode.
  if (!crossed_cost_switching_threshold_) {
    // Use the position tracking hysteresis values.
    c3_to_repos_hysteresis =
        sampling_params_.c3_to_repos_hysteresis_position_tracking;
    repos_to_c3_hysteresis =
        sampling_params_.repos_to_c3_hysteresis_position_tracking;
    hysteresis_between_repos_targets =
        sampling_params_.hysteresis_between_repos_targets_position_tracking;

    c3_to_repos_cost_fraction =
        sampling_params_.c3_to_repos_cost_fraction_position_tracking;
    repos_to_c3_cost_fraction =
        sampling_params_.repos_to_c3_cost_fraction_position_tracking;
    repos_to_repos_cost_fraction =
        sampling_params_.repos_to_repos_cost_fraction_position_tracking;
  } else {
    // Use the position and orientation tracking hysteresis values.
    c3_to_repos_hysteresis = sampling_params_.c3_to_repos_hysteresis;
    repos_to_c3_hysteresis = sampling_params_.repos_to_c3_hysteresis;
    hysteresis_between_repos_targets =
        sampling_params_.hysteresis_between_repos_targets;

    c3_to_repos_cost_fraction = sampling_params_.c3_to_repos_cost_fraction;
    repos_to_c3_cost_fraction = sampling_params_.repos_to_c3_cost_fraction;
    repos_to_repos_cost_fraction =
        sampling_params_.repos_to_repos_cost_fraction;
  }

  // Update the sample buffer.  Do this before switching modes since 1) if in
  // repositioning mode, don't add the repositioning target over and over again,
  // and 2) since the best sample in the buffer may be the best sample overall
  // and could be considered as a repositioning target.
  MaintainSampleBuffer(x_lcs_curr);
  // Add the best from the buffer to the samples, but only if in C3 mode and
  // only if the best in the buffer is distinct from the current set of samples.
  if ((is_doing_c3_) &&
      (sampling_params_.consider_best_buffer_sample_when_leaving_c3)) {
    double lowest_buffer_cost = sample_costs_buffer_[num_in_buffer_ - 1];
    Vector3d best_buffer_ee_sample =
        sample_buffer_.row(num_in_buffer_ - 1).head(3);

    double lowest_new_cost =
        *std::min_element(all_sample_costs_.begin(), all_sample_costs_.end());
    std::vector<double>::iterator it = std::min_element(
        std::begin(all_sample_costs_), std::end(all_sample_costs_));
    int lowest_new_cost_index =
        (SampleIndex)(std::distance(std::begin(all_sample_costs_), it));
    Vector3d best_new_ee_sample = all_sample_locations_[lowest_new_cost_index];

    // Initialize the buffer plan with something.
    if (dynamically_feasible_buffer_plan_.size() != N_ + 1) {
      c3_buffer_plan_ = c3_objects[lowest_new_cost_index];
      dynamically_feasible_buffer_plan_ =
          all_sample_dynamically_feasible_plans_[lowest_new_cost_index];
    }
    // If the best in the buffer is from the current set of samples, store the
    // associated C3 object and dynamically feasible plan, but don't add to the
    // set of samples to consider for repositioning.
    else if ((abs(lowest_buffer_cost - lowest_new_cost) < 1e-5) &&
             ((best_buffer_ee_sample - best_new_ee_sample).norm() < 1e-5)) {
      c3_buffer_plan_ = c3_objects[lowest_new_cost_index];
      dynamically_feasible_buffer_plan_ =
          all_sample_dynamically_feasible_plans_[lowest_new_cost_index];
    }
    // If the best in the buffer is distinct from the current set of samples,
    // consider it for repositioning by adding it to the set of samples, costs,
    // etc.
    else {
      all_sample_costs_.push_back(lowest_buffer_cost);
      all_sample_locations_.push_back(best_buffer_ee_sample);  // TODO problem?
      c3_objects.push_back(c3_buffer_plan_);
      all_sample_dynamically_feasible_plans_.push_back(
          dynamically_feasible_buffer_plan_);
    }
  }

  // Review the cost results to determine the best sample.
  double best_additional_sample_cost;
  if (num_total_samples > 1) {
    std::vector<double> additional_sample_cost_vector = std::vector<double>(
        all_sample_costs_.begin() + 1, all_sample_costs_.end());
    best_additional_sample_cost =
        *std::min_element(additional_sample_cost_vector.begin(),
                          additional_sample_cost_vector.end());
    std::vector<double>::iterator it =
        std::min_element(std::begin(additional_sample_cost_vector),
                         std::end(additional_sample_cost_vector));
    best_sample_index_ =
        (SampleIndex)(std::distance(std::begin(additional_sample_cost_vector),
                                    it) +
                      1);
  } else {
    // If there is no other sample, then set the best sample to the current
    // location and set the cost to a high value to ensure that the c3 is
    // chosen. Use absolute or relative hyseteresis based on the flag.
    if (!sampling_params_.use_relative_hysteresis) {
      best_additional_sample_cost = all_sample_costs_[CURRENT_LOCATION_INDEX] +
                                    c3_to_repos_hysteresis + 99;
    } else {
      best_additional_sample_cost =
          all_sample_costs_[CURRENT_LOCATION_INDEX] +
          (c3_to_repos_cost_fraction)*all_sample_costs_
              [CURRENT_LOCATION_INDEX] +
          99;
    }
  }

  if (verbose_) {
    std::cout << "All sample costs before hystereses: " << std::endl;
    for (int i = 0; i < num_total_samples; i++) {
      std::cout << "Sample " << i << " cost: " << all_sample_costs_[i]
                << std::endl;
    }
    std::cout << "In C3 mode? " << is_doing_c3_ << std::endl;
  }

  // Determine whether to do C3 or reposition.
  mode_switch_reason_ = MODE_SWITCH_REASON_NONE;
  if (is_doing_c3_ == true) {  // Currently doing C3.
    pursued_target_source_ = TARGET_SOURCE_NONE;

    // Update the lowest cost and position/orientation errors seen in this mode.
    bool updated_cost = false;
    bool updated_curr_pos_and_rot_cost = false;
    bool updated_pos_or_rot = false;
    bool reset_progress_cost_buffer = false;

    Eigen::MatrixXd Q_pos_and_rot = Q_[0].block(3, 3, 7, 7);
    Eigen::VectorXd pos_and_rot_error_vec =
        x_lcs_curr.segment(3, 7) - x_lcs_final_des.get_value().segment(3, 7);
    double curr_pos_and_rot_cost = pos_and_rot_error_vec.transpose() *
                                   Q_pos_and_rot * pos_and_rot_error_vec;

    if ((all_sample_costs_[CURRENT_LOCATION_INDEX] < lowest_cost_) ||
        (lowest_cost_ == -1.0)) {
      lowest_cost_ = all_sample_costs_[CURRENT_LOCATION_INDEX];
      updated_cost = true;
    }
    if ((curr_pos_and_rot_cost < lowest_pos_and_rot_current_cost_) ||
        (lowest_pos_and_rot_current_cost_ == -1.0)) {
      lowest_pos_and_rot_current_cost_ = curr_pos_and_rot_cost;
      updated_curr_pos_and_rot_cost = true;
    }
    if ((current_position_error_ < lowest_position_error_) ||
        (lowest_position_error_ == -1.0)) {
      lowest_position_error_ = current_position_error_;
      updated_pos_or_rot = true;
    }
    if ((current_orientation_error_ < lowest_orientation_error_) ||
        (lowest_orientation_error_ == -1.0)) {
      lowest_orientation_error_ = current_orientation_error_;
      updated_pos_or_rot = true;
    }

    if (radio_out->channel[6]) {
      std::cout << "Current rot and pos cost: " << curr_pos_and_rot_cost
                << std::endl;
    }

    if (progress_cost_buffer_.size() <
        sampling_params_.num_control_loops_to_wait_for_progress) {
      // If doing c3, add the current cost to the buffer if the buffer is not
      // full.
      progress_cost_buffer_.push(curr_pos_and_rot_cost);
    }
    if (progress_cost_buffer_.size() ==
        sampling_params_.num_control_loops_to_wait_for_progress) {
      // If doing c3 and tracking progress via the minimum progress to continue,
      // then check if the progress is enough to continue.
      double percentage_progress_in_cost =
          ((progress_cost_buffer_.back() - progress_cost_buffer_.front()) /
           progress_cost_buffer_.front()) *
          100;
      if (percentage_progress_in_cost >
          sampling_params_.min_percentage_decrease_in_cost_to_continue) {
        // If the progress is not enough or if it made negative progress in
        // those many loops, then reset the buffer.
        while (!progress_cost_buffer_.empty()) {
          progress_cost_buffer_.pop();
        }
        reset_progress_cost_buffer = true;
      } else {
        // if progress is enough, then pop the first element from the buffer.
        progress_cost_buffer_.pop();
      }
    }

    // Keep track of how many control loops have passed since the best seen
    // progress in this mode.
    if (((sampling_params_.track_c3_progress_via == C3_COST) && updated_cost) ||
        ((sampling_params_.track_c3_progress_via ==
          CURRENT_POSITION_AND_ORIENTATION_COST) &&
         updated_curr_pos_and_rot_cost) ||
        ((sampling_params_.track_c3_progress_via ==
          POSITION_OR_ORIENTATION_ERROR) &&
         updated_pos_or_rot)) {
      best_progress_steps_ago_ = 0;
    } else {
      best_progress_steps_ago_++;
    }

    // Switch to repositioning if the lowest seen cost while in C3 mode was
    // too many control loops ago.
    int num_control_loops_to_wait;
    if (!crossed_cost_switching_threshold_) {
      num_control_loops_to_wait =
          sampling_params_.num_control_loops_to_wait_position_tracking;
    } else {
      num_control_loops_to_wait = sampling_params_.num_control_loops_to_wait;
    }

    if (((sampling_params_.track_c3_progress_via != MIN_PROGRESS_TO_CONTINUE &&
          best_progress_steps_ago_ > num_control_loops_to_wait) ||
         (sampling_params_.track_c3_progress_via == MIN_PROGRESS_TO_CONTINUE &&
          reset_progress_cost_buffer)) &&
        (sampling_params_.num_additional_samples_c3 > 0) &&
        (!radio_out->channel[12])) {
      is_doing_c3_ = false;
      finished_reposition_flag_ = false;
      mode_switch_reason_ = MODE_SWITCH_TO_REPOS_UNPRODUCTIVE;
      std::cout << "Repositioning after not making progress in C3" << std::endl;

      lowest_cost_ = -1.0;
      lowest_pos_and_rot_current_cost_ = -1.0;
      lowest_position_error_ = -1.0;
      lowest_orientation_error_ = -1.0;
      best_progress_steps_ago_ = -1;
    }

    // Switch to repositioning if one of the other samples is better, with
    // hysteresis. Use absolute hysteresis values or percentage based hysteresis
    // based on the flag.
    else if ((all_sample_costs_[CURRENT_LOCATION_INDEX] >
                  best_additional_sample_cost + c3_to_repos_hysteresis &&
              !sampling_params_.use_relative_hysteresis) ||
             (sampling_params_.use_relative_hysteresis &&
              all_sample_costs_[CURRENT_LOCATION_INDEX] >
                  best_additional_sample_cost +
                      (c3_to_repos_cost_fraction)*all_sample_costs_
                          [CURRENT_LOCATION_INDEX]) &&
                 !radio_out->channel[12]) {
      is_doing_c3_ = false;
      finished_reposition_flag_ = false;
      mode_switch_reason_ = MODE_SWITCH_TO_REPOS_COST;
      std::cout << "Repositioning because found good sample" << std::endl;
    }

    // Determine the source of the repositioning target.
    if (!is_doing_c3_) {
      if (best_sample_index_ > sampling_params_.num_additional_samples_c3) {
        pursued_target_source_ = TARGET_SOURCE_FROM_BUFFER;
      } else {
        pursued_target_source_ = TARGET_SOURCE_NEW_SAMPLE;
      }
    }
  } else {  // Currently repositioning.
    // First, apply hysteresis between repositioning targets.
    if (best_sample_index_ == CURRENT_REPOSITION_INDEX) {
      pursued_target_source_ = TARGET_SOURCE_PREVIOUS;
    } else {
      // This means that the best sample is not the current repositioning
      // target. If the cost of the current repositioning target is better than
      // the current best sample by the hysteresis amount, then we continue
      // pursuing the previous repositioning target.
      if ((all_sample_costs_[CURRENT_REPOSITION_INDEX] <
               all_sample_costs_[best_sample_index_] +
                   hysteresis_between_repos_targets &&
           !sampling_params_.use_relative_hysteresis) ||
          (sampling_params_.use_relative_hysteresis &&
           all_sample_costs_[CURRENT_REPOSITION_INDEX] <
               all_sample_costs_[best_sample_index_] +
                   repos_to_repos_cost_fraction *
                       all_sample_costs_[CURRENT_REPOSITION_INDEX])) {
        best_sample_index_ = CURRENT_REPOSITION_INDEX;
        best_additional_sample_cost =
            all_sample_costs_[CURRENT_REPOSITION_INDEX];
        finished_reposition_flag_ = false;
        pursued_target_source_ = TARGET_SOURCE_PREVIOUS;
      }
      // Controller will switch to pursuing a new sample from its previous
      // repositioning target only if the cost of switching to that new sample
      // (with repos_to_repos hysteresis) is less than switching to C3 from
      // current location (with repos_to_c3 hysteresis), so add the
      // repos_to_repos hysteresis value here before the comparison to the
      // current location C3 cost with repos_to_c3 hysteresis afterwards.
      else {
        pursued_target_source_ = TARGET_SOURCE_NEW_SAMPLE;

        if (!sampling_params_.use_relative_hysteresis) {
          best_additional_sample_cost += hysteresis_between_repos_targets;
        } else {
          best_additional_sample_cost +=
              repos_to_repos_cost_fraction *
              all_sample_costs_[CURRENT_REPOSITION_INDEX];
        }
      }
    }

    // Switch to C3 if the current sample is better, with hysteresis.
    if ((best_additional_sample_cost >
             all_sample_costs_[CURRENT_LOCATION_INDEX] +
                 repos_to_c3_hysteresis &&
         !sampling_params_.use_relative_hysteresis) ||
        (sampling_params_.use_relative_hysteresis &&
         best_additional_sample_cost >
             all_sample_costs_[CURRENT_LOCATION_INDEX] +
                 repos_to_c3_cost_fraction * best_additional_sample_cost)) {
      is_doing_c3_ = true;
      finished_reposition_flag_ = false;
      if (all_sample_costs_[CURRENT_REPOSITION_INDEX] >
          sampling_params_.finished_reposition_cost) {
        mode_switch_reason_ = MODE_SWITCH_TO_C3_REACHED_REPOS_GOAL;
        std::cout << "Switching to C3 because reached repositioning target"
                  << std::endl;
      } else {
        mode_switch_reason_ = MODE_SWITCH_TO_C3_COST;
        std::cout << "Switching to C3 because lower in cost" << std::endl;
      }
      pursued_target_source_ = TARGET_SOURCE_NONE;

      // Reset the lowest cost seen in this mode.
      lowest_cost_ = -1.0;
      lowest_pos_and_rot_current_cost_ = -1.0;
      lowest_position_error_ = -1.0;
      lowest_orientation_error_ = -1.0;
      best_progress_steps_ago_ = 0;
      // reset progress cost buffer.
      while (!progress_cost_buffer_.empty()) {
        progress_cost_buffer_.pop();
      }
    }

    // Xbox controller override to force staying in C3 mode.
    if (radio_out->channel[12]) {
      std::cout << "Forcing into C3 mode" << std::endl;
      is_doing_c3_ = true;
      mode_switch_reason_ = MODE_SWITCH_TO_C3_XBOX;
      pursued_target_source_ = TARGET_SOURCE_NONE;

      // Reset the lowest cost seen in this mode.
      lowest_cost_ = -1.0;
      lowest_pos_and_rot_current_cost_ = -1.0;
      lowest_position_error_ = -1.0;
      lowest_orientation_error_ = -1.0;
      best_progress_steps_ago_ = 0;
      // reset progress cost buffer.
      while (!progress_cost_buffer_.empty()) {
        progress_cost_buffer_.pop();
      }
    }
  }

  if (verbose_) {
    std::cout << "All sample costs before hystereses: " << std::endl;
    for (int i = 0; i < num_total_samples; i++) {
      std::cout << "Sample " << i << " cost: " << all_sample_costs_[i]
                << std::endl;
    }
    std::cout << "In C3 mode after considering switch? " << is_doing_c3_
              << std::endl;
  }

  // These class variables get populated in the constructor in order to set osqp
  // options. They get replaced by the following values.
  // Update C3 objects and intermediates for current and best samples.
  c3_curr_plan_ = c3_objects.at(CURRENT_LOCATION_INDEX);
  c3_best_plan_ = c3_objects.at(best_sample_index_);
  // If doing warmstarting, will want to set the z, delta, and w vectors.

  // This is a redundant value used only for plotting and analysis.
  // update current and best sample costs.
  curr_and_best_sample_cost_ = {all_sample_costs_[CURRENT_LOCATION_INDEX],
                                all_sample_costs_[best_sample_index_]};

  // If a pursued repositioning target is from the buffer and not a new sample,
  // remove it from the buffer.
  if ((!is_doing_c3_) && (all_sample_costs_[best_sample_index_] ==
                          sample_costs_buffer_[num_in_buffer_ - 1])) {
    sample_buffer_.row(num_in_buffer_ - 1) = VectorXd::Zero(n_q_);
    sample_costs_buffer_[num_in_buffer_ - 1] = -1;
    num_in_buffer_--;
  }

  // Update the execution trajectories.  Both C3 and repositioning trajectories
  // are updated, but the is_doing_c3_ flag determines which one is used via the
  // downstream selector system.
  // Grab the drake context time and pass it to the C3traj & Repostraj update
  // functions.
  double t = context.get_discrete_state(plan_start_time_index_)[0];
  UpdateC3ExecutionTrajectory(x_lcs_curr, t);
  UpdateRepositioningExecutionTrajectory(x_lcs_curr, t);

  if (verbose_) {
    std::cout << "x_pred_curr_plan_ after updating: "
              << x_pred_curr_plan_.transpose() << std::endl;
    std::vector<VectorXd> zs = c3_curr_plan_->GetFullSolution();
    for (int i = 0; i < N_; i++) {
      std::cout << "z[" << i << "]: " << zs[i].transpose() << std::endl;
    }
    solvers::LCS verbose_lcs = candidate_lcs_objects.at(CURRENT_LOCATION_INDEX);
    Eigen::MatrixXd E = verbose_lcs.E_[0];
    Eigen::MatrixXd F = verbose_lcs.F_[0];
    Eigen::MatrixXd H = verbose_lcs.H_[0];
    Eigen::VectorXd c = verbose_lcs.c_[0];
    std::cout << "\nRight side of complementarity: " << std::endl;
    for (int i = 0; i < N_; i++) {
      Eigen::VectorXd x = zs[i].head(n_x_);
      Eigen::VectorXd lambda = zs[i].segment(n_x_, n_lambda_);
      Eigen::VectorXd u = zs[i].tail(n_u_);
      std::cout << "\t" << i << ": "
                << (E * x + F * lambda + H * u + c).transpose() << std::endl;
    }
    std::cout << "\nComplementarity violation: " << std::endl;
    for (int i = 0; i < N_; i++) {
      Eigen::VectorXd x = zs[i].head(n_x_);
      Eigen::VectorXd lambda = zs[i].segment(n_x_, n_lambda_);
      Eigen::VectorXd u = zs[i].tail(n_u_);
      std::cout << "\t" << i << ": "
                << lambda.dot(E * x + F * lambda + H * u + c) << std::endl;
    }

    std::cout << "Dynamically feasible ee current plan: " << std::endl;
    for (int i = 0; i < N_ + 1; i++) {
      std::cout << all_sample_dynamically_feasible_plans_
                       .at(CURRENT_LOCATION_INDEX)[i]
                       .segment(0, 3)
                       .transpose()
                << std::endl;
    }

    std::cout << "Dynamically feasible object current plan: " << std::endl;
    for (int i = 0; i < N_ + 1; i++) {
      std::cout << all_sample_dynamically_feasible_plans_
                       .at(CURRENT_LOCATION_INDEX)[i]
                       .segment(3, 7)
                       .transpose()
                << std::endl;
    }
  }

  // Adding delay
  std::this_thread::sleep_for(
      std::chrono::milliseconds(sampling_params_.control_loop_delay_ms));
  // End of control loop cleanup.
  auto finish = std::chrono::high_resolution_clock::now();
  auto elapsed = finish - start;
  double solve_time =
      std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() /
      1e6;
  filtered_solve_time_ = (1 - solve_time_filter_constant_) * solve_time +
                         (solve_time_filter_constant_)*filtered_solve_time_;

  if (verbose_) {
    std::cout << "At end of loop solve_time: " << solve_time << std::endl;
    std::cout << "At end of loop filtered_solve_time_: " << filtered_solve_time_
              << std::endl;
  }

  return drake::systems::EventStatus::Succeeded();
}

// Helper function to clamp end effector acceleration if using predicted state.
void SamplingC3Controller::ClampEndEffectorAcceleration(
    drake::VectorX<double>& x_lcs_curr) const {
  // Use fixed approximate loop time for acceleration capping heuristic.
  float approximate_loop_dt = std::min(0.1, filtered_solve_time_);
  float nominal_acceleration = 10;
  x_lcs_curr[0] =
      std::clamp(x_pred_curr_plan_[0],
                 x_lcs_curr[0] - nominal_acceleration * approximate_loop_dt *
                                     approximate_loop_dt,
                 x_lcs_curr[0] + nominal_acceleration * approximate_loop_dt *
                                     approximate_loop_dt);
  x_lcs_curr[1] =
      std::clamp(x_pred_curr_plan_[1],
                 x_lcs_curr[1] - nominal_acceleration * approximate_loop_dt *
                                     approximate_loop_dt,
                 x_lcs_curr[1] + nominal_acceleration * approximate_loop_dt *
                                     approximate_loop_dt);
  x_lcs_curr[2] =
      std::clamp(x_pred_curr_plan_[2],
                 x_lcs_curr[2] - nominal_acceleration * approximate_loop_dt *
                                     approximate_loop_dt,
                 x_lcs_curr[2] + nominal_acceleration * approximate_loop_dt *
                                     approximate_loop_dt);
  x_lcs_curr[n_q_ + 0] = std::clamp(
      x_pred_curr_plan_[n_q_ + 0],
      x_lcs_curr[n_q_ + 0] - nominal_acceleration * approximate_loop_dt,
      x_lcs_curr[n_q_ + 0] + nominal_acceleration * approximate_loop_dt);
  x_lcs_curr[n_q_ + 1] = std::clamp(
      x_pred_curr_plan_[n_q_ + 1],
      x_lcs_curr[n_q_ + 1] - nominal_acceleration * approximate_loop_dt,
      x_lcs_curr[n_q_ + 1] + nominal_acceleration * approximate_loop_dt);
  x_lcs_curr[n_q_ + 2] = std::clamp(
      x_pred_curr_plan_[n_q_ + 2],
      x_lcs_curr[n_q_ + 2] - nominal_acceleration * approximate_loop_dt,
      x_lcs_curr[n_q_ + 2] + nominal_acceleration * approximate_loop_dt);
}

// Perform one step of C3.
void SamplingC3Controller::UpdateC3ExecutionTrajectory(
    const VectorXd& x_lcs, const double& t_context) const {
  // Read the time from the t_context input for setting timestamps.
  double t = t_context;

  // Get the input from the plan.
  vector<VectorXd> u_sol = c3_curr_plan_->GetInputSolution();
  // Get full state solution from the plan.
  vector<VectorXd> x_sol = c3_curr_plan_->GetStateSolution();

  // Setting up matrices to set up LCMTrajectory object.
  Eigen::MatrixXd knots = Eigen::MatrixXd::Zero(n_x_, N_);
  Eigen::VectorXd timestamps = Eigen::VectorXd::Zero(N_);

  // Set up matrices for LCMTrajectory object.
  for (int i = 0; i < N_; i++) {
    knots.col(i) = x_sol[i];
    timestamps[i] = t + filtered_solve_time_ + (i)*dt_;
  }

  if (is_doing_c3_) {
    if (filtered_solve_time_ < 2 * dt_ &&
        sampling_c3_options_.at_least_predict_first_planned_trajectory_knot) {
      std::cerr
          << "Using first planned trajectory knot as predicted state for c3."
          << std::endl;
      x_pred_curr_plan_ = knots.col(2);
    } else if (filtered_solve_time_ < (N_ - 1) * dt_) {
      int last_passed_index = filtered_solve_time_ / dt_;
      double fraction_to_next_index =
          (filtered_solve_time_ / dt_) - (double)last_passed_index;
      x_pred_curr_plan_ =
          knots.col(last_passed_index) +
          fraction_to_next_index *
              (knots.col(last_passed_index + 1) - knots.col(last_passed_index));
    } else {
      x_pred_curr_plan_ = knots.col(N_ - 1);
    }
  }

  // Add end effector position target to LCM Trajectory.
  LcmTrajectory::Trajectory ee_traj;
  ee_traj.traj_name = "end_effector_position_target";
  ee_traj.datatypes = std::vector<std::string>(3, "double");
  ee_traj.datapoints = knots(Eigen::seqN(0, 3), Eigen::seqN(0, N_));
  ee_traj.time_vector = timestamps.cast<double>();
  c3_execution_lcm_traj_.ClearTrajectories();
  c3_execution_lcm_traj_.AddTrajectory(ee_traj.traj_name, ee_traj);

  // Add end effector force target to LCM Trajectory.
  // In c3 mode, the end effector forces should match the solved c3 inputs.
  Eigen::MatrixXd force_samples = Eigen::MatrixXd::Zero(3, N_);
  for (int i = 0; i < N_; i++) {
    force_samples.col(i) = u_sol[i];
  }
  LcmTrajectory::Trajectory force_traj;
  force_traj.traj_name = "end_effector_force_target";
  force_traj.datatypes =
      std::vector<std::string>(force_samples.rows(), "double");
  force_traj.datapoints = force_samples;
  force_traj.time_vector = timestamps.cast<double>();
  c3_execution_lcm_traj_.AddTrajectory(force_traj.traj_name, force_traj);

  // Add object position target to LCM Trajectory.
  LcmTrajectory::Trajectory object_traj;
  object_traj.traj_name = "object_position_target";
  object_traj.datatypes = std::vector<std::string>(3, "double");
  object_traj.datapoints = knots(Eigen::seqN(n_q_ - 3, 3), Eigen::seqN(0, N_));
  object_traj.time_vector = timestamps.cast<double>();
  c3_execution_lcm_traj_.AddTrajectory(object_traj.traj_name, object_traj);

  // Add object orientation target to LCM Trajectory.
  LcmTrajectory::Trajectory object_orientation_traj;
  MatrixXd orientation_samples = MatrixXd::Zero(4, N_);
  orientation_samples = knots(Eigen::seqN(n_q_ - 7, 4), Eigen::seqN(0, N_));
  object_orientation_traj.traj_name = "object_orientation_target";
  object_orientation_traj.datatypes =
      std::vector<std::string>(orientation_samples.rows(), "double");
  object_orientation_traj.datapoints = orientation_samples;
  object_orientation_traj.time_vector = timestamps.cast<double>();
  c3_execution_lcm_traj_.AddTrajectory(object_orientation_traj.traj_name,
                                       object_orientation_traj);
}

// Compute repositioning trajectory.
void SamplingC3Controller::UpdateRepositioningExecutionTrajectory(
    const VectorXd& x_lcs, const double& t_context) const {
  // Get the best sample location.
  Eigen::Vector3d best_sample_location =
      all_sample_locations_[best_sample_index_];
  // Update the previous repositioning target for reference in next loop.
  prev_repositioning_target_ = best_sample_location;

  // Get the current end effector location.
  Vector3d current_ee_location = x_lcs.head(3);
  Vector3d current_object_location = x_lcs.segment(7, 3);

  Vector3d curr_to_goal_vec = best_sample_location - current_ee_location;

  // Get two unit vectors in the plane of the arc between the current and goal
  // ee locations.
  Eigen::Vector3d v1 =
      (current_ee_location - current_object_location).normalized();
  Eigen::Vector3d v2 =
      (best_sample_location - current_object_location).normalized();
  // NOTE:  Need to clamp between not quite (-1, 1) to avoid NaNs.
  double travel_angle = std::acos(std::clamp(v1.dot(v2), -0.9999, 0.9999));

  // Read the time from the t_context i/p for setting timestamps.
  double t = t_context;

  // Setting up matrices to set up LCMTrajectory object.
  Eigen::MatrixXd knots = Eigen::MatrixXd::Zero(n_x_, N_);
  Eigen::VectorXd timestamps = Eigen::VectorXd::Zero(N_);
  for (int i = 0; i < N_; i++) {
    timestamps[i] = t + filtered_solve_time_ + (i)*dt_;
  }

  // Compute total estimated travel time for spline.
  double travel_distance = curr_to_goal_vec.norm();
  double total_travel_time =
      travel_distance / sampling_params_.reposition_speed;

  // Use a straight line trajectory if close to the target (where "close"
  // depends on whether using spline or arc trajectory type).
  if ((travel_distance < sampling_params_.use_straight_line_traj_under &&
       sampling_params_.repositioning_trajectory_type == 0) ||
      ((sampling_params_.repositioning_trajectory_type == 1 ||
        sampling_params_.repositioning_trajectory_type == 2) &&
       travel_angle < sampling_params_.use_straight_line_traj_within_angle) ||
      (sampling_params_.repositioning_trajectory_type == 3 &&
       (current_ee_location.head(2) - best_sample_location.head(2)).norm() <
           0.008)) {
    Eigen::VectorXd times = Eigen::VectorXd::Zero(2);
    times[0] = 0;
    // Ensure the times used to define PiecewisePolynomial are increasing.
    times[1] = std::max(total_travel_time, 0.0001);

    Eigen::MatrixXd points = Eigen::MatrixXd::Zero(n_x_, 2);
    points.col(0) = x_lcs;
    VectorXd next_lcs_state = x_lcs;
    next_lcs_state.head(3) = best_sample_location;
    next_lcs_state.segment(n_q_, 3) = Vector3d::Zero();
    points.col(1) = next_lcs_state;
    auto trajectory =
        PiecewisePolynomial<double>::FirstOrderHold(times, points);

    for (int i = 0; i < N_; i++) {
      double t_line = std::min((i)*dt_, total_travel_time);
      knots.col(i) = trajectory.value(t_line);

      if (i == 1 && t_line >= total_travel_time && !is_doing_c3_) {
        // If it can get there in one step, then set finished_reposition_flag_
        // to true.
        finished_reposition_flag_ = true;
      }
    }
  }

  // Otherwise, uphold a spherical arc repositioning trajectory.
  else if (sampling_params_.repositioning_trajectory_type == 1) {
    // Get the two waypoints as the ends of the arc trajectory.
    Eigen::Vector3d waypoint1 =
        current_object_location +
        v1 * sampling_params_.spherical_repositioning_radius;
    Eigen::Vector3d waypoint2 =
        current_object_location +
        v2 * sampling_params_.spherical_repositioning_radius;

    Eigen::Vector3d v3 = v1.cross(v2).normalized();
    Eigen::Vector3d v4 = v3.cross(v1).normalized();

    // Ensure the traversed arc stays above the ground.
    if (v4(2) < -0.5 && travel_angle > M_PI_2) {
      v4 = -v4;
      travel_angle = 2 * M_PI - travel_angle;
    }
    // Prevent getting stuck if the EE is 180 degrees around from the target.
    if (travel_angle > 0.9 * M_PI) {
      Eigen::Vector3d almost_v4 = v1;
      almost_v4(2) += 1;
      v3 = v1.cross(almost_v4.normalized()).normalized();
      v4 = v3.cross(v1).normalized();
    }

    // The arc is defined by the equation:
    // x = x_c + r*cos(theta)*v1 + r*sin(theta)*v4
    // ...where theta should be [0, travel_angle] and r*dtheta should be the
    // desired travel distance.
    double dtheta = sampling_params_.reposition_speed * dt_ /
                    sampling_params_.spherical_repositioning_radius;
    double step_size = sampling_params_.reposition_speed * dt_;

    knots.col(0) = x_lcs;
    int i = 1;
    // Handle the first leg:  straight line from current EE location to
    // waypoint1.
    double dist_to_wp1 = (current_ee_location - waypoint1).norm();
    while ((i * step_size < dist_to_wp1) && (i < N_)) {
      Eigen::Vector3d straight_line_point =
          current_ee_location +
          i * step_size / dist_to_wp1 * (waypoint1 - current_ee_location);

      VectorXd next_lcs_state = x_lcs;
      next_lcs_state.head(3) = straight_line_point;
      knots.col(i) = next_lcs_state;
      i++;
    }

    // Handle the second leg:  arc from waypoint1 to waypoint2.
    int leg1_i = i;
    double dtheta0 = (i * step_size - dist_to_wp1) / step_size * dtheta;
    while ((dtheta0 + (i - leg1_i) * dtheta < travel_angle) && (i < N_)) {
      Eigen::Vector3d arc_point =
          current_object_location +
          sampling_params_.spherical_repositioning_radius *
              (std::cos(dtheta0 + (i - leg1_i) * dtheta) * v1 +
               std::sin(dtheta0 + (i - leg1_i) * dtheta) * v4);

      VectorXd next_lcs_state = x_lcs;
      next_lcs_state.head(3) = arc_point;
      knots.col(i) = next_lcs_state;
      i++;
    }

    // Handle the last leg:  straight line from waypoint2 to goal EE location.
    int leg2_i = i;
    double dstep =
        (dtheta0 + (i - leg1_i) * dtheta - travel_angle) / dtheta * step_size;
    double dist_wp2_to_goal = (waypoint2 - best_sample_location).norm();
    while ((dstep + (i - leg2_i) * step_size < dist_wp2_to_goal) && (i < N_)) {
      Eigen::Vector3d straight_line_point =
          waypoint2 + (dstep + (i - leg2_i) * step_size) / dist_wp2_to_goal *
                          (best_sample_location - waypoint2);

      VectorXd next_lcs_state = x_lcs;
      next_lcs_state.head(3) = straight_line_point;
      knots.col(i) = next_lcs_state;
      i++;
    }

    // Enforce minimum z height for end effector, with a small buffer.
    for (int j = 0; j < i; j++) {
      if (knots(2, j) < sampling_c3_options_.workspace_limits[2][3] + 0.005) {
        knots(2, j) = sampling_c3_options_.workspace_limits[2][3] + 0.005;
      }
    }

    // Fill in the rest of the knots with the goal EE location.
    for (int j = i; j < N_; j++) {
      Eigen::Vector3d x_lcs_goal = x_lcs;
      x_lcs_goal.head(3) = best_sample_location;
      knots.col(j) = x_lcs_goal;
      if (j == 1 && !is_doing_c3_) {
        finished_reposition_flag_ = true;
      }
    }
  }

  // Use a spline trajectory.
  else if (sampling_params_.repositioning_trajectory_type == 0) {
    // Compute spline waypoints.
    Vector3d p0 = current_ee_location;
    Vector3d p3 = best_sample_location;
    Vector3d p1 =
        current_ee_location + 0.25 * curr_to_goal_vec - current_object_location;
    p1 = current_object_location +
         sampling_params_.spline_width * p1 / p1.norm();
    Vector3d p2 =
        current_ee_location + 0.75 * curr_to_goal_vec - current_object_location;
    p2 = current_object_location +
         sampling_params_.spline_width * p2 / p2.norm();

    for (int i = 0; i < N_; i++) {
      // This is a curve_fraction and is not in the units of time or distance.
      // When it is 0, it is the current location. When it is 1, it is the goal.
      double t_spline = (i)*dt_ / total_travel_time;

      if (i == 1 && t_spline >= 1 && !is_doing_c3_) {
        // If it can get there in one step, then set finished_reposition_flag_
        // to true.
        finished_reposition_flag_ = true;
        std::cout
            << "WARNING! Using spline but finished repositioning in one step."
            << std::endl;
      }
      // Don't overshoot the end of the spline.
      t_spline = std::min(1.0, t_spline);

      Vector3d next_ee_loc =
          p0 + t_spline * (-3 * p0 + 3 * p1) +
          std::pow(t_spline, 2) * (3 * p0 - 6 * p1 + 3 * p2) +
          std::pow(t_spline, 3) * (-p0 + 3 * p1 - 3 * p2 + p3);

      // Set the next LCS state as the current state with updated end effector
      // location and zero end effector velocity. Note that this means that the
      // object does not move in the planned trajectory. An alternative is that
      // we could simulate the object's motion with 0 input.
      VectorXd next_lcs_state = x_lcs;
      next_lcs_state.head(3) = next_ee_loc;
      next_lcs_state.segment(n_q_, 3) = Vector3d::Zero();
      // if z is under the table, set it to a min height.
      if (next_lcs_state[2] < sampling_c3_options_.workspace_limits[2][3]) {
        next_lcs_state[2] = sampling_c3_options_.workspace_limits[2][3];
      }

      knots.col(i) = next_lcs_state;
    }
  }

  // use circular trajectory.
  else if (sampling_params_.repositioning_trajectory_type == 2) {
    // Current object projection onto the plane of the circle.
    Vector3d current_object_projection = current_object_location;
    current_object_projection(2) =
        sampling_params_.circular_repositioning_height;

    // Project current ee location onto the plane of the circle.
    Vector3d curr_ee_projection = current_ee_location;
    curr_ee_projection(2) = sampling_params_.circular_repositioning_height;

    // project best sample onto the repositioning plane.
    Vector3d best_sample_projection = best_sample_location;
    best_sample_projection(2) = sampling_params_.circular_repositioning_height;

    Vector3d v1 = (curr_ee_projection - current_object_projection).normalized();
    Vector3d v2 =
        (best_sample_projection - current_object_projection).normalized();

    // Compute travel angle
    double travel_angle = std::acos(v1.dot(v2));

    // Define the waypoints for the circular trajectory.
    Eigen::Vector3d waypoint1 =
        current_object_projection +
        sampling_params_.circular_repositioning_radius * v1;
    Eigen::Vector3d waypoint2 =
        current_object_projection +
        sampling_params_.circular_repositioning_radius * v2;

    // Compute tangent vector to the circle at waypoint1.
    Eigen::Vector3d v3 = v1.cross(v2).normalized();
    Eigen::Vector3d v4 = v3.cross(v1).normalized();

    if (travel_angle > M_PI) {
      travel_angle = 2 * M_PI - travel_angle;
      v4 = -v4;
    }

    // The arc is defined by the equation: x = x_c + r*cos(theta) + r*sin(theta)
    // the relation between the linear velocity and the angular velocity is
    // given by v = r*omega, where v is the linear velocity and omega is the
    // angular velocity. The angular velocity is given by omega = v/r. The
    // angular velocity is given by omega = dtheta/dt. Therefore, dtheta =
    // v/r*dt. This is the increment in the angle in the time dt.
    double dtheta = sampling_params_.reposition_speed * dt_ /
                    sampling_params_.circular_repositioning_radius;
    double step_size = sampling_params_.reposition_speed * dt_;

    knots.col(0) = x_lcs;
    int i = 1;
    // Handle the first leg:  straight line from current EE location to
    // waypoint1.
    double dist_to_wp1 = (current_ee_location - waypoint1).norm();
    while ((i * step_size < dist_to_wp1) && (i < N_)) {
      // The division by dist_to_wp1 is to normalize the direction vector from
      // the current ee location to waypoint1.
      Eigen::Vector3d straight_line_point =
          current_ee_location +
          i * step_size / dist_to_wp1 * (waypoint1 - current_ee_location);

      VectorXd next_lcs_state = x_lcs;
      next_lcs_state.head(3) = straight_line_point;
      knots.col(i) = next_lcs_state;
      i++;
    }

    // Handle the second leg:  arc from waypoint1 to waypoint2.
    int leg1_i = i;
    double dtheta0 = (i * step_size - dist_to_wp1) / step_size * dtheta;
    while (((i - leg1_i) * dtheta < travel_angle) && (i < N_)) {
      Eigen::Vector3d arc_point =
          current_object_projection +
          sampling_params_.circular_repositioning_radius *
              (std::cos(dtheta0 + (i - leg1_i) * dtheta) * v1 +
               std::sin(dtheta0 + (i - leg1_i) * dtheta) * v4);
      arc_point(2) = sampling_params_.circular_repositioning_height;

      VectorXd next_lcs_state = x_lcs;
      next_lcs_state.head(3) = arc_point;
      knots.col(i) = next_lcs_state;
      i++;
    }

    // Handle the last leg:  straight line from waypoint2 to goal EE location.
    int leg2_i = i;
    double dstep =
        (dtheta0 + (i - leg1_i) * dtheta - travel_angle) / dtheta * step_size;
    double dist_wp2_to_goal = (waypoint2 - best_sample_location).norm();
    while ((dstep + (i - leg2_i) * step_size < dist_wp2_to_goal) && (i < N_)) {
      Eigen::Vector3d straight_line_point =
          waypoint2 + (dstep + (i - leg2_i) * step_size) / dist_wp2_to_goal *
                          (best_sample_location - waypoint2);

      VectorXd next_lcs_state = x_lcs;
      next_lcs_state.head(3) = straight_line_point;
      knots.col(i) = next_lcs_state;
      i++;
    }

    // Fill in the rest of the knots with the goal EE location.
    for (int j = i; j < N_; j++) {
      Eigen::Vector3d x_lcs_goal = x_lcs;
      x_lcs_goal.head(3) = best_sample_location;
      knots.col(j) = x_lcs_goal;
      if (j == 1 && !is_doing_c3_) {
        finished_reposition_flag_ = true;
      }
    }

  }

  // use piecewise linear trajectory. Go up, go to point above sample, go back
  // down to sample.
  else if (sampling_params_.repositioning_trajectory_type == 3) {
    // Define the waypoints for the three-leg repositioning.
    Eigen::Vector3d waypoint_above_ee = current_ee_location;
    waypoint_above_ee(2) = sampling_params_.repositioning_waypoint_height;

    Eigen::Vector3d waypoint_above_sample = best_sample_location;
    waypoint_above_sample(2) = sampling_params_.repositioning_waypoint_height;

    knots.col(0) = x_lcs;
    int i = 1;
    double step_size = sampling_params_.reposition_speed * dt_;

    // First leg: straight line from current EE location to waypoint_above_ee.
    double dist_to_wp1 = (current_ee_location - waypoint_above_ee).norm();
    while ((i * step_size < dist_to_wp1) && (i < N_)) {
      Eigen::Vector3d straight_line_point =
          current_ee_location + i * step_size / dist_to_wp1 *
                                    (waypoint_above_ee - current_ee_location);

      VectorXd next_lcs_state = x_lcs;
      next_lcs_state.head(3) = straight_line_point;
      knots.col(i) = next_lcs_state;
      i++;
    }

    // Second leg: straight line from waypoint_above_ee to
    // waypoint_above_sample.
    int leg1_i = i;
    double dstep0 = i * step_size - dist_to_wp1;
    double dist_to_wp2 = (waypoint_above_ee - waypoint_above_sample).norm();
    while ((dstep0 + (i - leg1_i) * step_size < dist_to_wp2) && (i < N_)) {
      Eigen::Vector3d straight_line_point =
          waypoint_above_ee + (dstep0 + (i - leg1_i) * step_size) /
                                  dist_to_wp2 *
                                  (waypoint_above_sample - waypoint_above_ee);

      VectorXd next_lcs_state = x_lcs;
      next_lcs_state.head(3) = straight_line_point;
      knots.col(i) = next_lcs_state;
      i++;
    }

    // Third leg: straight line from waypoint_above_sample to
    // best_sample_location. Really this doesn't even get used because it enters
    // the use_straight_line condition before then.
    int leg2_i = i;
    double dstep1 = (i - leg1_i) * step_size - dist_to_wp2;
    double dist_to_goal = (waypoint_above_sample - best_sample_location).norm();
    while ((dstep1 + (i - leg2_i) * step_size < dist_to_goal) && (i < N_)) {
      Eigen::Vector3d straight_line_point =
          waypoint_above_sample +
          (dstep1 + (i - leg2_i) * step_size) / dist_to_goal *
              (best_sample_location - waypoint_above_sample);

      VectorXd next_lcs_state = x_lcs;
      next_lcs_state.head(3) = straight_line_point;
      knots.col(i) = next_lcs_state;
      i++;
    }

    // Fill in the rest of the knots with the goal EE location.
    for (int j = i; j < N_; j++) {
      Eigen::Vector3d x_lcs_goal = x_lcs;
      x_lcs_goal.head(3) = best_sample_location;
      knots.col(j) = x_lcs_goal;
      if (j == 1 && !is_doing_c3_) {
        finished_reposition_flag_ = true;
      }
    }
  }

  // Set predicted end effector state in preparation for next control loop, if
  // currently in repositioning mode.
  if (!is_doing_c3_) {
    if (filtered_solve_time_ < dt_ &&
        sampling_c3_options_.at_least_predict_first_planned_trajectory_knot) {
      std::cerr
          << "Using first planned trajectory knot as predicted state for repos."
          << std::endl;
      x_pred_curr_plan_ = knots.col(2);
    } else if (filtered_solve_time_ < (N_ - 1) * dt_) {
      int last_passed_index = filtered_solve_time_ / dt_;
      double fraction_to_next_index =
          (filtered_solve_time_ / dt_) - (double)last_passed_index;
      x_pred_curr_plan_ =
          knots.col(last_passed_index) +
          fraction_to_next_index *
              (knots.col(last_passed_index + 1) - knots.col(last_passed_index));
    } else {
      x_pred_curr_plan_ = knots.col(N_ - 1);
    }
  }

  // Add end effector position target to LCM Trajectory.
  LcmTrajectory::Trajectory ee_traj;
  ee_traj.traj_name = "end_effector_position_target";
  ee_traj.datatypes = std::vector<std::string>(3, "double");
  ee_traj.datapoints = knots(Eigen::seqN(0, 3), Eigen::seqN(0, N_));
  ee_traj.time_vector = timestamps.cast<double>();
  repos_execution_lcm_traj_.ClearTrajectories();
  repos_execution_lcm_traj_.AddTrajectory(ee_traj.traj_name, ee_traj);

  // Add end effector force target to LCM Trajectory.
  // In repositioning mode, the end effector should not exert forces.
  MatrixXd force_samples = MatrixXd::Zero(3, N_);
  LcmTrajectory::Trajectory force_traj;
  force_traj.traj_name = "end_effector_force_target";
  force_traj.datatypes =
      std::vector<std::string>(force_samples.rows(), "double");
  force_traj.datapoints = force_samples;
  force_traj.time_vector = timestamps.cast<double>();
  repos_execution_lcm_traj_.AddTrajectory(force_traj.traj_name, force_traj);

  // Add object position target to LCM Trajectory.
  LcmTrajectory::Trajectory object_traj;
  object_traj.traj_name = "object_position_target";
  object_traj.datatypes = std::vector<std::string>(3, "double");
  object_traj.datapoints = knots(Eigen::seqN(n_q_ - 3, 3), Eigen::seqN(0, N_));
  object_traj.time_vector = timestamps.cast<double>();
  repos_execution_lcm_traj_.AddTrajectory(object_traj.traj_name, object_traj);

  // Add object orientation target to LCM Trajectory.
  LcmTrajectory::Trajectory object_orientation_traj;
  MatrixXd orientation_samples = MatrixXd::Zero(4, N_);
  orientation_samples = knots(Eigen::seqN(n_q_ - 7, 4), Eigen::seqN(0, N_));
  object_orientation_traj.traj_name = "object_orientation_target";
  object_orientation_traj.datatypes = std::vector<std::string>(4, "double");
  object_orientation_traj.datapoints = orientation_samples;
  object_orientation_traj.time_vector = timestamps.cast<double>();
  repos_execution_lcm_traj_.AddTrajectory(object_orientation_traj.traj_name,
                                          object_orientation_traj);
}

// Maintain the sample buffer:  prunes outdated samples and adds new.
void SamplingC3Controller::MaintainSampleBuffer(const VectorXd& x_lcs) const {
  // Determine if samples are outdated by comparing to the current jack position
  // and orientation.
  Vector3d jack_pos = x_lcs.segment(7, 3);
  Eigen::Vector4d jack_quat = x_lcs.segment(3, 4).normalized();

  MatrixXd buffer_xyzs =
      sample_buffer_.block(0, 7, sampling_params_.N_sample_buffer, 3);
  MatrixXd buffer_quats =
      sample_buffer_.block(0, 3, sampling_params_.N_sample_buffer, 4);

  // First, remove outdated samples that have moved too much from current jack
  // configuration.
  VectorXd quat_dots = (buffer_quats * jack_quat).array().abs();
  VectorXd angles = (2.0 * quat_dots.array().acos());
  Eigen::Array<bool, Eigen::Dynamic, 1> mask_satisfies_rot =
      (angles.array() < sampling_params_.ang_error_sample_retention);

  MatrixXd pos_deltas = buffer_xyzs.rowwise() - jack_pos.transpose();
  VectorXd distances = pos_deltas.rowwise().norm();
  Eigen::Array<bool, Eigen::Dynamic, 1> mask_satisfies_pos =
      (distances.array() < sampling_params_.pos_error_sample_retention);

  MatrixXd retained_samples =
      MatrixXd::Zero(sampling_params_.N_sample_buffer, n_q_);
  VectorXd retained_costs =
      -1 * VectorXd::Ones(sampling_params_.N_sample_buffer);
  int retained_count = 0;
  for (int i = 0; i < sampling_params_.N_sample_buffer; i++) {
    if (mask_satisfies_rot[i] && mask_satisfies_pos[i]) {
      retained_samples.row(retained_count) = sample_buffer_.row(i);
      retained_costs[retained_count] = sample_costs_buffer_[i];
      retained_count++;
    } else if (sample_costs_buffer_[i] < 0) {
      break;
    }
  }
  sample_buffer_ = retained_samples;
  sample_costs_buffer_ = retained_costs;

  // Second, in preparation for adding new samples stored in
  // all_sample_locations_ (excluding the current location), if the buffer is
  // going to overflow, get rid of the oldest samples first.  NOTE:  Step 4
  // moves the lowest cost sample in the buffer to the end, so the best sample
  // is usually excluded from this cut.
  int num_to_add = all_sample_locations_.size() - 1;
  if (!is_doing_c3_) {
    num_to_add--;
  }
  if (retained_count + num_to_add > sampling_params_.N_sample_buffer) {
    int shift_by =
        retained_count + num_to_add - sampling_params_.N_sample_buffer;
    retained_count -= shift_by;
    sample_buffer_.block(0, 0, retained_count, n_q_) =
        sample_buffer_.block(shift_by, 0, retained_count, n_q_);
    sample_costs_buffer_.segment(0, retained_count) =
        sample_costs_buffer_.segment(shift_by, retained_count);
  }

  // Third, add the new samples stored in all_sample_locations_ and
  // all_sample_costs_.  Don't add the current location (so the sample buffer
  // contains more broadly sampled locations) or a currently pursued
  // repositioning target.
  int buffer_count = retained_count;
  for (int i = retained_count;
       i < retained_count + all_sample_locations_.size(); i++) {
    DRAKE_ASSERT(i >= sampling_params_.N_sample_buffer);
    if ((i == retained_count) || (!is_doing_c3_ && i == retained_count + 1)) {
      // Skip the current location.
      // Skip the repositioning target if in repositioning mode.
    } else {
      VectorXd new_config = x_lcs.segment(0, n_q_);
      new_config.segment(0, 3) = all_sample_locations_[i - retained_count];
      // Ensure a normalized quaternion is written to the buffer.
      new_config.segment(3, 4) = jack_quat;
      sample_buffer_.row(buffer_count) = new_config;
      sample_costs_buffer_[buffer_count] =
          all_sample_costs_[i - retained_count];
      buffer_count++;
    }
  }
  num_in_buffer_ = buffer_count;

  // Lastly, ensure the lowest cost sample is at the end of the buffer.
  VectorXd eligible_costs = sample_costs_buffer_.head(num_in_buffer_);
  int lowest_cost_index;
  double lowest_buffer_cost = eligible_costs.minCoeff(&lowest_cost_index);
  VectorXd lowest_cost_sample = sample_buffer_.row(lowest_cost_index);
  sample_buffer_.row(lowest_cost_index) =
      sample_buffer_.row(num_in_buffer_ - 1);
  sample_costs_buffer_[lowest_cost_index] =
      sample_costs_buffer_[num_in_buffer_ - 1];
  sample_buffer_.row(num_in_buffer_ - 1) = lowest_cost_sample;
  sample_costs_buffer_[num_in_buffer_ - 1] = lowest_buffer_cost;

  DRAKE_ASSERT(sample_buffer_.cols() == sampling_params_.N_sample_buffer);
  DRAKE_ASSERT(sample_buffer_.rows() == n_q_);
  DRAKE_ASSERT(sample_costs_buffer_.size() == sampling_params_.N_sample_buffer);
}

// Output port handlers for current location
void SamplingC3Controller::OutputC3SolutionCurrPlanActor(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* output) const {
  double t = context.get_discrete_state(plan_start_time_index_)[0];

  auto c3_solution = std::make_unique<C3Output::C3Solution>();
  c3_solution->x_sol_ = MatrixXf::Zero(n_q_ + n_v_, N_);
  c3_solution->lambda_sol_ = MatrixXf::Zero(n_lambda_, N_);
  c3_solution->u_sol_ = MatrixXf::Zero(n_u_, N_);
  c3_solution->time_vector_ = VectorXf::Zero(N_);

  auto z_sol = c3_curr_plan_->GetFullSolution();
  for (int i = 0; i < N_; i++) {
    c3_solution->time_vector_(i) = filtered_solve_time_ + t + i * dt_;
    c3_solution->x_sol_.col(i) = z_sol[i].segment(0, n_x_).cast<float>();
    c3_solution->lambda_sol_.col(i) =
        z_sol[i].segment(n_x_, n_lambda_).cast<float>();
    c3_solution->u_sol_.col(i) =
        z_sol[i].segment(n_x_ + n_lambda_, n_u_).cast<float>();
  }

  MatrixXd knots = MatrixXd::Zero(6, N_);
  knots.topRows(3) = c3_solution->x_sol_.topRows(3).cast<double>();
  knots.bottomRows(3) =
      c3_solution->x_sol_.bottomRows(n_v_).topRows(3).cast<double>();

  LcmTrajectory::Trajectory end_effector_traj;
  end_effector_traj.traj_name = "end_effector_position_target";
  end_effector_traj.datatypes =
      std::vector<std::string>(knots.rows(), "double");
  end_effector_traj.datapoints = knots;
  end_effector_traj.time_vector = c3_solution->time_vector_.cast<double>();
  LcmTrajectory lcm_traj({end_effector_traj}, {"end_effector_position_target"},
                         "end_effector_position_target",
                         "end_effector_position_target", false);

  // NOTE: End effector orientation functionality is not implemented.

  // Generate force trajectory
  MatrixXd force_samples = c3_solution->u_sol_.cast<double>();
  LcmTrajectory::Trajectory force_traj;
  force_traj.traj_name = "end_effector_force_target";
  force_traj.datatypes =
      std::vector<std::string>(force_samples.rows(), "double");
  force_traj.datapoints = force_samples;
  force_traj.time_vector = c3_solution->time_vector_.cast<double>();
  lcm_traj.AddTrajectory(force_traj.traj_name, force_traj);

  output->saved_traj = lcm_traj.GenerateLcmObject();
  output->utime = context.get_time() * 1e6;
}

void SamplingC3Controller::OutputC3SolutionCurrPlanObject(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* output) const {
  double t = context.get_discrete_state(plan_start_time_index_)[0];

  auto c3_solution = std::make_unique<C3Output::C3Solution>();
  c3_solution->x_sol_ = MatrixXf::Zero(n_q_ + n_v_, N_);
  c3_solution->lambda_sol_ = MatrixXf::Zero(n_lambda_, N_);
  c3_solution->u_sol_ = MatrixXf::Zero(n_u_, N_);
  c3_solution->time_vector_ = VectorXf::Zero(N_);
  auto z_sol = c3_curr_plan_->GetFullSolution();

  for (int i = 0; i < N_; i++) {
    c3_solution->time_vector_(i) = filtered_solve_time_ + t + i * dt_;
    c3_solution->x_sol_.col(i) = z_sol[i].segment(0, n_x_).cast<float>();
    c3_solution->lambda_sol_.col(i) =
        z_sol[i].segment(n_x_, n_lambda_).cast<float>();
    c3_solution->u_sol_.col(i) =
        z_sol[i].segment(n_x_ + n_lambda_, n_u_).cast<float>();
  }

  MatrixXd knots = MatrixXd::Zero(6, N_);
  knots.topRows(3) = c3_solution->x_sol_.middleRows(n_q_ - 3, 3).cast<double>();
  knots.bottomRows(3) =
      c3_solution->x_sol_.middleRows(n_q_ + n_v_ - 3, 3).cast<double>();
  LcmTrajectory::Trajectory object_traj;
  object_traj.traj_name = "object_position_target";
  object_traj.datatypes = std::vector<std::string>(knots.rows(), "double");
  object_traj.datapoints = knots;
  object_traj.time_vector = c3_solution->time_vector_.cast<double>();
  LcmTrajectory lcm_traj({object_traj}, {"object_position_target"},
                         "object_target", "object_target", false);

  LcmTrajectory::Trajectory object_orientation_traj;
  // first 3 rows are rpy, last 3 rows are angular velocity
  MatrixXd orientation_samples = MatrixXd::Zero(4, N_);
  orientation_samples =
      c3_solution->x_sol_.middleRows(n_q_ - 7, 4).cast<double>();
  object_orientation_traj.traj_name = "object_orientation_target";
  object_orientation_traj.datatypes =
      std::vector<std::string>(orientation_samples.rows(), "double");
  object_orientation_traj.datapoints = orientation_samples;
  object_orientation_traj.time_vector =
      c3_solution->time_vector_.cast<double>();
  lcm_traj.AddTrajectory(object_orientation_traj.traj_name,
                         object_orientation_traj);

  output->saved_traj = lcm_traj.GenerateLcmObject();
  output->utime = context.get_time() * 1e6;
}

void SamplingC3Controller::OutputC3SolutionCurrPlan(
    const drake::systems::Context<double>& context,
    C3Output::C3Solution* c3_solution) const {
  double t = context.get_discrete_state(plan_start_time_index_)[0];

  auto z_sol = c3_curr_plan_->GetFullSolution();
  for (int i = 0; i < N_; i++) {
    c3_solution->time_vector_(i) = filtered_solve_time_ + t + i * dt_;
    c3_solution->x_sol_.col(i) = z_sol[i].segment(0, n_x_).cast<float>();
    c3_solution->lambda_sol_.col(i) =
        z_sol[i].segment(n_x_, n_lambda_).cast<float>();
    c3_solution->u_sol_.col(i) =
        z_sol[i].segment(n_x_ + n_lambda_, n_u_).cast<float>();
  }
}

void SamplingC3Controller::OutputC3IntermediatesCurrPlan(
    const drake::systems::Context<double>& context,
    C3Output::C3Intermediates* c3_intermediates) const {
  double t = context.get_discrete_state(plan_start_time_index_)[0] +
             filtered_solve_time_;
  auto z_sol_curr_plan = c3_curr_plan_->GetFullSolution();
  auto delta_curr_plan = c3_curr_plan_->GetDualDeltaSolution();
  auto w_curr_plan = c3_curr_plan_->GetDualWSolution();

  for (int i = 0; i < N_; i++) {
    c3_intermediates->time_vector_(i) = t + i * dt_;
    c3_intermediates->z_.col(i) = z_sol_curr_plan[i].cast<float>();
    c3_intermediates->w_.col(i) = w_curr_plan[i].cast<float>();
    c3_intermediates->delta_.col(i) = delta_curr_plan[i].cast<float>();
  }
}

void SamplingC3Controller::OutputLCSContactJacobianCurrPlan(
    const drake::systems::Context<double>& context,
    std::pair<Eigen::MatrixXd, std::vector<Eigen::VectorXd>>*
        lcs_contact_jacobian) const {
  const TimestampedVector<double>* lcs_x =
      (TimestampedVector<double>*)this->EvalVectorInput(context,
                                                        lcs_state_input_port_);

  UpdateContext(n_q_, n_v_, n_u_, plant_, context_, plant_ad_, context_ad_,
                lcs_x->get_data());

  C3Options c3_options_for_curr_location = sampling_c3_options_.GetC3Options(
      crossed_cost_switching_threshold_,
      sampling_c3_options_.num_contacts_index_for_curr_location);

  // Preprocessing the contact pairs
  vector<SortedPair<GeometryId>> resolved_contact_pairs;
  resolved_contact_pairs = LCSFactory::PreProcessor(
      plant_, *context_, contact_pairs_,
      sampling_c3_options_
          .resolve_contacts_to_list[sampling_c3_options_.num_contacts_index],
      c3_options_for_curr_location.num_friction_directions,
      c3_options_for_curr_location.num_contacts, verbose_);

  // print size of resolved_contact_pairs
  *lcs_contact_jacobian = LCSFactory::ComputeContactJacobian(
      plant_, *context_, resolved_contact_pairs,
      c3_options_for_curr_location.num_friction_directions,
      c3_options_for_curr_location.mu, contact_model_);
}

// Output port handlers for best sample location
void SamplingC3Controller::OutputC3SolutionBestPlanActor(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* output) const {
  double t = context.get_discrete_state(plan_start_time_index_)[0];

  auto z_sol = c3_best_plan_->GetFullSolution();
  auto c3_solution = std::make_unique<C3Output::C3Solution>();
  c3_solution->x_sol_ = MatrixXf::Zero(n_q_ + n_v_, N_);
  c3_solution->lambda_sol_ = MatrixXf::Zero(n_lambda_, N_);
  c3_solution->u_sol_ = MatrixXf::Zero(n_u_, N_);
  c3_solution->time_vector_ = VectorXf::Zero(N_);

  for (int i = 0; i < N_; i++) {
    c3_solution->time_vector_(i) = filtered_solve_time_ + t + i * dt_;
    c3_solution->x_sol_.col(i) = z_sol[i].segment(0, n_x_).cast<float>();
    c3_solution->lambda_sol_.col(i) =
        z_sol[i].segment(n_x_, n_lambda_).cast<float>();
    c3_solution->u_sol_.col(i) =
        z_sol[i].segment(n_x_ + n_lambda_, n_u_).cast<float>();
  }

  for (int i = 0; i < N_; i++) {
    c3_solution->time_vector_(i) = filtered_solve_time_ + t + i * dt_;
    c3_solution->x_sol_.col(i) = z_sol[i].segment(0, n_x_).cast<float>();
    c3_solution->lambda_sol_.col(i) =
        z_sol[i].segment(n_x_, n_lambda_).cast<float>();
    c3_solution->u_sol_.col(i) =
        z_sol[i].segment(n_x_ + n_lambda_, n_u_).cast<float>();
  }

  MatrixXd knots = MatrixXd::Zero(6, N_);
  knots.topRows(3) = c3_solution->x_sol_.topRows(3).cast<double>();
  knots.bottomRows(3) =
      c3_solution->x_sol_.bottomRows(n_v_).topRows(3).cast<double>();

  LcmTrajectory::Trajectory end_effector_traj;
  end_effector_traj.traj_name = "end_effector_position_target";
  end_effector_traj.datatypes =
      std::vector<std::string>(knots.rows(), "double");
  end_effector_traj.datapoints = knots;
  end_effector_traj.time_vector = c3_solution->time_vector_.cast<double>();
  LcmTrajectory lcm_traj({end_effector_traj}, {"end_effector_position_target"},
                         "end_effector_position_target",
                         "end_effector_position_target", false);

  // NOTE: End effector orientation functionality is not implemented.

  // Generate force trajectory
  MatrixXd force_samples = c3_solution->u_sol_.cast<double>();
  LcmTrajectory::Trajectory force_traj;
  force_traj.traj_name = "end_effector_force_target";
  force_traj.datatypes =
      std::vector<std::string>(force_samples.rows(), "double");
  force_traj.datapoints = force_samples;
  force_traj.time_vector = c3_solution->time_vector_.cast<double>();
  lcm_traj.AddTrajectory(force_traj.traj_name, force_traj);

  output->saved_traj = lcm_traj.GenerateLcmObject();
  output->utime = context.get_time() * 1e6;
}

void SamplingC3Controller::OutputC3SolutionBestPlanObject(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* output) const {
  double t = context.get_discrete_state(plan_start_time_index_)[0];

  auto z_sol = c3_best_plan_->GetFullSolution();
  auto c3_solution = std::make_unique<C3Output::C3Solution>();
  c3_solution->x_sol_ = MatrixXf::Zero(n_q_ + n_v_, N_);
  c3_solution->lambda_sol_ = MatrixXf::Zero(n_lambda_, N_);
  c3_solution->u_sol_ = MatrixXf::Zero(n_u_, N_);
  c3_solution->time_vector_ = VectorXf::Zero(N_);

  for (int i = 0; i < N_; i++) {
    c3_solution->time_vector_(i) = filtered_solve_time_ + t + i * dt_;
    c3_solution->x_sol_.col(i) = z_sol[i].segment(0, n_x_).cast<float>();
    c3_solution->lambda_sol_.col(i) =
        z_sol[i].segment(n_x_, n_lambda_).cast<float>();
    c3_solution->u_sol_.col(i) =
        z_sol[i].segment(n_x_ + n_lambda_, n_u_).cast<float>();
  }

  MatrixXd knots = MatrixXd::Zero(6, N_);
  knots.topRows(3) = c3_solution->x_sol_.middleRows(n_q_ - 3, 3).cast<double>();
  knots.bottomRows(3) =
      c3_solution->x_sol_.middleRows(n_q_ + n_v_ - 3, 3).cast<double>();
  LcmTrajectory::Trajectory object_traj;
  object_traj.traj_name = "object_position_target";
  object_traj.datatypes = std::vector<std::string>(knots.rows(), "double");
  object_traj.datapoints = knots;
  object_traj.time_vector = c3_solution->time_vector_.cast<double>();
  LcmTrajectory lcm_traj({object_traj}, {"object_position_target"},
                         "object_target", "object_target", false);

  LcmTrajectory::Trajectory object_orientation_traj;
  // first 3 rows are rpy, last 3 rows are angular velocity
  MatrixXd orientation_samples = MatrixXd::Zero(4, N_);
  orientation_samples =
      c3_solution->x_sol_.middleRows(n_q_ - 7, 4).cast<double>();
  object_orientation_traj.traj_name = "object_orientation_target";
  object_orientation_traj.datatypes =
      std::vector<std::string>(orientation_samples.rows(), "double");
  object_orientation_traj.datapoints = orientation_samples;
  object_orientation_traj.time_vector =
      c3_solution->time_vector_.cast<double>();
  lcm_traj.AddTrajectory(object_orientation_traj.traj_name,
                         object_orientation_traj);

  output->saved_traj = lcm_traj.GenerateLcmObject();
  output->utime = context.get_time() * 1e6;
}

void SamplingC3Controller::OutputC3SolutionBestPlan(
    const drake::systems::Context<double>& context,
    C3Output::C3Solution* c3_solution) const {
  double t = context.get_discrete_state(plan_start_time_index_)[0];

  auto z_sol = c3_best_plan_->GetFullSolution();
  for (int i = 0; i < N_; i++) {
    c3_solution->time_vector_(i) = filtered_solve_time_ + t + i * dt_;
    c3_solution->x_sol_.col(i) = z_sol[i].segment(0, n_x_).cast<float>();
    c3_solution->lambda_sol_.col(i) =
        z_sol[i].segment(n_x_, n_lambda_).cast<float>();
    c3_solution->u_sol_.col(i) =
        z_sol[i].segment(n_x_ + n_lambda_, n_u_).cast<float>();
  }
}

void SamplingC3Controller::OutputC3IntermediatesBestPlan(
    const drake::systems::Context<double>& context,
    C3Output::C3Intermediates* c3_intermediates) const {
  double t = context.get_discrete_state(plan_start_time_index_)[0] +
             filtered_solve_time_;
  auto z_sol_best_plan = c3_best_plan_->GetFullSolution();
  auto delta_best_plan = c3_best_plan_->GetDualDeltaSolution();
  auto w_best_plan = c3_best_plan_->GetDualWSolution();

  for (int i = 0; i < N_; i++) {
    c3_intermediates->time_vector_(i) = t + i * dt_;
    c3_intermediates->z_.col(i) = z_sol_best_plan[i].cast<float>();
    c3_intermediates->w_.col(i) = w_best_plan[i].cast<float>();
    c3_intermediates->delta_.col(i) = delta_best_plan[i].cast<float>();
  }
}

void SamplingC3Controller::OutputLCSContactJacobianBestPlan(
    const drake::systems::Context<double>& context,
    std::pair<Eigen::MatrixXd, std::vector<Eigen::VectorXd>>*
        lcs_contact_jacobian) const {
  const TimestampedVector<double>* lcs_x =
      (TimestampedVector<double>*)this->EvalVectorInput(context,
                                                        lcs_state_input_port_);

  // Linearize about state with end effector in sample location.
  VectorXd x_sample = lcs_x->get_data();
  x_sample.head(3) = all_sample_locations_[best_sample_index_];
  UpdateContext(n_q_, n_v_, n_u_, plant_, context_, plant_ad_, context_ad_,
                x_sample);

  C3Options c3_options = sampling_c3_options_.GetC3Options(
      crossed_cost_switching_threshold_,
      sampling_c3_options_.num_contacts_index);
  // Preprocessing the contact pairs
  vector<SortedPair<GeometryId>> resolved_contact_pairs;
  resolved_contact_pairs = LCSFactory::PreProcessor(
      plant_, *context_, contact_pairs_,
      sampling_c3_options_
          .resolve_contacts_to_list[sampling_c3_options_.num_contacts_index],
      c3_options.num_friction_directions, c3_options.num_contacts, verbose_);

  *lcs_contact_jacobian = LCSFactory::ComputeContactJacobian(
      plant_, *context_, resolved_contact_pairs,
      c3_options.num_friction_directions, c3_options.mu, contact_model_);
  // Revert the context.
  UpdateContext(n_q_, n_v_, n_u_, plant_, context_, plant_ad_, context_ad_,
                lcs_x->get_data());
}

// Output port handlers for executing C3 and repositioning ports
void SamplingC3Controller::OutputC3TrajExecuteActor(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* output_c3_execution_lcm_traj) const {
  // Create a matrix containing the tracking trajectory including position
  LcmTrajectory::Trajectory end_effector_traj =
      c3_execution_lcm_traj_.GetTrajectory("end_effector_position_target");
  DRAKE_DEMAND(end_effector_traj.datapoints.rows() == 3);
  LcmTrajectory lcm_traj({end_effector_traj}, {"end_effector_position_target"},
                         "end_effector_position_target",
                         "end_effector_position_target", false);
  // NOTE: End effector orientation functionality is not implemented.

  // TODO: Might need to add a force trajectory that is non-zero for the
  // downstream osc to track.
  // TODO : ARE WE ACTUALLY TRACKING THE FORCE?
  // TODO: The 5 here is the hardcoded planning horizon.
  MatrixXd force_samples = MatrixXd::Zero(3, 5);
  LcmTrajectory::Trajectory force_traj =
      c3_execution_lcm_traj_.GetTrajectory("end_effector_force_target");
  lcm_traj.AddTrajectory(force_traj.traj_name, force_traj);

  output_c3_execution_lcm_traj->saved_traj = lcm_traj.GenerateLcmObject();
  output_c3_execution_lcm_traj->utime = context.get_time() * 1e6;
}

void SamplingC3Controller::OutputC3TrajExecuteObject(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* output_c3_execution_lcm_traj) const {
  // Create a matrix containing the tracking trajectory including position
  LcmTrajectory::Trajectory object_traj =
      c3_execution_lcm_traj_.GetTrajectory("object_position_target");
  LcmTrajectory lcm_traj({object_traj}, {"object_position_target"},
                         "object_target", "object_target", false);

  LcmTrajectory::Trajectory object_orientation_traj =
      c3_execution_lcm_traj_.GetTrajectory("object_orientation_target");
  lcm_traj.AddTrajectory(object_orientation_traj.traj_name,
                         object_orientation_traj);

  output_c3_execution_lcm_traj->saved_traj = lcm_traj.GenerateLcmObject();
  output_c3_execution_lcm_traj->utime = context.get_time() * 1e6;
}

void SamplingC3Controller::OutputReposTrajExecuteActor(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* output_repos_execution_lcm_traj)
    const {
  // *output_repos_execution_lcm_traj = repos_execution_lcm_traj_;

  // Create a matrix containing the tracking trajectory including position
  LcmTrajectory::Trajectory end_effector_traj =
      repos_execution_lcm_traj_.GetTrajectory("end_effector_position_target");
  DRAKE_DEMAND(end_effector_traj.datapoints.rows() == 3);
  LcmTrajectory lcm_traj({end_effector_traj}, {"end_effector_position_target"},
                         "end_effector_position_target",
                         "end_effector_position_target", false);
  // NOTE: End effector orientation functionality is not implemented.

  // TODO: Might need to add a force trajectory that is non-zero for the
  // downstream osc to track.
  // TODO : ARE WE ACTUALLY TRACKING THE FORCE?
  // TODO: The 5 here is the hardcoded planning horizon.
  MatrixXd force_samples = MatrixXd::Zero(3, 5);
  LcmTrajectory::Trajectory force_traj =
      repos_execution_lcm_traj_.GetTrajectory("end_effector_force_target");
  lcm_traj.AddTrajectory(force_traj.traj_name, force_traj);

  output_repos_execution_lcm_traj->saved_traj = lcm_traj.GenerateLcmObject();
  output_repos_execution_lcm_traj->utime = context.get_time() * 1e6;
}

void SamplingC3Controller::OutputReposTrajExecuteObject(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* output_repos_execution_lcm_traj)
    const {
  // *output_repos_execution_lcm_traj = repos_execution_lcm_traj_;

  LcmTrajectory::Trajectory object_traj =
      repos_execution_lcm_traj_.GetTrajectory("object_position_target");
  LcmTrajectory lcm_traj({object_traj}, {"object_position_target"},
                         "object_target", "object_target", false);

  LcmTrajectory::Trajectory object_orientation_traj =
      repos_execution_lcm_traj_.GetTrajectory("object_orientation_target");
  lcm_traj.AddTrajectory(object_orientation_traj.traj_name,
                         object_orientation_traj);

  output_repos_execution_lcm_traj->saved_traj = lcm_traj.GenerateLcmObject();
  output_repos_execution_lcm_traj->utime = context.get_time() * 1e6;
}

void SamplingC3Controller::OutputTrajExecuteActor(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* output_execution_lcm_traj) const {
  LcmTrajectory execution_lcm_traj;
  if (is_doing_c3_) {
    execution_lcm_traj = c3_execution_lcm_traj_;
  } else {
    execution_lcm_traj = repos_execution_lcm_traj_;
  }

  // Create a matrix containing the tracking trajectory including position
  LcmTrajectory::Trajectory end_effector_traj =
      execution_lcm_traj.GetTrajectory("end_effector_position_target");
  DRAKE_DEMAND(end_effector_traj.datapoints.rows() == 3);
  LcmTrajectory lcm_traj({end_effector_traj}, {"end_effector_position_target"},
                         "end_effector_position_target",
                         "end_effector_position_target", false);
  // NOTE: End effector orientation functionality is not implemented.

  // TODO: Might need to add a force trajectory that is non-zero for the
  // downstream osc to track.
  // TODO : ARE WE ACTUALLY TRACKING THE FORCE?
  // TODO: The 5 here is the hardcoded planning horizon.
  MatrixXd force_samples = MatrixXd::Zero(3, 5);
  LcmTrajectory::Trajectory force_traj =
      execution_lcm_traj.GetTrajectory("end_effector_force_target");
  lcm_traj.AddTrajectory(force_traj.traj_name, force_traj);

  output_execution_lcm_traj->saved_traj = lcm_traj.GenerateLcmObject();
  output_execution_lcm_traj->utime = context.get_time() * 1e6;
}

void SamplingC3Controller::OutputTrajExecuteObject(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* output_execution_lcm_traj) const {
  LcmTrajectory execution_lcm_traj;
  if (is_doing_c3_) {
    execution_lcm_traj = c3_execution_lcm_traj_;
  } else {
    execution_lcm_traj = repos_execution_lcm_traj_;
  }

  LcmTrajectory::Trajectory object_traj =
      execution_lcm_traj.GetTrajectory("object_position_target");
  LcmTrajectory lcm_traj({object_traj}, {"object_position_target"},
                         "object_target", "object_target", false);

  LcmTrajectory::Trajectory object_orientation_traj =
      execution_lcm_traj.GetTrajectory("object_orientation_target");
  lcm_traj.AddTrajectory(object_orientation_traj.traj_name,
                         object_orientation_traj);

  output_execution_lcm_traj->saved_traj = lcm_traj.GenerateLcmObject();
  output_execution_lcm_traj->utime = context.get_time() * 1e6;
}

void SamplingC3Controller::OutputIsC3Mode(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* output) const {
  Eigen::VectorXd vec = VectorXd::Constant(1, is_doing_c3_);
  // convert the above into a matrix
  Eigen::MatrixXd c3_mode_data = Eigen::MatrixXd::Zero(1, 1);
  Eigen::VectorXd timestamp = Eigen::VectorXd::Zero(1);

  // Reading the boolean value into the matrix.
  c3_mode_data(0, 0) = vec(0);
  // This timestamp corresponds to the trajectory object.
  timestamp(0) = 0.0;

  LcmTrajectory::Trajectory c3_mode;

  c3_mode.traj_name = "is_c3_mode";
  c3_mode.datatypes = std::vector<std::string>(1, "bool");
  c3_mode.datapoints = c3_mode_data;
  c3_mode.time_vector = timestamp.cast<double>();

  LcmTrajectory c3_mode_traj({c3_mode}, {"is_c3_mode"}, "is_c3_mode",
                             "is_c3_mode", false);

  // Output the mode as an lcm message
  output->saved_traj = c3_mode_traj.GenerateLcmObject();
  output->utime = context.get_time() * 1e6;
}

// Output port handler for Dynamically feasible trajectory used for cost
// computation. This will directy output an lcmt_timestamped_saved_traj
// object with the dynamically feasible trajectory.
void SamplingC3Controller::OutputDynamicallyFeasibleCurrPlanActor(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* dynamically_feasible_curr_plan_actor)
    const {
  std::vector<Eigen::VectorXd> dynamically_feasible_traj =
      std::vector<Eigen::VectorXd>(N_ + 1, VectorXd::Zero(n_x_));
  // Create a matrix of dynamically feasible plan including positions and
  // orientations.
  for (int i = 0; i < N_ + 1; i++) {
    dynamically_feasible_traj[i]
        << all_sample_dynamically_feasible_plans_.at(CURRENT_LOCATION_INDEX)[i];
  }
  Eigen::MatrixXd knots =
      Eigen::MatrixXd::Zero(3, dynamically_feasible_traj.size());
  Eigen::VectorXd timestamps =
      Eigen::VectorXd::Zero(dynamically_feasible_traj.size());
  for (int i = 0; i < dynamically_feasible_traj.size(); i++) {
    knots.col(i) = dynamically_feasible_traj[i].head(3);
    timestamps(i) = i;
  }
  LcmTrajectory::Trajectory ee_traj;
  // position trajectory
  Eigen::MatrixXd position_samples = Eigen::MatrixXd::Zero(3, N_ + 1);
  position_samples = knots.bottomRows(3);
  ee_traj.traj_name = "ee_position_target";
  ee_traj.datatypes =
      std::vector<std::string>(position_samples.rows(), "double");
  ee_traj.datapoints = position_samples;
  ee_traj.time_vector = timestamps.cast<double>();

  LcmTrajectory ee_traj_lcm({ee_traj}, {"ee_position_target"},
                            "ee_position_target", "ee_position_target", false);

  // Output the trajectory as an lcm message
  dynamically_feasible_curr_plan_actor->saved_traj =
      ee_traj_lcm.GenerateLcmObject();
  dynamically_feasible_curr_plan_actor->utime = context.get_time() * 1e6;
}

// Output port handler for Dynamically feasible trajectory used for cost
// computation.
void SamplingC3Controller::OutputDynamicallyFeasibleCurrPlanObject(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* dynamically_feasible_curr_plan_object)
    const {
  std::vector<Eigen::VectorXd> dynamically_feasible_traj =
      std::vector<Eigen::VectorXd>(N_ + 1, VectorXd::Zero(n_x_));
  // Extract object pose only from the dynamically feasible plan for each time
  // step.
  for (int i = 0; i < N_ + 1; i++) {
    dynamically_feasible_traj[i]
        << all_sample_dynamically_feasible_plans_.at(CURRENT_LOCATION_INDEX)[i];
  }

  // Create a matrix containing the dynamically feasible plan including position
  // and orientation
  // TODO: Change the 7 to read the number of states from the
  // dynamically_feasible_plan
  Eigen::MatrixXd knots =
      Eigen::MatrixXd::Zero(7, dynamically_feasible_traj.size());
  Eigen::VectorXd timestamps =
      Eigen::VectorXd::Zero(dynamically_feasible_traj.size());
  for (int i = 0; i < dynamically_feasible_traj.size(); i++) {
    knots.col(i) = dynamically_feasible_traj[i].segment(3, 7);
    timestamps(i) = i;
  }

  LcmTrajectory::Trajectory object_traj;
  // position trajectory
  Eigen::MatrixXd position_samples = Eigen::MatrixXd::Zero(3, 6);
  position_samples = knots.bottomRows(3);
  object_traj.traj_name = "object_position_target";
  object_traj.datatypes =
      std::vector<std::string>(position_samples.rows(), "double");
  object_traj.datapoints = position_samples;
  object_traj.time_vector = timestamps.cast<double>();
  LcmTrajectory lcm_traj({object_traj}, {"object_position_target"},
                         "object_target", "object_target", false);

  LcmTrajectory::Trajectory object_orientation_traj;
  // orientation as quaternion
  Eigen::MatrixXd orientation_samples = Eigen::MatrixXd::Zero(4, 6);
  orientation_samples = knots.topRows(4);
  object_orientation_traj.traj_name = "object_orientation_target";
  object_orientation_traj.datatypes =
      std::vector<std::string>(orientation_samples.rows(), "double");
  object_orientation_traj.datapoints = orientation_samples;
  object_orientation_traj.time_vector = timestamps.cast<double>();
  lcm_traj.AddTrajectory(object_orientation_traj.traj_name,
                         object_orientation_traj);

  dynamically_feasible_curr_plan_object->saved_traj =
      lcm_traj.GenerateLcmObject();
  dynamically_feasible_curr_plan_object->utime = context.get_time() * 1e6;
}

void SamplingC3Controller::OutputDynamicallyFeasibleBestPlanActor(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* dynamically_feasible_best_plan)
    const {
  std::vector<Eigen::VectorXd> dynamically_feasible_traj =
      std::vector<Eigen::VectorXd>(N_ + 1, VectorXd::Zero(n_x_));
  // Extract object pose only from the dynamically feasible plan for each time
  // step.
  for (int i = 0; i < N_ + 1; i++) {
    dynamically_feasible_traj[i]
        << all_sample_dynamically_feasible_plans_.at(best_sample_index_)[i];
  }

  Eigen::MatrixXd knots =
      Eigen::MatrixXd::Zero(3, dynamically_feasible_traj.size());
  Eigen::VectorXd timestamps =
      Eigen::VectorXd::Zero(dynamically_feasible_traj.size());
  for (int i = 0; i < dynamically_feasible_traj.size(); i++) {
    knots.col(i) = dynamically_feasible_traj[i].head(3);
    timestamps(i) = i;
  }

  LcmTrajectory::Trajectory ee_traj;
  // position trajectory
  Eigen::MatrixXd position_samples = Eigen::MatrixXd::Zero(3, 6);
  position_samples = knots.bottomRows(3);
  ee_traj.traj_name = "ee_position_target";
  ee_traj.datatypes =
      std::vector<std::string>(position_samples.rows(), "double");
  ee_traj.datapoints = position_samples;
  ee_traj.time_vector = timestamps.cast<double>();

  LcmTrajectory ee_traj_lcm({ee_traj}, {"ee_position_target"},
                            "ee_position_target", "ee_position_target", false);

  // Output the trajectory as an lcm message
  dynamically_feasible_best_plan->saved_traj = ee_traj_lcm.GenerateLcmObject();
  dynamically_feasible_best_plan->utime = context.get_time() * 1e6;
}

void SamplingC3Controller::OutputDynamicallyFeasibleBestPlanObject(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* dynamically_feasible_best_plan)
    const {
  std::vector<Eigen::VectorXd> dynamically_feasible_traj =
      std::vector<Eigen::VectorXd>(N_ + 1, VectorXd::Zero(n_x_));
  // Extract object pose only from the dynamically feasible plan for each time
  // step.
  for (int i = 0; i < N_ + 1; i++) {
    dynamically_feasible_traj[i]
        << all_sample_dynamically_feasible_plans_.at(best_sample_index_)[i];
  }

  // Create a matrix containing the dynamically feasible plan including position
  // and orientation
  // TODO: Change the 7 to read the number of states from the
  // dynamically_feasible_plan
  Eigen::MatrixXd knots =
      Eigen::MatrixXd::Zero(7, dynamically_feasible_traj.size());
  Eigen::VectorXd timestamps =
      Eigen::VectorXd::Zero(dynamically_feasible_traj.size());
  for (int i = 0; i < dynamically_feasible_traj.size(); i++) {
    knots.col(i) = dynamically_feasible_traj[i].segment(3, 7);
    timestamps(i) = i;
  }

  LcmTrajectory::Trajectory object_traj;
  // position trajectory
  Eigen::MatrixXd position_samples = Eigen::MatrixXd::Zero(3, 6);
  position_samples = knots.bottomRows(3);
  object_traj.traj_name = "object_position_target";
  object_traj.datatypes =
      std::vector<std::string>(position_samples.rows(), "double");
  object_traj.datapoints = position_samples;
  object_traj.time_vector = timestamps.cast<double>();
  LcmTrajectory lcm_traj({object_traj}, {"object_position_target"},
                         "object_target", "object_target", false);

  LcmTrajectory::Trajectory object_orientation_traj;
  // orientation as quaternion
  Eigen::MatrixXd orientation_samples = Eigen::MatrixXd::Zero(4, 6);
  orientation_samples = knots.topRows(4);
  object_orientation_traj.traj_name = "object_orientation_target";
  object_orientation_traj.datatypes =
      std::vector<std::string>(orientation_samples.rows(), "double");
  object_orientation_traj.datapoints = orientation_samples;
  object_orientation_traj.time_vector = timestamps.cast<double>();
  lcm_traj.AddTrajectory(object_orientation_traj.traj_name,
                         object_orientation_traj);

  dynamically_feasible_best_plan->saved_traj = lcm_traj.GenerateLcmObject();
  dynamically_feasible_best_plan->utime = context.get_time() * 1e6;
}

// Output port handlers for sample-related ports
void SamplingC3Controller::OutputAllSampleLocations(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* output_all_sample_locations) const {
  // Output all sample locations including current location.
  std::vector<Eigen::Vector3d> sample_locations = std::vector<Eigen::Vector3d>(
      all_sample_locations_.begin(), all_sample_locations_.end());
  // Padd with zeros to make sure the size is max_num_samples_ + 1 for the
  // visualizer.
  while (sample_locations.size() < max_num_samples_ + 1) {
    sample_locations.push_back(Vector3d::Zero());
  }

  // Create a matrix of sample locations
  Eigen::MatrixXd sample_datapoints =
      Eigen::MatrixXd::Zero(3, sample_locations.size());
  Eigen::VectorXd timestamps = Eigen::VectorXd::Zero(sample_locations.size());

  for (int i = 0; i < sample_locations.size(); i++) {
    sample_datapoints.col(i) = sample_locations[i];
    timestamps(i) = i;
  }

  LcmTrajectory::Trajectory sample_positions;
  sample_positions.traj_name = "sample_locations";
  sample_positions.datatypes = std::vector<std::string>(3, "double");
  sample_positions.datapoints = sample_datapoints;
  sample_positions.time_vector = timestamps.cast<double>();

  LcmTrajectory sample_traj({sample_positions}, {"sample_locations"},
                            "sample_locations", "sample_locations", false);

  // Output the sample locations
  output_all_sample_locations->saved_traj = sample_traj.GenerateLcmObject();
  output_all_sample_locations->utime = context.get_time() * 1e6;
  // *all_sample_locations = sample_locations;
}

void SamplingC3Controller::OutputAllSampleCosts(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* output_all_sample_costs) const {
  // Create a matrix of sample costs
  Eigen::MatrixXd cost_datapoints =
      Eigen::MatrixXd::Zero(1, all_sample_costs_.size());
  Eigen::VectorXd timestamps = Eigen::VectorXd::Zero(all_sample_costs_.size());

  for (int i = 0; i < all_sample_costs_.size(); i++) {
    cost_datapoints(0, i) = all_sample_costs_[i];
    // This is a dummy timestamp for the sample costs.
    timestamps(i) = i * dt_;
  }

  LcmTrajectory::Trajectory sample_costs_traj;
  sample_costs_traj.traj_name = "sample_costs";
  sample_costs_traj.datatypes = std::vector<std::string>(1, "double");
  sample_costs_traj.datapoints = cost_datapoints;
  sample_costs_traj.time_vector = timestamps.cast<double>();

  LcmTrajectory cost_traj({sample_costs_traj}, {"sample_costs"}, "sample_costs",
                          "sample_costs", false);

  // Output the sample costs
  output_all_sample_costs->saved_traj = cost_traj.GenerateLcmObject();
  output_all_sample_costs->utime = context.get_time() * 1e6;

  if (verbose_) {
    std::cout << "All sample costs as per output port: " << std::endl;
    for (int i = 0; i < all_sample_costs_.size(); i++) {
      std::cout << all_sample_costs_[i] << std::endl;
    }
  }
}

void SamplingC3Controller::OutputCurrAndBestSampleCost(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* output_curr_and_best_sample_cost)
    const {
  // *curr_and_best_sample_cost = curr_and_best_sample_cost_;

  // Create a matrix of sample costs
  Eigen::MatrixXd cost_datapoints =
      Eigen::MatrixXd::Zero(1, curr_and_best_sample_cost_.size());
  Eigen::VectorXd timestamps =
      Eigen::VectorXd::Zero(curr_and_best_sample_cost_.size());

  for (int i = 0; i < curr_and_best_sample_cost_.size(); i++) {
    cost_datapoints(0, i) = curr_and_best_sample_cost_[i];
    // This is a dummy timestamp for the sample costs.
    timestamps(i) = i * dt_;
  }

  LcmTrajectory::Trajectory curr_and_best_sample_cost_traj;
  curr_and_best_sample_cost_traj.traj_name = "curr_and_best_sample_cost";
  curr_and_best_sample_cost_traj.datatypes =
      std::vector<std::string>(1, "double");
  curr_and_best_sample_cost_traj.datapoints = cost_datapoints;
  curr_and_best_sample_cost_traj.time_vector = timestamps.cast<double>();

  LcmTrajectory curr_and_best_cost_traj(
      {curr_and_best_sample_cost_traj}, {"curr_and_best_sample_cost"},
      "curr_and_best_sample_cost", "curr_and_best_sample_cost", false);

  // Output the sample costs
  output_curr_and_best_sample_cost->saved_traj =
      curr_and_best_cost_traj.GenerateLcmObject();
  output_curr_and_best_sample_cost->utime = context.get_time() * 1e6;
}

void SamplingC3Controller::OutputDebug(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_sampling_c3_debug* debug_msg) const {
  debug_msg->utime = context.get_time() * 1e6;

  debug_msg->is_c3_mode = is_doing_c3_;

  // Redundant radio things included in debug message for convenience.
  const auto& radio_out =
      this->EvalInputValue<dairlib::lcmt_radio_out>(context, radio_port_);
  debug_msg->is_teleop = radio_out->channel[14];  // 14 = teleop
  debug_msg->is_force_tracking =
      !radio_out->channel[11];                            // 11 = force tracking
                                                          //      disabled
  debug_msg->is_forced_into_c3 = radio_out->channel[12];  // 12 = forced into C3

  debug_msg->in_pose_tracking_mode = crossed_cost_switching_threshold_;

  debug_msg->mode_switch_reason = mode_switch_reason_;
  debug_msg->source_of_pursued_target = pursued_target_source_;

  debug_msg->detected_goal_changes = detected_goal_changes_;

  debug_msg->best_progress_steps_ago = best_progress_steps_ago_;
  debug_msg->lowest_cost = lowest_cost_;
  debug_msg->lowest_pos_and_rot_current_cost = lowest_pos_and_rot_current_cost_;
  debug_msg->lowest_position_error = lowest_position_error_;
  debug_msg->lowest_orientation_error = lowest_orientation_error_;

  debug_msg->current_pos_error = current_position_error_;
  debug_msg->current_rot_error = current_orientation_error_;
}

void SamplingC3Controller::OutputSampleBufferConfigurations(
    const drake::systems::Context<double>& context,
    Eigen::MatrixXd* sample_buffer_configurations) const {
  *sample_buffer_configurations = sample_buffer_;
}

void SamplingC3Controller::OutputSampleBufferCosts(
    const drake::systems::Context<double>& context,
    Eigen::VectorXd* sample_buffer_costs) const {
  *sample_buffer_costs = sample_costs_buffer_;
}

}  // namespace systems
}  // namespace dairlib
