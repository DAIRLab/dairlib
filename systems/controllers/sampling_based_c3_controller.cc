#include "sampling_based_c3_controller.h"
#include <iostream>

#include <omp.h>
#include <utility>
#include "dairlib/lcmt_radio_out.hpp"
#include "multibody/multibody_utils.h"
#include "external/drake/tools/install/libdrake/_virtual_includes/drake_shared_library/drake/multibody/plant/multibody_plant.h"
#include "solvers/c3_miqp.h"
#include "solvers/c3_qp.h"
#include "solvers/lcs.h"
#include "solvers/lcs_factory.h"
#include "solvers/lcs_factory_preprocessor.h"
#include "generate_samples.h"

namespace dairlib {

using drake::multibody::ModelInstanceIndex;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteValues;
using Eigen::MatrixXd;
using Eigen::MatrixXf;
using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::VectorXf;
using solvers::C3;
using solvers::C3MIQP;
using solvers::C3QP;
using solvers::LCS;
using solvers::LCSFactory;
using solvers::LCSFactoryPreProcessor;
using std::vector;
using systems::TimestampedVector;
using drake::multibody::MultibodyPlant;
using drake::SortedPair;
using drake::geometry::GeometryId;

namespace systems {

SamplingC3Controller::SamplingC3Controller(
    drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context,
    drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad,
    drake::systems::Context<drake::AutoDiffXd>* context_ad,
    const std::vector<std::vector<drake::SortedPair<drake::geometry::GeometryId>>>& contact_geoms,
    C3Options c3_options,
    SamplingC3SamplingParams sampling_params)
    : plant_(plant),
      context_(context),
      plant_ad_(plant_ad),
      context_ad_(context_ad),
      contact_pairs_(contact_geoms),
      c3_options_(std::move(c3_options)),
      sampling_params_(std::move(sampling_params)),
      G_(std::vector<MatrixXd>(c3_options_.N, c3_options_.G)),
      U_(std::vector<MatrixXd>(c3_options_.N, c3_options_.U)),
      N_(c3_options_.N) {
  this->set_name("sampling_c3_controller");

  double discount_factor = 1;
  for (int i = 0; i < N_; ++i) {
    Q_.push_back(discount_factor * c3_options_.Q);
    R_.push_back(discount_factor * c3_options_.R);
    discount_factor *= c3_options_.gamma;
  }
  Q_.push_back(discount_factor * c3_options_.Q);
  DRAKE_DEMAND(Q_.size() == N_ + 1);
  DRAKE_DEMAND(R_.size() == N_);

  n_q_ = plant_.num_positions();
  n_v_ = plant_.num_velocities();
  n_u_ = plant_.num_actuators();

  n_x_ = n_q_ + n_v_;
  dt_ = c3_options_.dt;
  solve_time_filter_constant_ = c3_options_.solve_time_filter_alpha;
  if (c3_options_.contact_model == "stewart_and_trinkle") {
    n_lambda_ =
        2 * c3_options_.num_contacts +
        2 * c3_options_.num_friction_directions * c3_options_.num_contacts;
  } else if (c3_options_.contact_model == "anitescu") {
    n_lambda_ =
        2 * c3_options_.num_friction_directions * c3_options_.num_contacts;
  } else {
    std::cerr << ("Unknown contact model") << std::endl;
    DRAKE_THROW_UNLESS(false);
  }

  // Creates placeholder lcs to construct base C3 problem
  // Placeholder LCS will have correct size as it's already determined by the
  // contact model
  auto lcs_placeholder = CreatePlaceholderLCS();
  auto x_desired_placeholder =
      std::vector<VectorXd>(N_ + 1, VectorXd::Zero(n_x_));
  if (c3_options_.projection_type == "MIQP") {
    c3_curr_plan_ = std::make_unique<C3MIQP>(lcs_placeholder,
                                        C3::CostMatrices(Q_, R_, G_, U_),
                                        x_desired_placeholder, c3_options_);
    c3_best_plan_ = std::make_unique<C3MIQP>(lcs_placeholder,
                                        C3::CostMatrices(Q_, R_, G_, U_),
                                        x_desired_placeholder, c3_options_);

  } else if (c3_options_.projection_type == "QP") {
    c3_curr_plan_ = std::make_unique<C3QP>(lcs_placeholder,
                                      C3::CostMatrices(Q_, R_, G_, U_),
                                      x_desired_placeholder, c3_options_);
    c3_best_plan_ = std::make_unique<C3QP>(lcs_placeholder,
                                      C3::CostMatrices(Q_, R_, G_, U_),
                                      x_desired_placeholder, c3_options_);

  } else {
    std::cerr << ("Unknown projection type") << std::endl;
    DRAKE_THROW_UNLESS(false);
  }

  c3_curr_plan_->SetOsqpSolverOptions(solver_options_);
  c3_best_plan_->SetOsqpSolverOptions(solver_options_);

  // Set actor bounds
  for (int i : vector<int>({0})) {
    Eigen::RowVectorXd A = VectorXd::Zero(n_x_);
    A(i) = 1.0;
    c3_curr_plan_->AddLinearConstraint(A, c3_options_.world_x_limits[0],
                                  c3_options_.world_x_limits[1], 1);
    c3_best_plan_->AddLinearConstraint(A, c3_options_.world_x_limits[0],
                                  c3_options_.world_x_limits[1], 1);
  }
  for (int i : vector<int>({1})) {
    Eigen::RowVectorXd A = VectorXd::Zero(n_x_);
    A(i) = 1.0;
    c3_curr_plan_->AddLinearConstraint(A, c3_options_.world_y_limits[0],
                                  c3_options_.world_y_limits[1], 1);
    c3_best_plan_->AddLinearConstraint(A, c3_options_.world_y_limits[0],
                                  c3_options_.world_y_limits[1], 1);
  }
  for (int i : vector<int>({2})) {
    Eigen::RowVectorXd A = VectorXd::Zero(n_x_);
    A(i) = 1.0;
    c3_curr_plan_->AddLinearConstraint(A, c3_options_.world_z_limits[0],
                                  c3_options_.world_z_limits[1], 1);
    c3_best_plan_->AddLinearConstraint(A, c3_options_.world_z_limits[0],
                                  c3_options_.world_z_limits[1], 1);
  }
  for (int i : vector<int>({0, 1})) {
    Eigen::RowVectorXd A = VectorXd::Zero(n_u_);
    A(i) = 1.0;
    c3_curr_plan_->AddLinearConstraint(A, c3_options_.u_horizontal_limits[0],
                                  c3_options_.u_horizontal_limits[1], 2);
    c3_best_plan_->AddLinearConstraint(A, c3_options_.u_horizontal_limits[0],
                                  c3_options_.u_horizontal_limits[1], 2);
  }
  for (int i : vector<int>({2})) {
    Eigen::RowVectorXd A = VectorXd::Zero(n_u_);
    A(i) = 1.0;
    c3_curr_plan_->AddLinearConstraint(A, c3_options_.u_vertical_limits[0],
                                  c3_options_.u_vertical_limits[1], 2);
    c3_best_plan_->AddLinearConstraint(A, c3_options_.u_vertical_limits[0],
                                  c3_options_.u_vertical_limits[1], 2);
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

  // Output ports.
  auto c3_solution = C3Output::C3Solution();
  c3_solution.x_sol_ = MatrixXf::Zero(n_q_ + n_v_, N_);
  c3_solution.lambda_sol_ = MatrixXf::Zero(n_lambda_, N_);
  c3_solution.u_sol_ = MatrixXf::Zero(n_u_, N_);
  c3_solution.time_vector_ = VectorXf::Zero(N_);
  auto c3_intermediates = C3Output::C3Intermediates();
  c3_intermediates.w_ = MatrixXf::Zero(n_x_ + n_lambda_ + n_u_, N_);
  c3_intermediates.delta_ = MatrixXf::Zero(n_x_ + n_lambda_ + n_u_, N_);
  c3_intermediates.time_vector_ = VectorXf::Zero(N_);
  auto lcs_contact_jacobian = std::pair(Eigen::MatrixXd(n_x_, n_lambda_),
                                        std::vector<Eigen::VectorXd>());
  int num_samples = std::max(sampling_params_.num_additional_samples_repos,
                             sampling_params_.num_additional_samples_c3);
  all_sample_locations_ = vector<Vector3d>(num_samples + 1, Vector3d::Zero());
  all_sample_costs_ = std::vector<double>(num_samples + 1, -1);
  LcmTrajectory lcm_traj = LcmTrajectory();

  // Current location plan output ports.
  c3_solution_curr_plan_port_ = this->DeclareAbstractOutputPort(
    "c3_solution_curr_plan", c3_solution,
    &SamplingC3Controller::OutputC3SolutionCurrPlan
  ).get_index();
  c3_intermediates_curr_plan_port_ = this->DeclareAbstractOutputPort(
    "c3_intermediates_curr_plan", c3_intermediates,
    &SamplingC3Controller::OutputC3IntermediatesCurrPlan
  ).get_index();
  lcs_contact_jacobian_curr_plan_port_ = this->DeclareAbstractOutputPort(
    "J_lcs_curr_plan, p_lcs_curr_plan", lcs_contact_jacobian,
    &SamplingC3Controller::OutputLCSContactJacobianCurrPlan
  ).get_index();

  // Best sample plan output ports.
  c3_solution_best_plan_port_ = this->DeclareAbstractOutputPort(
    "c3_solution_best_plan", c3_solution,
    &SamplingC3Controller::OutputC3SolutionBestPlan
  ).get_index();
  c3_intermediates_best_plan_port_ = this->DeclareAbstractOutputPort(
    "c3_intermediates_best_plan", c3_intermediates,
    &SamplingC3Controller::OutputC3IntermediatesBestPlan
  ).get_index();
  lcs_contact_jacobian_best_plan_port_ = this->DeclareAbstractOutputPort(
    "J_lcs_best_plan, p_lcs_best_plan", lcs_contact_jacobian,
    &SamplingC3Controller::OutputLCSContactJacobianBestPlan
  ).get_index();

  // Execution trajectory output ports.
  c3_traj_execute_port_ = this->DeclareAbstractOutputPort(
    "c3_traj_execute", lcm_traj, // TODO don't use class vars for port types
    &SamplingC3Controller::OutputC3TrajExecute
  ).get_index();
  repos_traj_execute_port_ = this->DeclareAbstractOutputPort(
    "repos_traj_execute", lcm_traj, // TODO don't use class vars for port types
    &SamplingC3Controller::OutputReposTrajExecute
  ).get_index();
  // NOTE: We reuse c3_execution_lcm_traj_ to set the right type for the port.
  traj_execute_port_ = this->DeclareAbstractOutputPort(
    "traj_execute", lcm_traj, // TODO don't use class vars for port types
    &SamplingC3Controller::OutputTrajExecute
  ).get_index();
  is_c3_mode_port_ = this->DeclareVectorOutputPort(
    "is_c3_mode", drake::systems::BasicVector<double>(1),
    &SamplingC3Controller::OutputIsC3Mode
  ).get_index();
  
  // Sample location related output ports.
  all_sample_locations_port_ = this->DeclareAbstractOutputPort(
    "all_sample_locations", vector<Vector3d>(num_samples + 1, Vector3d::Zero()), // TODO don't use class vars for port types
    &SamplingC3Controller::OutputAllSampleLocations
  ).get_index();
  all_sample_costs_port_ = this->DeclareAbstractOutputPort(
    "all_sample_costs", std::vector<double>(num_samples + 1, -1), // TODO don't use class vars for port types
    &SamplingC3Controller::OutputAllSampleCosts
  ).get_index();

  plan_start_time_index_ = DeclareDiscreteState(1);
  x_pred_curr_plan_ = VectorXd::Zero(n_x_);
  x_pred_best_plan_ = VectorXd::Zero(n_x_);

  DeclareForcedDiscreteUpdateEvent(&SamplingC3Controller::ComputePlan);

  // Set parallelization settings.
  omp_set_dynamic(0);           // Explicitly disable dynamic teams.
  omp_set_nested(1);            // Enable nested threading.
  if (c3_options_.num_outer_threads == 0) {
    // Interpret setting number of threads to zero as a request to use all
    // machine's threads.
    num_threads_to_use_ = omp_get_max_threads();
  }
  else {
    num_threads_to_use_ = c3_options_.num_outer_threads;
  }
}

LCS SamplingC3Controller::CreatePlaceholderLCS() const {
  MatrixXd A = MatrixXd::Zero(n_x_, n_x_);
  MatrixXd B = MatrixXd::Zero(n_x_, n_u_);
  VectorXd d = VectorXd::Zero(n_x_);
  MatrixXd D = MatrixXd::Zero(n_x_, n_lambda_);
  MatrixXd E = MatrixXd::Zero(n_lambda_, n_x_);
  MatrixXd F = MatrixXd::Zero(n_lambda_, n_lambda_);
  MatrixXd H = MatrixXd::Zero(n_lambda_, n_u_);
  VectorXd c = VectorXd::Zero(n_lambda_);
  return LCS(A, B, D, d, E, F, H, c, c3_options_.N, c3_options_.dt);
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
  const TimestampedVector<double>* lcs_x_curr =
      (TimestampedVector<double>*)this->EvalVectorInput(context,
                                                        lcs_state_input_port_);

  // Store the current LCS state.
  drake::VectorX<double> x_lcs_curr = lcs_x_curr->get_data();

  // Allow the x-box controller to set the state of the system.
  if (!radio_out->channel[14] && c3_options_.use_predicted_x0 && !x_pred_curr_plan_.isZero()) {
    x_lcs_curr[0] = std::clamp(
      x_pred_curr_plan_[0], x_lcs_curr[0] - 10 * dt_ * dt_,
      x_lcs_curr[0] + 10 * dt_ * dt_
    );
    x_lcs_curr[1] = std::clamp(
      x_pred_curr_plan_[1], x_lcs_curr[1] - 10 * dt_ * dt_,
      x_lcs_curr[1] + 10 * dt_ * dt_
    );
    x_lcs_curr[2] = std::clamp(
      x_pred_curr_plan_[2], x_lcs_curr[2] - 10 * dt_ * dt_,
      x_lcs_curr[2] + 10 * dt_ * dt_
    );
    x_lcs_curr[n_q_ + 0] = std::clamp(
      x_pred_curr_plan_[n_q_ + 0], x_lcs_curr[n_q_ + 0] - 10 * dt_,
      x_lcs_curr[n_q_ + 0] + 10 * dt_
    );
    x_lcs_curr[n_q_ + 1] = std::clamp(
      x_pred_curr_plan_[n_q_ + 1], x_lcs_curr[n_q_ + 1] - 10 * dt_,
      x_lcs_curr[n_q_ + 1] + 10 * dt_
    );
    x_lcs_curr[n_q_ + 2] = std::clamp(
      x_pred_curr_plan_[n_q_ + 2], x_lcs_curr[n_q_ + 2] - 10 * dt_,
      x_lcs_curr[n_q_ + 2] + 10 * dt_
    );
  }

  discrete_state->get_mutable_value(plan_start_time_index_)[0] =
      lcs_x_curr->get_timestamp();

  std::vector<VectorXd> x_desired =
      std::vector<VectorXd>(N_ + 1, x_lcs_des.value());

  // Force Checking of Workspace Limits
  DRAKE_DEMAND(lcs_x_curr->get_data()[0] >
               c3_options_.world_x_limits[0] - c3_options_.workspace_margins);
  DRAKE_DEMAND(lcs_x_curr->get_data()[0] <
               c3_options_.world_x_limits[1] + c3_options_.workspace_margins);
  DRAKE_DEMAND(lcs_x_curr->get_data()[1] >
               c3_options_.world_y_limits[0] - c3_options_.workspace_margins);
  DRAKE_DEMAND(lcs_x_curr->get_data()[1] <
               c3_options_.world_y_limits[1] + c3_options_.workspace_margins);
  DRAKE_DEMAND(lcs_x_curr->get_data()[2] >
               c3_options_.world_z_limits[0] - c3_options_.workspace_margins);
  DRAKE_DEMAND(lcs_x_curr->get_data()[2] <
               c3_options_.world_z_limits[1] + c3_options_.workspace_margins);

  // Generate multiple samples and include current location as first item.
  std::vector<Eigen::VectorXd> candidate_states = generate_sample_states(
    n_q_, n_v_, x_lcs_curr, is_doing_c3_, sampling_params_);
  candidate_states.insert(candidate_states.begin(), x_lcs_curr);
  int num_total_samples = candidate_states.size();

  // Update the set of sample locations under consideration.
  for (int i = 0; i < num_total_samples; i++) {
    all_sample_locations_[i] = candidate_states[i].head(3);
  }
  for (int i = num_total_samples; i < all_sample_locations_.size(); i++) {
    all_sample_locations_[i] = Vector3d::Zero();
  }

  // Make LCS objects for each sample.
  std::vector<solvers::LCS> candidate_lcs_objects;
  for (int i; i < num_total_samples; i++) {
    // Context needs to be updated to create the LCS objects.
    UpdateContext(candidate_states[i]);

    // TODO: Make the ContactModel contact_model accessiible everywhere from 
    // constructor.
    // Resolving contact model param to ContactModel type to pass to LCSFactory.
    solvers::ContactModel contact_model;
    if (c3_options_.contact_model == "stewart_and_trinkle") {
      contact_model = solvers::ContactModel::kStewartAndTrinkle;
    } else if (c3_options_.contact_model == "anitescu") {
      contact_model = solvers::ContactModel::kAnitescu;
    } else {
      throw std::runtime_error("unknown or unsupported contact model");
    }
    // Create an LCS object.
    // Preprocessing the contact pairs to resolve the contact pairs
    vector<SortedPair<GeometryId>> resolved_contact_pairs =
      LCSFactoryPreProcessor::PreProcessor(
        plant_, *context_, contact_pairs_, c3_options_.num_friction_directions);
    auto sample_system_scaling_pair = solvers::LCSFactory::LinearizePlantToLCS(
      plant_, *context_, plant_ad_, *context_ad_, resolved_contact_pairs,
      c3_options_.num_friction_directions, c3_options_.mu, 
      c3_options_.planning_dt, N_, contact_model);
    solvers::LCS lcs_object_sample = sample_system_scaling_pair.first;

    // Store LCS object.
    candidate_lcs_objects.push_back(lcs_object_sample);
  }

  // Preparation for parallelization.
  all_sample_costs_ = std::vector<double>(num_total_samples, -1);
  std::vector<std::shared_ptr<solvers::C3>> c3_objects(num_total_samples, nullptr);
  std::vector<std::vector<Eigen::VectorXd>> deltas(num_total_samples, 
                  std::vector<Eigen::VectorXd>(N_,VectorXd::Zero(n_x_ + n_lambda_ + n_u_)));
  std::vector<std::vector<Eigen::VectorXd>> ws(num_total_samples, 
                  std::vector<Eigen::VectorXd>(N_,VectorXd::Zero(n_x_ + n_lambda_ + n_u_)));

  // Parallelize over computing C3 costs for each sample.
  #pragma omp parallel for num_threads(num_threads_to_use_)
    for (int i = 0; i < num_total_samples; i++) {
      // Get the candidate state, its LCS representation.
      Eigen::VectorXd test_state = candidate_states.at(i);
      solvers::LCS test_system = candidate_lcs_objects.at(i);

      // Set up C3 MIQP.
      std::shared_ptr<solvers::C3> test_c3_object;
      
      if (c3_options_.projection_type == "MIQP") {
        test_c3_object = std::make_shared<C3MIQP>(test_system, 
                                            C3::CostMatrices(Q_, R_, G_, U_), 
                                            x_desired, c3_options_);

      } else if (c3_options_.projection_type == "QP") {
        test_c3_object = std::make_shared<C3QP>(test_system, 
                                            C3::CostMatrices(Q_, R_, G_, U_), 
                                            x_desired, c3_options_);
      } else {
        std::cerr << ("Unknown projection type") << std::endl;
        DRAKE_THROW_UNLESS(false);
      }

      // Solve C3, store resulting object and cost.
      test_c3_object->Solve(test_state, deltas.at(i), ws.at(i));
      auto cost_trajectory_pair = test_c3_object->CalcCost(test_state, true);
      double c3_cost = cost_trajectory_pair.first;
      // TODO: This doesn't work because of the unique_ptr. Figure out how to fix.
       #pragma omp critical
      {
        c3_objects.at(i) = test_c3_object;
      }
      // Add travel cost.  Ignore differences in z.
      double xy_travel_distance = (test_state.head(2) - 
                                   x_lcs_curr.head(2)).norm();
      all_sample_costs_[i] = c3_cost + 
        sampling_params_.travel_cost_per_meter*xy_travel_distance;
      // Add additional costs based on repositioning progress.
      if ((i==CURRENT_REPOSITION_INDEX) & (finished_reposition_flag_==true)) {
        all_sample_costs_[i] += sampling_params_.finished_reposition_cost;
      }
      else if (i > CURRENT_LOCATION_INDEX) {
        all_sample_costs_[i] += sampling_params_.reposition_fixed_cost;
      }
    }
  // End of parallelization

  // Review the cost results to determine the best sample.
  double best_additional_sample_cost;
  if (num_total_samples > 1) {
    std::vector<double> additional_sample_cost_vector = 
      std::vector<double>(all_sample_costs_.begin()+1, all_sample_costs_.end());
    best_additional_sample_cost = 
      *std::min_element(additional_sample_cost_vector.begin(),
                        additional_sample_cost_vector.end());
    std::vector<double>::iterator it = 
      std::min_element(std::begin(additional_sample_cost_vector),
                       std::end(additional_sample_cost_vector));
    best_sample_index_ = (SampleIndex)(
      std::distance(std::begin(additional_sample_cost_vector), it) + 1);
  }

  // Update C3 objects and intermediates for current and best samples.
  // c3_curr_plan_ = std::make_unique<solvers::C3>(std::move(c3_objects.at(CURRENT_LOCATION_INDEX)));
  c3_curr_plan_ = c3_objects.at(CURRENT_LOCATION_INDEX);
  // c3_best_plan_ = std::make_unique<solvers::C3>(std::move(c3_objects.at(best_sample_index_)));
  c3_best_plan_ = c3_objects.at(best_sample_index_);
  delta_curr_plan_ = deltas.at(CURRENT_LOCATION_INDEX);
  delta_best_plan_ = deltas.at(best_sample_index_);
  w_curr_plan_ = ws.at(CURRENT_LOCATION_INDEX);
  w_best_plan_ = ws.at(best_sample_index_);

  // Determine whether to do C3 or reposition.
  if (is_doing_c3_ == true) { // Currently doing C3.
    // Switch to repositioning if one of the other samples is better, with
    // hysteresis.
    if (all_sample_costs_[CURRENT_LOCATION_INDEX] > 
        best_additional_sample_cost + sampling_params_.switching_hysteresis) {
      is_doing_c3_ = false;
      finished_reposition_flag_ = false;
    }
  }
  else { // Currently repositioning.
    // Switch to C3 if the current sample is better, with hysteresis.
    if (best_additional_sample_cost > 
        all_sample_costs_[CURRENT_LOCATION_INDEX] + 
        sampling_params_.switching_hysteresis) {
      is_doing_c3_ = true;
      finished_reposition_flag_ = false;
    }
  }

  // Update the execution trajectories.  Both C3 and repositioning trajectories
  // are updated, but the is_doing_c3_ flag determines which one is used via the
  // downstream selector system.
  SamplingC3Controller::UpdateContext(x_lcs_curr);
  // Grab the drake context time and pass it to the C3traj update functions.
  double t = context.get_discrete_state(plan_start_time_index_)[0];
  UpdateC3ExecutionTrajectory(x_lcs_curr, t);
  // Grab the drake context time and pass it to the Repostraj update functions.
  // TODO: Do we need to do this again or can we just use the same time?
  t = context.get_discrete_state(plan_start_time_index_)[0];
  UpdateRepositioningExecutionTrajectory(x_lcs_curr, t);

  // End of control loop cleanup.
  auto finish = std::chrono::high_resolution_clock::now();
  auto elapsed = finish - start;
  double solve_time =
      std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() /
      1e6;
  filtered_solve_time_ = (1 - solve_time_filter_constant_) * solve_time +
                         (solve_time_filter_constant_)*filtered_solve_time_;
  return drake::systems::EventStatus::Succeeded();
}

// Helper function to update context of a plant with a given state.
void SamplingC3Controller::UpdateContext(Eigen::VectorXd lcs_state) const {
    // Update autodiff.
    VectorXd xu_test(n_q_ + n_v_ + n_u_);
    
    // u here is set to a vector of 1000s -- TODO why?
    VectorXd test_u = 1000*VectorXd::Ones(n_u_);

    // Update context with respect to positions and velocities associated with 
    // the candidate state.
    VectorXd test_q = lcs_state.head(n_q_);
    VectorXd test_v = lcs_state.tail(n_v_);
    
    xu_test << test_q, test_v, test_u;                   
    auto xu_ad_test = drake::math::InitializeAutoDiff(xu_test);

    plant_ad_.SetPositionsAndVelocities(
        context_ad_,
        xu_ad_test.head(n_q_ + n_v_));

    multibody::SetInputsIfNew<AutoDiffXd>(
        plant_ad_, xu_ad_test.tail(n_u_), context_ad_);

    plant_.SetPositions(context_, test_q);
    plant_.SetVelocities(context_, test_v);
    multibody::SetInputsIfNew<double>(plant_, test_u, context_);
}

// Perform one step of C3.
void SamplingC3Controller::UpdateC3ExecutionTrajectory(
  const VectorXd& x_lcs, const double& t_context) const {

  // Read the time from the t_context i/p for setting timestamps.
  double t = t_context;

  // Get the input from the plan.
  vector<VectorXd> u_sol = c3_curr_plan_->GetInputSolution();


  // Resolving contact model param to ContactModel type to pass to LCSFactory.
  solvers::ContactModel contact_model;
  if (c3_options_.contact_model == "stewart_and_trinkle") {
    contact_model = solvers::ContactModel::kStewartAndTrinkle;
  } else if (c3_options_.contact_model == "anitescu") {
    contact_model = solvers::ContactModel::kAnitescu;
  } else {
    throw std::runtime_error("unknown or unsupported contact model");
  }

  // Create an LCS object.
  vector<SortedPair<GeometryId>> resolved_contact_pairs =
    LCSFactoryPreProcessor::PreProcessor(
      plant_, *context_, contact_pairs_, c3_options_.num_friction_directions);
  auto system_scaling_pair = solvers::LCSFactory::LinearizePlantToLCS(
    plant_, *context_, plant_ad_, *context_ad_, resolved_contact_pairs,
    c3_options_.num_friction_directions, c3_options_.mu, 
    c3_options_.execution_dt, N_, contact_model);
  solvers::LCS lcs_object = system_scaling_pair.first;

  // Setting up matrices to set up LCMTrajectory object.
  Eigen::MatrixXd knots = Eigen::MatrixXd::Zero(n_x_, N_ + 1);
  Eigen::VectorXd timestamps = Eigen::VectorXd::Zero(N_ + 1);

  // Roll out the execution LCS with the planned inputs.
  knots.col(0) = x_lcs;
  timestamps[0] = t + filtered_solve_time_;
  for (int i = 0; i < N_; i++) {
    // Set up matrices for LCMTrajectory object.
    // Simulate returns a const Eigen::VectorXd& so we need to cast it to a
    // non-const type.
    knots.col(i+1) = lcs_object.Simulate(knots.col(i), u_sol[i]);
    timestamps[i+1] = t + filtered_solve_time_ + (i+1)*c3_options_.execution_dt;
  }
  
  LcmTrajectory::Trajectory c3_execution_traj;
  c3_execution_traj.traj_name = "end_effector_position_target";
  c3_execution_traj.datatypes =
      std::vector<std::string>(knots.rows(), "double");
  c3_execution_traj.datapoints = 
    knots(Eigen::seqN(0, 3),Eigen::seqN(0, N_+1));
  c3_execution_traj.time_vector = timestamps.cast<double>();
  c3_execution_lcm_traj_.ClearTrajectories();
  c3_execution_lcm_traj_.AddTrajectory("end_effector_position_target", 
                                      c3_execution_traj);

  // In c3 mode, the end effector forces should match the solved c3 inputs.
  Eigen::MatrixXd force_samples = Eigen::MatrixXd::Zero(u_sol.size(), u_sol[0].size()); //u_sol;
  for (int i = 0; i < u_sol.size(); i++) {
    force_samples.col(i) = u_sol[i];
  }
  LcmTrajectory::Trajectory force_traj;
  force_traj.traj_name = "end_effector_force_target";
  force_traj.datatypes =
      std::vector<std::string>(force_samples.rows(), "double");
  force_traj.datapoints = force_samples;
  force_traj.time_vector = timestamps.cast<double>();
  c3_execution_lcm_traj_.AddTrajectory(force_traj.traj_name, force_traj);

  LcmTrajectory::Trajectory object_traj;
  object_traj.traj_name = "object_position_target";
  object_traj.datatypes = std::vector<std::string>(knots.rows(), "double");
  object_traj.datapoints = knots(Eigen::seqN(n_q_ - 3, 3),Eigen::seqN(0, N_+1));
  object_traj.time_vector = timestamps.cast<double>();
  c3_execution_lcm_traj_.AddTrajectory(object_traj.traj_name, object_traj);

  LcmTrajectory::Trajectory object_orientation_traj;
  // first 3 rows are rpy, last 3 rows are angular velocity
  MatrixXd orientation_samples = MatrixXd::Zero(4, N_);
  orientation_samples = knots(Eigen::seqN(n_q_ - 7, 4),Eigen::seqN(0, N_+1));
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

  // Get the current end effector location.
  Vector3d current_ee_location = x_lcs.head(3);
  Vector3d current_object_location = x_lcs.segment(7, 3);

  Vector3d curr_to_goal_vec = best_sample_location - current_ee_location;

  // Compute spline waypoints.
  Vector3d p0 = current_ee_location;
  Vector3d p3 = best_sample_location;
  Vector3d p1 = current_ee_location + 0.25*curr_to_goal_vec - 
                current_object_location;
  p1 = current_object_location + sampling_params_.spline_width*p1/p1.norm();
  Vector3d p2 = current_ee_location + 0.75*curr_to_goal_vec - 
                current_object_location;
  p2 = current_object_location + sampling_params_.spline_width*p2/p2.norm();

  // Compute total estimated travel time for spline.
  double travel_distance = curr_to_goal_vec.norm();
  double total_travel_time = travel_distance/sampling_params_.reposition_speed;

  // Read the time from the t_context i/p for setting timestamps.
  double t = t_context;

  // Setting up matrices to set up LCMTrajectory object.
  Eigen::MatrixXd knots = Eigen::MatrixXd::Zero(n_x_, N_+1);
  Eigen::VectorXd timestamps = Eigen::VectorXd::Zero(N_+1);

  // Roll out the execution LCS with the planned inputs.
  knots.col(0) = x_lcs;
  timestamps[0] = t + filtered_solve_time_;

  for (int i = 0; i < N_+1; i++) {
    double t_spline = (i+1)*c3_options_.execution_dt/total_travel_time;
    // Don't overshoot the end of the spline.
    t_spline = std::min(1.0, t_spline);

    Vector3d next_ee_loc = p0 + t_spline*(-3*p0 + 3*p1) + 
                           std::pow(t_spline,2) * (3*p0 - 6*p1 + 3*p2) +
                           std::pow(t_spline,3) * (-p0 + 3*p1 - 3*p2 + p3);

    // Set the next LCS state as the current state with updated end effector
    // location and zero end effector velocity. Note that this means that the
    // object does not move in the planned trajectory. An alternative is that we
    // could simulate the object's motion with 0 input.
    VectorXd next_lcs_state = x_lcs;
    next_lcs_state.head(3) = next_ee_loc;
    next_lcs_state.segment(n_q_, 3) = Vector3d::Zero();

    knots.col(i)= next_lcs_state;
    timestamps[i] = t + filtered_solve_time_ + (i+1)*c3_options_.execution_dt;
  }

  LcmTrajectory::Trajectory repos_execution_traj;
  repos_execution_traj.traj_name = "end_effector_position_target";
  repos_execution_traj.datatypes =
      std::vector<std::string>(knots.rows(), "double");
  repos_execution_traj.datapoints = 
    knots(Eigen::seqN(0, 3),Eigen::seqN(0, N_+1));
  repos_execution_traj.time_vector = timestamps.cast<double>();
  repos_execution_lcm_traj_.ClearTrajectories();
  repos_execution_lcm_traj_.AddTrajectory("end_effector_position_target", 
                                          repos_execution_traj);
  
  // In repositioning mode, the end effector should not exert forces.
  MatrixXd force_samples = MatrixXd::Zero(3, N_);
  LcmTrajectory::Trajectory force_traj;
  force_traj.traj_name = "end_effector_force_target";
  force_traj.datatypes =
      std::vector<std::string>(force_samples.rows(), "double");
  force_traj.datapoints = force_samples;
  force_traj.time_vector = timestamps.cast<double>();
  repos_execution_lcm_traj_.AddTrajectory(force_traj.traj_name, force_traj);


  LcmTrajectory::Trajectory object_traj;
  object_traj.traj_name = "object_position_target";
  object_traj.datatypes = std::vector<std::string>(knots.rows(), "double");
  object_traj.datapoints = 
    knots(Eigen::seqN(n_q_ - 3, 3),Eigen::seqN(0, N_+1));
  object_traj.time_vector = timestamps.cast<double>();
  repos_execution_lcm_traj_.AddTrajectory(object_traj.traj_name, object_traj);

  LcmTrajectory::Trajectory object_orientation_traj;
  // first 3 rows are rpy, last 3 rows are angular velocity
  MatrixXd orientation_samples = MatrixXd::Zero(4, N_);
  orientation_samples = knots(Eigen::seqN(n_q_ - 7, 4),Eigen::seqN(0, N_+1));
  object_orientation_traj.traj_name = "object_orientation_target";
  object_orientation_traj.datatypes =
      std::vector<std::string>(orientation_samples.rows(), "double");
  object_orientation_traj.datapoints = orientation_samples;
  object_orientation_traj.time_vector = timestamps.cast<double>();
  repos_execution_lcm_traj_.AddTrajectory(object_orientation_traj.traj_name,
                         object_orientation_traj);
}

// Output port handlers for current location
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

  // Interpolate the z_sol according to the time it takes to perform a control 
  // loop. If control loop is longer than the plan horizon, use the last z.
  if (filtered_solve_time_ < (N_ - 1) * dt_) {
    int index = filtered_solve_time_ / dt_;
    double weight = ((index + 1) * dt_ - filtered_solve_time_) / dt_;
    x_pred_curr_plan_ = weight * z_sol[index].segment(0, n_x_) +
              (1 - weight) * z_sol[index + 1].segment(0, n_x_);
  } else {
    x_pred_curr_plan_ = z_sol[N_ - 1].segment(0, n_x_);
  }
}

void SamplingC3Controller::OutputC3IntermediatesCurrPlan(
    const drake::systems::Context<double>& context,
    C3Output::C3Intermediates* c3_intermediates) const {
  double t = context.get_discrete_state(plan_start_time_index_)[0] +
             filtered_solve_time_;

  for (int i = 0; i < N_; i++) {
    c3_intermediates->time_vector_(i) = t + i * c3_options_.dt;
    c3_intermediates->w_.col(i) = w_curr_plan_[i].cast<float>();
    c3_intermediates->delta_.col(i) = delta_curr_plan_[i].cast<float>();
  }
}

void SamplingC3Controller::OutputLCSContactJacobianCurrPlan(
    const drake::systems::Context<double>& context,
    std::pair<Eigen::MatrixXd, std::vector<Eigen::VectorXd>>* lcs_contact_jacobian
) const {
  const TimestampedVector<double>* lcs_x =
      (TimestampedVector<double>*)this->EvalVectorInput(context,
                                                        lcs_state_input_port_);

  UpdateContext(lcs_x->get_data());
  
  solvers::ContactModel contact_model;
  if (c3_options_.contact_model == "stewart_and_trinkle") {
    contact_model = solvers::ContactModel::kStewartAndTrinkle;
  } else if (c3_options_.contact_model == "anitescu") {
    contact_model = solvers::ContactModel::kAnitescu;
  } else {
    throw std::runtime_error("unknown or unsupported contact model");
  }

  vector<SortedPair<GeometryId>> resolved_contact_pairs =
    LCSFactoryPreProcessor::PreProcessor(
      plant_, *context_, contact_pairs_, c3_options_.num_friction_directions);
  *lcs_contact_jacobian = LCSFactory::ComputeContactJacobian(
      plant_, *context_, plant_ad_, *context_ad_, resolved_contact_pairs,
      c3_options_.num_friction_directions, c3_options_.mu, c3_options_.dt,
      c3_options_.N, contact_model);
}

// Output port handlers for best sample location
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

  // Interpolate the z_sol according to the time it takes to perform a control 
  // loop. If control loop is longer than the plan horizon, use the last z.
  if (filtered_solve_time_ < (N_ - 1) * dt_) {
    int index = filtered_solve_time_ / dt_;
    double weight = ((index + 1) * dt_ - filtered_solve_time_) / dt_;
    x_pred_best_plan_ = weight * z_sol[index].segment(0, n_x_) +
              (1 - weight) * z_sol[index + 1].segment(0, n_x_);
  } else {
    x_pred_best_plan_ = z_sol[N_-1].segment(0, n_x_);
  }
}

void SamplingC3Controller::OutputC3IntermediatesBestPlan(
    const drake::systems::Context<double>& context,
    C3Output::C3Intermediates* c3_intermediates) const {
  double t = context.get_discrete_state(plan_start_time_index_)[0] +
             filtered_solve_time_;

  for (int i = 0; i < N_; i++) {
    c3_intermediates->time_vector_(i) = t + i * c3_options_.dt;
    c3_intermediates->w_.col(i) = w_best_plan_[i].cast<float>();
    c3_intermediates->delta_.col(i) = delta_best_plan_[i].cast<float>();
  }
}

void SamplingC3Controller::OutputLCSContactJacobianBestPlan(
    const drake::systems::Context<double>& context,
    std::pair<Eigen::MatrixXd, std::vector<Eigen::VectorXd>>* lcs_contact_jacobian
) const {

  const TimestampedVector<double>* lcs_x =
      (TimestampedVector<double>*)this->EvalVectorInput(context,
                                                        lcs_state_input_port_);

  // Linearize about state with end effector in sample location.
  VectorXd x_sample = lcs_x->get_data();
  x_sample.head(3) = all_sample_locations_[best_sample_index_];
  UpdateContext(x_sample);

  solvers::ContactModel contact_model;
  if (c3_options_.contact_model == "stewart_and_trinkle") {
    contact_model = solvers::ContactModel::kStewartAndTrinkle;
  } else if (c3_options_.contact_model == "anitescu") {
    contact_model = solvers::ContactModel::kAnitescu;
  } else {
    throw std::runtime_error("unknown or unsupported contact model");
  }

  vector<SortedPair<GeometryId>> resolved_contact_pairs =
    LCSFactoryPreProcessor::PreProcessor(
      plant_, *context_, contact_pairs_, c3_options_.num_friction_directions);
  *lcs_contact_jacobian = LCSFactory::ComputeContactJacobian(
      plant_, *context_, plant_ad_, *context_ad_, resolved_contact_pairs,
      c3_options_.num_friction_directions, c3_options_.mu, c3_options_.dt,
      c3_options_.N, contact_model);

  // Revert the context.
  UpdateContext(lcs_x->get_data());
}

// Output port handlers for executing C3 and repositioning ports
void SamplingC3Controller::OutputC3TrajExecute(
    const drake::systems::Context<double>& context,
    LcmTrajectory* output_c3_execution_lcm_traj) const {
  // TODO: The below assignment may not work; may need to iterate over length of
  // the vector of timestamped vectors and use SetData / set_timestamp.
  *output_c3_execution_lcm_traj = c3_execution_lcm_traj_;
}

void SamplingC3Controller::OutputReposTrajExecute(
    const drake::systems::Context<double>& context,
    LcmTrajectory* output_repos_execution_lcm_traj) const {
  // TODO: The below assignment may not work; may need to iterate over length of
  // the vector of timestamped vectors and use SetData / set_timestamp.
  *output_repos_execution_lcm_traj = repos_execution_lcm_traj_;
}

void SamplingC3Controller::OutputTrajExecute(
    const drake::systems::Context<double>& context,
    LcmTrajectory* output_execution_lcm_traj) const {
  if(is_doing_c3_){
    *output_execution_lcm_traj = c3_execution_lcm_traj_;
  } else {
    *output_execution_lcm_traj = repos_execution_lcm_traj_;
  }
}

void SamplingC3Controller::OutputIsC3Mode(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* is_c3_mode) const {
  Eigen::VectorXd vec = VectorXd::Constant(1, is_doing_c3_);
  is_c3_mode->SetFromVector(vec);
}

// Output port handlers for sample-related ports
void SamplingC3Controller::OutputAllSampleLocations(
    const drake::systems::Context<double>& context,
    std::vector<Eigen::Vector3d>* all_sample_locations) const {
  *all_sample_locations = all_sample_locations_;
}

void SamplingC3Controller::OutputAllSampleCosts(
    const drake::systems::Context<double>& context,
    std::vector<double>* all_sample_costs) const {
  *all_sample_costs = all_sample_costs_;
}

} // namespace systems 
} // namespace dairlib

