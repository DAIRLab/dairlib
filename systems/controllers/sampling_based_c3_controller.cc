#include "sampling_based_c3_controller.h"
#include <iostream>
#include <thread>

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

#include <Eigen/Dense>

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
    SamplingC3SamplingParams sampling_params,
    bool verbose)
    : plant_(plant),
      context_(context),
      plant_ad_(plant_ad),
      context_ad_(context_ad),
      contact_pairs_(contact_geoms),
      c3_options_(std::move(c3_options)),
      sampling_params_(std::move(sampling_params)),
      G_(std::vector<MatrixXd>(c3_options_.N, c3_options_.G)),
      U_(std::vector<MatrixXd>(c3_options_.N, c3_options_.U)),
      N_(c3_options_.N),
      verbose_(verbose){
  this->set_name("sampling_c3_controller");

  double discount_factor = 1;
  for (int i = 0; i < N_; ++i) {
    Q_.push_back(discount_factor * c3_options_.Q_position);
    R_.push_back(discount_factor * c3_options_.R);
    discount_factor *= c3_options_.gamma;
  }
  Q_.push_back(discount_factor * c3_options_.Q_position);
  DRAKE_DEMAND(Q_.size() == N_ + 1);
  DRAKE_DEMAND(R_.size() == N_);

  n_q_ = plant_.num_positions();
  n_v_ = plant_.num_velocities();
  n_u_ = plant_.num_actuators();

  n_x_ = n_q_ + n_v_;

  solve_time_filter_constant_ = c3_options_.solve_time_filter_alpha;
  if (c3_options_.contact_model == "stewart_and_trinkle") {
    n_lambda_ =
        2 * c3_options_.num_contacts[c3_options_.num_contacts_index] +
        2 * c3_options_.num_friction_directions * c3_options_.num_contacts[c3_options_.num_contacts_index];
  } else if (c3_options_.contact_model == "anitescu") {
    n_lambda_ =
        2 * c3_options_.num_friction_directions * c3_options_.num_contacts[c3_options_.num_contacts_index];
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
  int num_samples = std::max(sampling_params_.num_additional_samples_repos + 1,
                             sampling_params_.num_additional_samples_c3);
  // The +1 here is to account for the current location.
  all_sample_locations_ = vector<Vector3d>(num_samples + 1, Vector3d::Zero());
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
    "c3_traj_execute", lcm_traj, 
    &SamplingC3Controller::OutputC3TrajExecute
  ).get_index();
  repos_traj_execute_port_ = this->DeclareAbstractOutputPort(
    "repos_traj_execute", lcm_traj, 
    &SamplingC3Controller::OutputReposTrajExecute
  ).get_index();
  // NOTE: We reuse lcm_traj to set the right type for the port.
  traj_execute_port_ = this->DeclareAbstractOutputPort(
    "traj_execute", lcm_traj, 
    &SamplingC3Controller::OutputTrajExecute
  ).get_index();
  is_c3_mode_port_ = this->DeclareVectorOutputPort(
    "is_c3_mode", drake::systems::BasicVector<double>(1),
    &SamplingC3Controller::OutputIsC3Mode
  ).get_index();

  // Output ports for dynamically feasible plans used for cost computation and 
  // visualization.
  dynamically_feasible_curr_plan_port_ = this->DeclareAbstractOutputPort(
    "dynamically_feasible_curr_plan", vector<VectorXd>(N_ + 1, VectorXd::Zero(n_x_)), 
    &SamplingC3Controller::OutputDynamicallyFeasibleCurrPlan
  ).get_index();
  dynamically_feasible_best_plan_port_ = this->DeclareAbstractOutputPort(
    "dynamically_feasible_best_plan", vector<VectorXd>(N_ + 1, VectorXd::Zero(n_x_)), 
    &SamplingC3Controller::OutputDynamicallyFeasibleBestPlan
  ).get_index();
  
  // Sample location related output ports.
  // This port will output all samples except the current location.
  // all_sample_locations_port_ does not include the current location. So 
  // index 0 is the first sample.
  all_sample_locations_port_ = this->DeclareAbstractOutputPort(
    "all_sample_locations", vector<Vector3d>(num_samples + 1, Vector3d::Zero()), 
    &SamplingC3Controller::OutputAllSampleLocations
  ).get_index();
  // all_sample_costs_port_ does include the current location. So index 0 is
  // the current location cost.
  all_sample_costs_port_ = this->DeclareAbstractOutputPort(
    "all_sample_costs", std::vector<double>(num_samples + 1, -1), 
    &SamplingC3Controller::OutputAllSampleCosts
  ).get_index();

  curr_and_best_sample_costs_port_ = this->DeclareAbstractOutputPort(
    "current_and_best_sample_cost", std::vector<double>(2, -1), 
    &SamplingC3Controller::OutputCurrAndBestSampleCost
  ).get_index();

  plan_start_time_index_ = DeclareDiscreteState(1);
  x_pred_curr_plan_ = VectorXd::Zero(n_x_);

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

  if (verbose_) {
    std::cout << "Initial filtered_solve_time_: " << filtered_solve_time_ << std::endl;
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
  return LCS(A, B, D, d, E, F, H, c, c3_options_.N, c3_options_.planning_dt);
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
      *this->template EvalVectorInput<BasicVector>(context, final_target_input_port_);
  const TimestampedVector<double>* lcs_x_curr =
      (TimestampedVector<double>*)this->EvalVectorInput(context,
                                                        lcs_state_input_port_);

  // Store the current LCS state.
  drake::VectorX<double> x_lcs_curr = lcs_x_curr->get_data();

  if(verbose_){
    std::cout << "x_lcs_curr: " << x_lcs_curr.transpose() << std::endl;
    std::cout << "x_lcs_des: " << x_lcs_des.get_value().transpose() << std::endl;
    std::cout << "x_lcs_final_des: " << x_lcs_final_des.get_value().transpose() << std::endl;
    std::cout << "x_pred_curr_plan_: " << x_pred_curr_plan_.transpose() << std::endl;
  }

  // If not tele-opping and if generating samples about a predicted lcs state, 
  // clamp the predicted next state to be not too far away from the current 
  // state.
  // Only use predicted state for c3 if use_predicted_x0_c3 is true and is_doing_c3_.
  // Only use predicted state for repositioning if use_predicted_x0_repos is true and !is_doing_c3_.
  if (!radio_out->channel[14] && !x_pred_curr_plan_.isZero() && c3_options_.use_predicted_x0_c3 && is_doing_c3_) {
    ClampEndEffectorAcceleration(x_lcs_curr);
    if(verbose_){
      std::cout << "x_lcs_curr after clamping in C3 mode: " << x_lcs_curr.transpose() << std::endl;
    }
  }
  else if (!radio_out->channel[14] && !x_pred_curr_plan_.isZero() && c3_options_.use_predicted_x0_repos && !is_doing_c3_) {
    ClampEndEffectorAcceleration(x_lcs_curr);
    if(verbose_){
      std::cout << "x_lcs_curr after clamping in repositioning mode: " << x_lcs_curr.transpose() << std::endl;
    }
  }
  else if (verbose_) {
    std::cout << "Not clamping x_lcs_curr." << std::endl;
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
    n_q_, n_v_, x_lcs_curr, is_doing_c3_, sampling_params_, c3_options_);
  
  // Add the previous best repositioning target to the candidate states at the 
  // index 1 always. (Index 0 will become the current state.)
  if (!is_doing_c3_){
    // Add the prev best repositioning target to the candidate states.
    Eigen::VectorXd repositioning_target_state = x_lcs_curr;
    repositioning_target_state.head(3) = prev_repositioning_target_;
    candidate_states.insert(candidate_states.begin(), repositioning_target_state);
  }
  // Insert the current location at the beginning of the candidate states.
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
  for (int i = 0; i < num_total_samples; i++) {
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
    // Preprocessing the contact pairs
    vector<SortedPair<GeometryId>> resolved_contact_pairs;
    bool use_closest_ee_obj_contact;
    // num_contacts_index 0 and 2 mean model closest ee-obj pair.
    // num_contacts_index 2 includes ee-ground contact. This is already handled 
    // in franka_c3_controller.cc.
    if(c3_options_.num_contacts_index == 0 || c3_options_.num_contacts_index == 2){
      use_closest_ee_obj_contact = true;
    }
    // num_contacts_index 1 and 3 mean model each ee-capsule pair.
    // num_contacts_index 3 includes ee-ground contact. This is already handled 
    // in franka_c3_controller.cc.
    else if(c3_options_.num_contacts_index == 1 || c3_options_.num_contacts_index == 3){
      use_closest_ee_obj_contact = false;
    }
    else{
      throw std::runtime_error("unknown or unsupported num_contacts_index");
    }
    resolved_contact_pairs = LCSFactoryPreProcessor::PreProcessor(
      plant_, *context_, contact_pairs_, c3_options_.num_friction_directions,
      c3_options_.num_contacts[c3_options_.num_contacts_index], 
      use_closest_ee_obj_contact, verbose_);

    solvers::LCS lcs_object_sample = solvers::LCSFactory::LinearizePlantToLCS(
      plant_, *context_, plant_ad_, *context_ad_, resolved_contact_pairs,
      c3_options_.num_friction_directions, c3_options_.mu[c3_options_.num_contacts_index], 
      c3_options_.planning_dt, N_, contact_model);

    // Store LCS object.
    candidate_lcs_objects.push_back(lcs_object_sample);
  }
  // Reset the context to the current lcs state. 
  UpdateContext(x_lcs_curr);

  if (verbose_) {
    // Print the LCS matrices for verbose inspection.
    solvers::LCS verbose_lcs = candidate_lcs_objects.at(CURRENT_LOCATION_INDEX);
    std::cout<<"A: "<<std::endl;
    std::cout<<verbose_lcs.A_[0]<<std::endl;
    std::cout<<"B: "<<std::endl;
    std::cout<<verbose_lcs.B_[0]<<std::endl;
    std::cout<<"D: "<<std::endl;
    std::cout<<verbose_lcs.D_[0]<<std::endl;
    std::cout<<"d: "<<std::endl;
    std::cout<<verbose_lcs.d_[0]<<std::endl;
    std::cout<<"E: "<<std::endl;
    std::cout<<verbose_lcs.E_[0]<<std::endl;
    std::cout<<"F: "<<std::endl;
    std::cout<<verbose_lcs.F_[0]<<std::endl;
    std::cout<<"H: "<<std::endl;
    std::cout<<verbose_lcs.H_[0]<<std::endl;
    std::cout<<"c: "<<std::endl;
    std::cout<<verbose_lcs.c_[0]<<std::endl;
  }

  // Preparation for parallelization.
  // This size is adapting to the number of samples based on mode.
  all_sample_costs_ = std::vector<double>(num_total_samples, -1);
  all_sample_dynamically_feasible_plans_ = std::vector<std::vector<Eigen::VectorXd>>(
    num_total_samples, std::vector<Eigen::VectorXd>(N_ + 1, VectorXd::Zero(n_x_)));
  std::vector<std::shared_ptr<solvers::C3>> c3_objects(num_total_samples, nullptr);
  std::vector<std::vector<Eigen::VectorXd>> deltas(num_total_samples, 
                  std::vector<Eigen::VectorXd>(N_,VectorXd::Zero(n_x_ + n_lambda_ + n_u_)));
  std::vector<std::vector<Eigen::VectorXd>> ws(num_total_samples, 
                  std::vector<Eigen::VectorXd>(N_,VectorXd::Zero(n_x_ + n_lambda_ + n_u_)));
  
  // if object is within a fixed radius of the desired location, 
  // change crossed_cost_switching_threshold_ to true.
  if (!crossed_cost_switching_threshold_){
    // std::cout<<"current object location: "<<x_lcs_curr.segment(7,2).transpose()<<std::endl;
    if ((x_lcs_curr.segment(7,2) - x_lcs_final_des.value().segment(7,2)).norm() < 
        sampling_params_.cost_switching_threshold_distance){
      crossed_cost_switching_threshold_ = true;
      std::cout << "Crossed cost switching threshold." << std::endl;
    }
  }

  if(crossed_cost_switching_threshold_){
    // clear the Q_ values and replace with costs for position and orientation.
    Q_.clear();
    double discount_factor = 1;
    for (int i = 0; i < N_; ++i) {
      Q_.push_back(discount_factor * c3_options_.Q_position_and_orientation);
      discount_factor *= c3_options_.gamma;
    }
    Q_.push_back(discount_factor * c3_options_.Q_position_and_orientation);
  }

  if (verbose_) {
    std::cout << "Q_[0] with gamma " << c3_options_.gamma << ":" << std::endl;
    std::cout << Q_[0] << std::endl;
    std::cout << "R_[0] with gamma " << c3_options_.gamma << ":" << std::endl;
    std::cout << R_[0] << std::endl;
  }

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
      test_c3_object->SetOsqpSolverOptions(solver_options_);
      test_c3_object->Solve(test_state, deltas.at(i), ws.at(i));
      // Get the state solution and calculate the cost.
      auto cost_trajectory_pair = test_c3_object->CalcCost(
        sampling_params_.cost_type);
      double c3_cost = cost_trajectory_pair.first;
      all_sample_dynamically_feasible_plans_.at(i) = cost_trajectory_pair.second;

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
      if ((i==CURRENT_REPOSITION_INDEX) && (finished_reposition_flag_==true)) {
        all_sample_costs_[i] += sampling_params_.finished_reposition_cost;
        finished_reposition_flag_ = false;
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
  else{
    // If there is no other sample, then set the best sample to the current 
    // location and set the cost to a high value to ensure that the c3 is chosen.
    best_additional_sample_cost = all_sample_costs_[CURRENT_LOCATION_INDEX] 
                                  + sampling_params_.c3_to_repos_hysteresis + 99;
  }

  if (verbose_) {
    std::cout << "All sample costs before hystereses: " << std::endl;
    for (int i = 0; i < num_total_samples; i++) {
      std::cout << "Sample " << i << " cost: " << all_sample_costs_[i] << std::endl;
    }
    std::cout << "In C3 mode? " << is_doing_c3_ << std::endl;
  }

  // Determine whether to do C3 or reposition.
  if (is_doing_c3_ == true) { // Currently doing C3.
    // Switch to repositioning if one of the other samples is better, with
    // hysteresis.
    if (all_sample_costs_[CURRENT_LOCATION_INDEX] > 
        best_additional_sample_cost + sampling_params_.c3_to_repos_hysteresis) {
      is_doing_c3_ = false;
      finished_reposition_flag_ = false;
    }
  }
  else { // Currently repositioning.
    // Switch to C3 if the current sample is better, with hysteresis.
    if(best_sample_index_ != CURRENT_REPOSITION_INDEX){
      // This means that the best sample is not the current repositioning
      // target. If the cost of the current repositioning target is better than
      // the current best sample by the hysteresis amount, then we continue 
      // pursuing the previous repositioning target.
      if(all_sample_costs_[CURRENT_REPOSITION_INDEX] < all_sample_costs_[best_sample_index_] + 
        sampling_params_.hysteresis_between_repos_targets){
          best_sample_index_ = CURRENT_REPOSITION_INDEX;
          best_additional_sample_cost = all_sample_costs_[CURRENT_REPOSITION_INDEX];
          finished_reposition_flag_ = false;
      }
      // Controller will switch to pursuing a new sample from its previous
      // repositioning target only if the cost of switching to that new sample
      // (with repos_to_repos hysteresis) is less than switching to C3 from
      // current location (with repos_to_c3 hysteresis), so add the
      // repos_to_repos hysteresis value here before the comparison to the
      // current location C3 cost with repos_to_c3 hysteresis afterwards.
      else {
        best_additional_sample_cost += sampling_params_.hysteresis_between_repos_targets;
      }
    }
    if (best_additional_sample_cost > 
        all_sample_costs_[CURRENT_LOCATION_INDEX] + 
        sampling_params_.repos_to_c3_hysteresis) {
      is_doing_c3_ = true;
      finished_reposition_flag_ = false;
    }
  }

  if (verbose_) {
    std::cout << "All sample costs before hystereses: " << std::endl;
    for (int i = 0; i < num_total_samples; i++) {
      std::cout << "Sample " << i << " cost: " << all_sample_costs_[i] << std::endl;
    }
    std::cout << "In C3 mode after considering switch? " << is_doing_c3_ << std::endl;
  }

  // These class variables get populated in the constructor in order to set osqp 
  // options. They get replaced by the following values.
  // Update C3 objects and intermediates for current and best samples.
  c3_curr_plan_ = c3_objects.at(CURRENT_LOCATION_INDEX);
  c3_best_plan_ = c3_objects.at(best_sample_index_);
  // These aren't currently used anywhere but may be required when implementing 
  // warm start.
  delta_curr_plan_ = deltas.at(CURRENT_LOCATION_INDEX);
  delta_best_plan_ = deltas.at(best_sample_index_);
  w_curr_plan_ = ws.at(CURRENT_LOCATION_INDEX);
  w_best_plan_ = ws.at(best_sample_index_);

  // This is a redundant value used only for plotting and analysis.
  // update current and best sample costs.
  curr_and_best_sample_cost_ = {all_sample_costs_[CURRENT_LOCATION_INDEX],
                                all_sample_costs_[best_sample_index_]};

  // Update the execution trajectories.  Both C3 and repositioning trajectories
  // are updated, but the is_doing_c3_ flag determines which one is used via the
  // downstream selector system.
  // Grab the drake context time and pass it to the C3traj & Repostraj update 
  // functions.
  double t = context.get_discrete_state(plan_start_time_index_)[0];
  UpdateC3ExecutionTrajectory(x_lcs_curr, t);
  UpdateRepositioningExecutionTrajectory(x_lcs_curr, t);

  if (verbose_) {
    std::cout << "x_pred_curr_plan_ after updating: " << x_pred_curr_plan_.transpose() << std::endl;
    std::vector<VectorXd> zs = c3_curr_plan_->GetFullSolution();
    for (int i = 0; i < N_; i++) {
      std::cout << "z[" << i << "]: " << zs[i].transpose() << std::endl;
    }
  }

  if (verbose_) {
    std::cout << "Dynamically feasible current plan: " << std::endl;
    for (int i=0; i<N_+1; i++){
      std::cout << all_sample_dynamically_feasible_plans_.at(CURRENT_LOCATION_INDEX)[i].segment(3,7).transpose() << std::endl;
    }
  }


  // Adding delay
  std::this_thread::sleep_for(std::chrono::milliseconds(sampling_params_.control_loop_delay_ms));
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
    std::cout << "At end of loop filtered_solve_time_: " << filtered_solve_time_ << std::endl;
  }
  
  return drake::systems::EventStatus::Succeeded();
}

// Helper function to clamp end effector acceleration if using predicted state.
void SamplingC3Controller::ClampEndEffectorAcceleration(
  drake::VectorX<double>& x_lcs_curr) const {
  // Use fixed approximate loop time for acceleration capping heuristic.
  float approximate_loop_dt = 0.05;
  float nominal_acceleration = 10;
        x_lcs_curr[0] = std::clamp(
          x_pred_curr_plan_[0], 
          x_lcs_curr[0] - nominal_acceleration * approximate_loop_dt * approximate_loop_dt,
          x_lcs_curr[0] + nominal_acceleration * approximate_loop_dt * approximate_loop_dt
        );
        x_lcs_curr[1] = std::clamp(
          x_pred_curr_plan_[1], 
          x_lcs_curr[1] - nominal_acceleration * approximate_loop_dt * approximate_loop_dt,
          x_lcs_curr[1] + nominal_acceleration * approximate_loop_dt * approximate_loop_dt
        );
        x_lcs_curr[2] = std::clamp(
          x_pred_curr_plan_[2], 
          x_lcs_curr[2] - nominal_acceleration * approximate_loop_dt * approximate_loop_dt,
          x_lcs_curr[2] + nominal_acceleration * approximate_loop_dt * approximate_loop_dt
        );
        x_lcs_curr[n_q_ + 0] = std::clamp(
          x_pred_curr_plan_[n_q_ + 0], 
          x_lcs_curr[n_q_ + 0] - nominal_acceleration * approximate_loop_dt,
          x_lcs_curr[n_q_ + 0] + nominal_acceleration * approximate_loop_dt
        );
        x_lcs_curr[n_q_ + 1] = std::clamp(
          x_pred_curr_plan_[n_q_ + 1], 
          x_lcs_curr[n_q_ + 1] - nominal_acceleration * approximate_loop_dt,
          x_lcs_curr[n_q_ + 1] + nominal_acceleration * approximate_loop_dt
        );
        x_lcs_curr[n_q_ + 2] = std::clamp(
          x_pred_curr_plan_[n_q_ + 2], 
          x_lcs_curr[n_q_ + 2] - nominal_acceleration * approximate_loop_dt,
          x_lcs_curr[n_q_ + 2] + nominal_acceleration * approximate_loop_dt
        );
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

  // Read the time from the t_context input for setting timestamps.
  double t = t_context;

  // Get the input from the plan.
  vector<VectorXd> u_sol = c3_curr_plan_->GetInputSolution();
  // Get full state solution from the plan.
  vector<VectorXd> x_sol = c3_curr_plan_->GetStateSolution();

  // Resolving contact model param to ContactModel type to pass to LCSFactory.
  solvers::ContactModel contact_model;
  if (c3_options_.contact_model == "stewart_and_trinkle") {
    contact_model = solvers::ContactModel::kStewartAndTrinkle;
  } else if (c3_options_.contact_model == "anitescu") {
    contact_model = solvers::ContactModel::kAnitescu;
  } else {
    throw std::runtime_error("unknown or unsupported contact model");
  }

  // Setting up matrices to set up LCMTrajectory object.
  Eigen::MatrixXd knots = Eigen::MatrixXd::Zero(n_x_, N_);
  Eigen::VectorXd timestamps = Eigen::VectorXd::Zero(N_);

  // Set up matrices for LCMTrajectory object.
  for (int i = 0; i < N_-1; i++) {
    knots.col(i) = x_sol[i+1];
    timestamps[i] = t + filtered_solve_time_ + (i+1)*c3_options_.planning_dt;
  }
  // TODO: Set the last knot to the simulated last state in the plan.
  knots.col(N_-1) = x_sol[N_-1];
  timestamps[N_-1] = t + filtered_solve_time_ + N_*c3_options_.planning_dt;


  if(is_doing_c3_){
    if (filtered_solve_time_ < (N_ - 1) * c3_options_.planning_dt) {
      int last_passed_index = filtered_solve_time_ / c3_options_.planning_dt;
      double fraction_to_next_index =
        (filtered_solve_time_ / c3_options_.planning_dt) - (double)last_passed_index;
      x_pred_curr_plan_ = knots.col(last_passed_index) +
        fraction_to_next_index * (knots.col(last_passed_index + 1) -
          knots.col(last_passed_index));
    } else {
      x_pred_curr_plan_ = knots.col(N_ - 1);
    }
  }

  // Add end effector position target to LCM Trajectory.
  LcmTrajectory::Trajectory ee_traj;
  ee_traj.traj_name = "end_effector_position_target";
  ee_traj.datatypes = std::vector<std::string>(3, "double");
  ee_traj.datapoints = knots(Eigen::seqN(0, 3),Eigen::seqN(0, N_));
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
  object_traj.datapoints = knots(Eigen::seqN(n_q_ - 3, 3),Eigen::seqN(0, N_));
  object_traj.time_vector = timestamps.cast<double>();
  c3_execution_lcm_traj_.AddTrajectory(object_traj.traj_name, object_traj);

  // Add object orientation target to LCM Trajectory.
  LcmTrajectory::Trajectory object_orientation_traj;
  MatrixXd orientation_samples = MatrixXd::Zero(4, N_);
  orientation_samples = knots(Eigen::seqN(n_q_ - 7, 4),Eigen::seqN(0, N_));
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
  Eigen::MatrixXd knots = Eigen::MatrixXd::Zero(n_x_, N_);
  Eigen::VectorXd timestamps = Eigen::VectorXd::Zero(N_);

  for (int i = 0; i < N_; i++) {
    // This is a curve_fraction and is not in the units of time or distance. 
    // When it is 0, it is the current location. When it is 1, it is the goal.
    double t_spline = (i)*c3_options_.planning_dt/total_travel_time;

    if (i == 1 && t_spline >= 1 && !is_doing_c3_){
      // If it can get there in one step, then set finished_reposition_flag_ to
      // true.
      finished_reposition_flag_ = true;
    }
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
    // if z is under the jack height, set it to jack height.
    if (next_lcs_state[2] < 0.0197) {
      next_lcs_state[2] = 0.0197;
    }

    knots.col(i)= next_lcs_state;
    timestamps[i] = t + filtered_solve_time_ + (i)*c3_options_.planning_dt;
  }

  if(!is_doing_c3_){
    if (filtered_solve_time_ < (N_ - 1) * c3_options_.planning_dt) {
      int last_passed_index = filtered_solve_time_ / c3_options_.planning_dt;
      double fraction_to_next_index =
        (filtered_solve_time_ / c3_options_.planning_dt) - (double)last_passed_index;
      x_pred_curr_plan_ = knots.col(last_passed_index) +
        fraction_to_next_index * (knots.col(last_passed_index + 1) -
          knots.col(last_passed_index));
    } else {
      x_pred_curr_plan_ = knots.col(N_ - 1);
    }
  }

  // Add end effector position target to LCM Trajectory.
  LcmTrajectory::Trajectory ee_traj;
  ee_traj.traj_name = "end_effector_position_target";
  ee_traj.datatypes = std::vector<std::string>(3, "double");
  ee_traj.datapoints = knots(Eigen::seqN(0, 3),Eigen::seqN(0, N_));
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
  object_traj.datapoints = 
    knots(Eigen::seqN(n_q_ - 3, 3),Eigen::seqN(0, N_));
  object_traj.time_vector = timestamps.cast<double>();
  repos_execution_lcm_traj_.AddTrajectory(object_traj.traj_name, object_traj);

  // Add object orientation target to LCM Trajectory.
  LcmTrajectory::Trajectory object_orientation_traj;
  MatrixXd orientation_samples = MatrixXd::Zero(4, N_);
  orientation_samples = knots(Eigen::seqN(n_q_ - 7, 4),Eigen::seqN(0, N_));
  object_orientation_traj.traj_name = "object_orientation_target";
  object_orientation_traj.datatypes =
      std::vector<std::string>(4, "double");
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
    c3_solution->time_vector_(i) = filtered_solve_time_ + t + i * c3_options_.planning_dt;
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
    c3_intermediates->time_vector_(i) = t + i * c3_options_.planning_dt;
    c3_intermediates->z_.col(i) = z_sol_curr_plan[i].cast<float>();
    c3_intermediates->w_.col(i) = w_curr_plan[i].cast<float>();
    c3_intermediates->delta_.col(i) = delta_curr_plan[i].cast<float>();
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

  // Preprocessing the contact pairs
  vector<SortedPair<GeometryId>> resolved_contact_pairs;
  bool use_closest_ee_obj_contact;
  // num_contacts_index 0 and 2 mean model closest ee-obj pair.
  // num_contacts_index 2 includes ee-ground contact. This is already handled 
  // in franka_c3_controller.cc.
  if(c3_options_.num_contacts_index == 0 || c3_options_.num_contacts_index == 2){
    use_closest_ee_obj_contact = true;
  }
  // num_contacts_index 1 and 3 mean model each ee-capsule pair.
  // num_contacts_index 3 includes ee-ground contact. This is already handled 
  // in franka_c3_controller.cc.
  else if(c3_options_.num_contacts_index == 1 || c3_options_.num_contacts_index == 3){
    use_closest_ee_obj_contact = false;
  }
  else{
    throw std::runtime_error("unknown or unsupported num_contacts_index");
  }
  resolved_contact_pairs = LCSFactoryPreProcessor::PreProcessor(
    plant_, *context_, contact_pairs_, c3_options_.num_friction_directions,
    c3_options_.num_contacts[c3_options_.num_contacts_index], 
    use_closest_ee_obj_contact, verbose_);

  // print size of resolved_contact_pairs
  *lcs_contact_jacobian = LCSFactory::ComputeContactJacobian(
      plant_, *context_, plant_ad_, *context_ad_, resolved_contact_pairs,
      c3_options_.num_friction_directions, 
      c3_options_.mu[c3_options_.num_contacts_index], c3_options_.planning_dt,
      c3_options_.N, contact_model);
}

// Output port handlers for best sample location
void SamplingC3Controller::OutputC3SolutionBestPlan(
    const drake::systems::Context<double>& context,
    C3Output::C3Solution* c3_solution) const {
  double t = context.get_discrete_state(plan_start_time_index_)[0];

  auto z_sol = c3_best_plan_->GetFullSolution();
  for (int i = 0; i < N_; i++) {
    c3_solution->time_vector_(i) = filtered_solve_time_ + t + i * c3_options_.planning_dt;
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
    c3_intermediates->time_vector_(i) = t + i * c3_options_.planning_dt;
    c3_intermediates->z_.col(i) = z_sol_best_plan[i].cast<float>();
    c3_intermediates->w_.col(i) = w_best_plan[i].cast<float>();
    c3_intermediates->delta_.col(i) = delta_best_plan[i].cast<float>();
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
  // TODO
  solvers::ContactModel contact_model;
  if (c3_options_.contact_model == "stewart_and_trinkle") {
    contact_model = solvers::ContactModel::kStewartAndTrinkle;
  } else if (c3_options_.contact_model == "anitescu") {
    contact_model = solvers::ContactModel::kAnitescu;
  } else {
    throw std::runtime_error("unknown or unsupported contact model");
  }

  // Preprocessing the contact pairs
  vector<SortedPair<GeometryId>> resolved_contact_pairs;
  bool use_closest_ee_obj_contact;
  // num_contacts_index 0 and 2 mean model closest ee-obj pair.
  // num_contacts_index 2 includes ee-ground contact. This is already handled 
  // in franka_c3_controller.cc.
  if(c3_options_.num_contacts_index == 0 || c3_options_.num_contacts_index == 2){
    use_closest_ee_obj_contact = true;
  }
  // num_contacts_index 1 and 3 mean model each ee-capsule pair.
  // num_contacts_index 3 includes ee-ground contact. This is already handled 
  // in franka_c3_controller.cc.
  else if(c3_options_.num_contacts_index == 1 || c3_options_.num_contacts_index == 3){
    use_closest_ee_obj_contact = false;
  }
  else{
    throw std::runtime_error("unknown or unsupported num_contacts_index");
  }
  resolved_contact_pairs = LCSFactoryPreProcessor::PreProcessor(
    plant_, *context_, contact_pairs_, c3_options_.num_friction_directions,
    c3_options_.num_contacts[c3_options_.num_contacts_index], 
    use_closest_ee_obj_contact, verbose_);

  *lcs_contact_jacobian = LCSFactory::ComputeContactJacobian(
      plant_, *context_, plant_ad_, *context_ad_, resolved_contact_pairs,
      c3_options_.num_friction_directions, 
      c3_options_.mu[c3_options_.num_contacts_index], c3_options_.planning_dt,
      c3_options_.N, contact_model);
  // Revert the context.
  UpdateContext(lcs_x->get_data());
}

// Output port handlers for executing C3 and repositioning ports
void SamplingC3Controller::OutputC3TrajExecute(
    const drake::systems::Context<double>& context,
    LcmTrajectory* output_c3_execution_lcm_traj) const {
  *output_c3_execution_lcm_traj = c3_execution_lcm_traj_;
}

void SamplingC3Controller::OutputReposTrajExecute(
    const drake::systems::Context<double>& context,
    LcmTrajectory* output_repos_execution_lcm_traj) const {
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

// Output port handler for Dynamically feasible trajectory used for cost 
// computation.
void SamplingC3Controller::OutputDynamicallyFeasibleCurrPlan(
    const drake::systems::Context<double>& context,
    std::vector<Eigen::VectorXd>* dynamically_feasible_curr_plan) const {
    std::vector<Eigen::VectorXd> obj_pose = std::vector<Eigen::VectorXd>(N_ + 1, VectorXd::Zero(7));
  // Extract object pose only from the dynamically feasible plan for each time step.
  for(int i = 0; i < N_ + 1; i++){
    obj_pose[i] << all_sample_dynamically_feasible_plans_.at(CURRENT_LOCATION_INDEX)[i].segment(3,7);
  }
  std::vector<Eigen::VectorXd> object_pose = std::vector<Eigen::VectorXd>(
    obj_pose.begin(), obj_pose.end());
  // Output dynamically feasible current plan.
  *dynamically_feasible_curr_plan = object_pose;
}
void SamplingC3Controller::OutputDynamicallyFeasibleBestPlan(
    const drake::systems::Context<double>& context,
    std::vector<Eigen::VectorXd>* dynamically_feasible_best_plan) const { 
    std::vector<Eigen::VectorXd> obj_pose = std::vector<Eigen::VectorXd>(N_ + 1, VectorXd::Zero(7));
  // Extract object pose only from the dynamically feasible plan for each time step.
  for(int i = 0; i < N_ + 1; i++){
    obj_pose[i] << all_sample_dynamically_feasible_plans_.at(best_sample_index_)[i].segment(3,7);
  }
  std::vector<Eigen::VectorXd> object_pose = std::vector<Eigen::VectorXd>(
    obj_pose.begin(), obj_pose.end());
  // Output dynamically feasible current plan.
  *dynamically_feasible_best_plan = object_pose;
}

// Output port handlers for sample-related ports
void SamplingC3Controller::OutputAllSampleLocations(
    const drake::systems::Context<double>& context,
    std::vector<Eigen::Vector3d>* all_sample_locations) const {
  // Output all sample locations including current location.
  std::vector<Eigen::Vector3d> sample_locations = std::vector<Eigen::Vector3d>(
    all_sample_locations_.begin(), all_sample_locations_.end());

  *all_sample_locations = sample_locations;
}

void SamplingC3Controller::OutputAllSampleCosts(
    const drake::systems::Context<double>& context,
    std::vector<double>* all_sample_costs) const {
  *all_sample_costs = all_sample_costs_;
}

void SamplingC3Controller::OutputCurrAndBestSampleCost(
    const drake::systems::Context<double>& context,
    std::vector<double>* curr_and_best_sample_cost) const {
  *curr_and_best_sample_cost = curr_and_best_sample_cost_;
}

} // namespace systems 
} // namespace dairlib
