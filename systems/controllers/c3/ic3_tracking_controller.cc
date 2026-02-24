#include "ic3_tracking_controller.h"
#include <Eigen/Dense>

#include "solvers/c3_miqp.h"
#include "solvers/c3_qp.h"
#include "solvers/c3_plus.h"
#include "solvers/lcs_factory.h"
#include "common/quaternion_error_hessian.h"
#include <iostream>

namespace dairlib {

using drake::multibody::ModelInstanceIndex;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteValues;
using Eigen::MatrixXd;
using Eigen::MatrixXf;
using Eigen::VectorXd;
using Eigen::VectorXf;
using solvers::C3Base;
using solvers::C3MIQP;
using solvers::C3QP;
using solvers::C3Plus;
using solvers::LCS;
using solvers::LCSFactory;
using std::vector;
using systems::TimestampedVector;

namespace systems {

iC3TrackingController::iC3TrackingController(
    const drake::multibody::MultibodyPlant<double>& plant, C3Options c3_options, 
      iC3Options ic3_options, double time_to_wait)
    : plant_(plant),
      c3_options_(std::move(c3_options)),
      ic3_options_(std::move(ic3_options)),
      G_(std::vector<MatrixXd>(c3_options_.N, c3_options_.G)),
      U_(std::vector<MatrixXd>(c3_options_.N, c3_options_.U)),
      N_(c3_options_.N), 
      time_to_wait_(time_to_wait) {
  this->set_name("ic3_tracking_controller");


  double discount_factor = 1;
  for (int i = 0; i < N_; ++i) {
    Q_.push_back(discount_factor * c3_options_.Q * ic3_options.c3_dt_scaling);
    R_.push_back(discount_factor * c3_options_.R * ic3_options.c3_dt_scaling);
    discount_factor *= c3_options_.gamma;
  }
  Q_.push_back(discount_factor * c3_options_.Q * ic3_options.c3_dt_scaling);
  DRAKE_DEMAND(Q_.size() == N_ + 1);
  DRAKE_DEMAND(R_.size() == N_);

  n_q_ = plant_.num_positions();
  n_v_ = plant_.num_velocities();
  n_u_ = plant_.num_actuators();
  n_x_ = n_q_ + n_v_;

  std::cout << "n_q: " << n_q_ << std::endl;
  std::cout << "n_v: " << n_v_ << std::endl;
  std::cout << "n_u: " << n_u_ << std::endl;
  std::cout << "n_x: " << n_x_ << std::endl;

  dt_ = c3_options_.dt;
  solve_time_filter_constant_ = c3_options_.solve_time_filter_alpha;
  if (c3_options_.contact_model == "stewart_and_trinkle") {
    n_lambda_ =
        2 * c3_options_.num_contacts +
        2 * c3_options_.num_friction_directions * c3_options_.num_contacts;
  } else if (c3_options_.contact_model == "anitescu") {
    n_lambda_ =
        2 * c3_options_.num_friction_directions * c3_options_.num_contacts;
  }
  VectorXd zeros = VectorXd::Zero(n_x_ + n_lambda_ + n_u_);

  n_u_ = plant_.num_actuators();

  // Creates placeholder lcs to construct base C3 problem
  // Placeholder LCS will have correct size as it's already determined by the
  // contact model
  auto lcs_placeholder = CreatePlaceholderLCS();
  auto x_desired_placeholder =
      std::vector<VectorXd>(N_ + 1, VectorXd::Zero(n_x_));
  if (c3_options_.projection_type == "MIQP") {
    c3_ = std::make_unique<C3MIQP>(lcs_placeholder,
                                   C3Base::CostMatrices(Q_, R_, G_, U_),
                                   x_desired_placeholder, c3_options_);

  } else if (c3_options_.projection_type == "QP") {
    c3_ = std::make_unique<C3QP>(lcs_placeholder,
                                 C3Base::CostMatrices(Q_, R_, G_, U_),
                                 x_desired_placeholder, c3_options_);

  } else if (c3_options_.projection_type == "C3+") {
    c3_ = std::make_unique<C3Plus>(lcs_placeholder,
                                 C3Base::CostMatrices(Q_, R_, G_, U_),
                                 x_desired_placeholder, c3_options_);

  } else {
    std::cerr << ("Unknown projection type") << std::endl;
    DRAKE_THROW_UNLESS(false);
  }

  c3_->SetOsqpSolverOptions(solver_options_);


  lcs_state_input_port_ =
      this->DeclareVectorInputPort("x_lcs", TimestampedVector<double>(n_x_))
          .get_index();
  lcs_input_port_ =
      this->DeclareAbstractInputPort("lcs", drake::Value<LCS>(lcs_placeholder))
          .get_index();

  target_input_port_ =
      this->DeclareVectorInputPort("x_lcs_des", n_x_).get_index();

  // Stuff specific to tracking ic3
  lqr_input_port_ =
      this->DeclareAbstractInputPort("lqr_input", drake::Value<lcmt_lqr_output>())
          .get_index();

  ic3_x_port_ =
      this->DeclareAbstractInputPort("ic3_x_port", drake::Value<lcmt_timestamped_saved_traj>())
          .get_index();

  ic3_u_port_ =
      this->DeclareAbstractInputPort("ic3_u_port", drake::Value<lcmt_timestamped_saved_traj>())
          .get_index();

  auto c3_solution = C3Output::C3Solution();
  c3_solution.x_sol_ = MatrixXf::Zero(n_q_ + n_v_, N_);
  c3_solution.lambda_sol_ = MatrixXf::Zero(n_lambda_, N_);
  c3_solution.u_sol_ = MatrixXf::Zero(n_u_, N_);
  c3_solution.time_vector_ = VectorXf::Zero(N_);
  auto c3_intermediates = C3Output::C3Intermediates();
  c3_intermediates.z_ = MatrixXf::Zero(n_x_ + n_lambda_ + n_u_, N_);
  c3_intermediates.delta_ = MatrixXf::Zero(n_x_ + n_lambda_ + n_u_, N_);
  c3_intermediates.w_ = MatrixXf::Zero(n_x_ + n_lambda_ + n_u_, N_);
  c3_intermediates.time_vector_ = VectorXf::Zero(N_);
  c3_solution_port_ =
      this->DeclareAbstractOutputPort("c3_solution", c3_solution,
                                      &iC3TrackingController::OutputC3Solution)
          .get_index();
  c3_intermediates_port_ =
      this->DeclareAbstractOutputPort("c3_intermediates", c3_intermediates,
                                      &iC3TrackingController::OutputC3Intermediates)
          .get_index();

  plan_start_time_index_ = DeclareDiscreteState(1);
  x_pred_index_ = DeclareDiscreteState(n_x_);
  filtered_solve_time_index_ = DeclareDiscreteState(1);

  if (c3_options_.publish_frequency > 0) {
    DeclarePeriodicDiscreteUpdateEvent(1 / c3_options_.publish_frequency, 0.0,
                                       &iC3TrackingController::ComputePlan);
  } else {
    DeclareForcedDiscreteUpdateEvent(&iC3TrackingController::ComputePlan);
  }

  this->DeclarePerStepDiscreteUpdateEvent(
    &iC3TrackingController::SetFirstCallTime);

	t0_idx_ = this->DeclareDiscreteState(1); // first time output called
	called_ = false;
}

LCS iC3TrackingController::CreatePlaceholderLCS() const {
  MatrixXd A = MatrixXd::Ones(n_x_, n_x_);
  MatrixXd B = MatrixXd::Zero(n_x_, n_u_);
  VectorXd d = VectorXd::Zero(n_x_);
  MatrixXd D = MatrixXd::Ones(n_x_, n_lambda_);
  MatrixXd E = MatrixXd::Zero(n_lambda_, n_x_);
  MatrixXd F = MatrixXd::Zero(n_lambda_, n_lambda_);
  MatrixXd H = MatrixXd::Zero(n_lambda_, n_u_);
  VectorXd c = VectorXd::Zero(n_lambda_);
  return LCS(A, B, D, d, E, F, H, c, c3_options_.N, c3_options_.dt);
}

drake::systems::EventStatus iC3TrackingController::ComputePlan(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  auto start = std::chrono::high_resolution_clock::now();

  double t0 = context.get_discrete_state(t0_idx_).GetAtIndex(0);

  const BasicVector<double>& x_des =
      *this->template EvalVectorInput<BasicVector>(context, target_input_port_);
  const TimestampedVector<double>* lcs_x =
      (TimestampedVector<double>*)this->EvalVectorInput(context,
                                                        lcs_state_input_port_);
  auto& lcs =
      this->EvalAbstractInput(context, lcs_input_port_)->get_value<LCS>();
  drake::VectorX<double> x_lcs = lcs_x->get_data();
  //std::cout << "x_lcs " << x_lcs.transpose() << std::endl;

  auto& x_pred = context.get_discrete_state(x_pred_index_).value();
  auto mutable_x_pred = discrete_state->get_mutable_value(x_pred_index_);
  auto mutable_solve_time =
      discrete_state->get_mutable_value(filtered_solve_time_index_);

  const std::string trajectory_name = "iteration_" + std::to_string(ic3_options_.iter_to_use);
  const auto& lcm_all_x_trajectories = 
    this->EvalAbstractInput(context, ic3_x_port_)->get_value<lcmt_timestamped_saved_traj>();
  const auto& lcm_all_u_trajectories = 
    this->EvalAbstractInput(context, ic3_u_port_)->get_value<lcmt_timestamped_saved_traj>();
  LcmTrajectory x_trajectory = LcmTrajectory(lcm_all_x_trajectories.saved_traj);
  LcmTrajectory u_trajectory = LcmTrajectory(lcm_all_u_trajectories.saved_traj);
  LcmTrajectory::Trajectory state_trajectory = x_trajectory.GetTrajectory(trajectory_name);
  LcmTrajectory::Trajectory force_trajectory = u_trajectory.GetTrajectory(trajectory_name);
  MatrixXd state_data = state_trajectory.datapoints;
  MatrixXd force_data = force_trajectory.datapoints;

  double curr_time = context.get_time() - t0;  
  double ic3_dt = ic3_options_.dt;
  int ic3_timestep = (curr_time - time_to_wait_) / ic3_dt;

  if (x_lcs.segment(n_q_, 3).norm() > 0.01 && c3_options_.use_predicted_x0 &&
      !x_pred.isZero()) {
    x_lcs[0] = std::clamp(x_pred[0], x_lcs[0] - 10 * dt_ * dt_,
                          x_lcs[0] + 10 * dt_ * dt_);
    x_lcs[1] = std::clamp(x_pred[1], x_lcs[1] - 10 * dt_ * dt_,
                          x_lcs[1] + 10 * dt_ * dt_);
    x_lcs[2] = std::clamp(x_pred[2], x_lcs[2] - 10 * dt_ * dt_,
                          x_lcs[2] + 10 * dt_ * dt_);
    x_lcs[n_q_ + 0] = std::clamp(x_pred[n_q_ + 0], x_lcs[n_q_ + 0] - 10 * dt_,
                                 x_lcs[n_q_ + 0] + 10 * dt_);
    x_lcs[n_q_ + 1] = std::clamp(x_pred[n_q_ + 1], x_lcs[n_q_ + 1] - 10 * dt_,
                                 x_lcs[n_q_ + 1] + 10 * dt_);
    x_lcs[n_q_ + 2] = std::clamp(x_pred[n_q_ + 2], x_lcs[n_q_ + 2] - 10 * dt_,
                                 x_lcs[n_q_ + 2] + 10 * dt_);
  }

  discrete_state->get_mutable_value(plan_start_time_index_)[0] =
      lcs_x->get_timestamp();


  std::vector<VectorXd> x_desired =
      std::vector<VectorXd>(N_ + 1, x_des.value());

  /*
  // HARDCODED, set x_des for plate state to planned states
  if (0 <= ic3_timestep) {
    for (int i = 0; i < N_+1; i++) {
      int idx = ic3_timestep + i;
      if (idx < ic3_options_.N + 1) {
        x_desired[i].segment(0, 5) = state_data.col(idx).segment(0, 5);
        x_desired[i].segment(12, 5) = state_data.col(idx).segment(12, 5);
      } else {
        x_desired[i].segment(0, 5) = state_data.col(ic3_options_.N ).segment(0, 5);
        x_desired[i].segment(12, 5) = state_data.col(ic3_options_.N ).segment(12, 5);
      }
    }
  }
  */    

  UpdateQuaternionCosts(x_lcs, x_des.value());
  C3Base::CostMatrices new_costs(Q_, R_, G_, U_);
  c3_->UpdateCostMatrices(new_costs);
  c3_->UpdateTarget(x_desired);

  // Update final cost with LQR value function
  const auto* lqr_input = this->EvalAbstractInput(context, lqr_input_port_);
  if (lqr_input != nullptr && 0 <= ic3_timestep && 
        ic3_timestep <= ic3_options_.N - c3_options_.N * ic3_options_.c3_dt_scaling) {
    const auto& lqr_all_inputs = lqr_input->get_value<lcmt_lqr_output>();
    int idx = ic3_timestep + c3_options_.N * ic3_options_.c3_dt_scaling; // Want H[k + N]

    const std::vector<std::vector<std::vector<double>>>& source_H = lqr_all_inputs.H;
    const std::vector<std::vector<double>>& source_g = lqr_all_inputs.g;

    vector<Eigen::MatrixXd> H;
    vector<Eigen::VectorXd> g;

    for (int i = 0; i < source_H.size(); i++) {

      Eigen::MatrixXd mat(MatrixXd::Zero(source_H[i].size(), source_H[i][0].size()));
      for (int j = 0; j < source_H[i].size(); j++) {
          mat.row(j) = Eigen::Map<const Eigen::VectorXd>(source_H[i][j].data(), source_H[i][j].size());
      }
      H.push_back(std::move(mat));
      g.push_back(Eigen::VectorXd::Map(source_g[i].data(), source_g[i].size()));
    }
    c3_->UpdateFinalCost(H[idx], g[idx]);


    vector<VectorXd> u_target;
    for (int i = 0; i < N_; i++) {
      if (ic3_timestep <= ic3_options_.N) {
        if (i + ic3_timestep < ic3_options_.N) {
          u_target.push_back(force_data.col(i + ic3_timestep));
        } else {
          u_target.push_back(force_data.col(ic3_options_.N-1));
        }
      } else {
        VectorXd gravity(VectorXd::Zero(n_u_));
        gravity[2] = 5;
        u_target.push_back(gravity);
        u_target.push_back(gravity);
      }
    }
    c3_->UpdateInputTarget(u_target);
  }

  if (ic3_options_.add_position_constraints) {
    // HARD CODED
    MatrixXd A(MatrixXd::Zero(n_x_, n_x_));
    A(0, 0) = 1;
    A(1, 1) = 1;
    A(2, 2) = 1;
    A(3, 3) = 1;
    A(4, 4) = 1;

    VectorXd lower_bound(VectorXd::Zero(n_x_));
    VectorXd upper_bound(VectorXd::Zero(n_x_));

    // Plate position constraints
    lower_bound[0] = -0.1;
    lower_bound[1] = -0.1;
    lower_bound[2] = -0.3; 
    lower_bound[3] = -0.4;
    lower_bound[4] = -0.4;
  
    // Plate rotation constraints
    upper_bound[0] = 0.1;
    upper_bound[1] = 0.1;
    upper_bound[2] = 0.3;
    upper_bound[3] = 0.4;
    upper_bound[4] = 0.4;

    c3_->AddVectorLinearConstraint(A, lower_bound, upper_bound, 1);
    
  }
  
  if (ic3_options_.add_input_constraints) {
    // HARD CODED
    MatrixXd A_u(MatrixXd::Zero(n_u_, n_u_));
    A_u(0, 0) = 1;
    A_u(1, 1) = 1;
    A_u(2, 2) = 1;
    A_u(3, 3) = 1;
    A_u(4, 4) = 1;

    VectorXd lower_bound_u(VectorXd::Zero(n_u_));
    VectorXd upper_bound_u(VectorXd::Zero(n_u_));

    lower_bound_u << -1, -1, 3, -0.6, -0.6;
    upper_bound_u << 1, 1, 7, 0.6, 0.6;

    c3_->AddVectorLinearConstraint(A_u, lower_bound_u, upper_bound_u, 2);    
   
  }
  c3_->UpdateLCS(lcs);

  auto c3_start = std::chrono::high_resolution_clock::now();
  c3_->Solve(x_lcs);

  auto finish = std::chrono::high_resolution_clock::now();
  auto elapsed = finish - start;
  double solve_time =
      std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() /
      1e6;
  double c3_solve_time =
      std::chrono::duration_cast<std::chrono::microseconds>(finish - c3_start).count() /
      1e6;
  
  mutable_solve_time[0] = (1 - solve_time_filter_constant_) * solve_time +
                          solve_time_filter_constant_ * mutable_solve_time[0];
  if (c3_options_.publish_frequency > 0) {
    solve_time = 1.0 / c3_options_.publish_frequency;
    mutable_solve_time[0] = solve_time;
  }

  auto z_sol = c3_->GetFullSolution();
  if (mutable_solve_time[0] < (N_ - 1) * dt_) {
    int index = mutable_solve_time[0] / dt_;
    double weight = (mutable_solve_time[0] - index * dt_) / dt_;
    mutable_x_pred = (1 - weight) * z_sol[index].segment(0, n_x_) +
                     weight * z_sol[index + 1].segment(0, n_x_);
  } else {
    mutable_x_pred = z_sol[N_ - 1].segment(0, n_x_);
  }

  std::cout << "c3 solve time: " << c3_solve_time << std::endl;

  if (ic3_timestep >= 0 && ic3_timestep <= ic3_options_.N) {
    //std::cout << "ic3_timestep: " << ic3_timestep << ": ";
    //std::cout << c3_->GetInputSolution()[0].transpose() << std::endl;    
  }

  // std::cout << "c3 sol: " << z_sol[0].transpose() << std::endl;
  // std::cout << "c3 sol: " << z_sol[1].transpose() << std::endl;
  // std::cout << "c3 sol: " << z_sol[2].transpose() << std::endl;
  // std::cout << "c3 sol: " << z_sol[3].transpose() << std::endl;
  // std::cout << "c3 sol: " << z_sol[4].transpose() << std::endl;

  return drake::systems::EventStatus::Succeeded();
}

void iC3TrackingController::UpdateQuaternionCosts(
    const Eigen::VectorXd& x_curr, const Eigen::VectorXd& x_des) const {
    
  Q_.clear();
  R_.clear();
  G_.clear();
  U_.clear();

  double discount_factor = 1;
  for (int i = 0; i < N_; i++) {
    Q_.push_back(discount_factor * c3_options_.Q);
    discount_factor *=  c3_options_.gamma;
    if (i < N_) {
      R_.push_back(discount_factor *  c3_options_.R);
      G_.push_back(c3_options_.G);
      U_.push_back(c3_options_.U);
    }
  }  
  Q_.push_back(discount_factor * c3_options_.Q); 

  int index = 5;
      
  Eigen::VectorXd quat_curr_i = x_curr.segment(index, 4);
  Eigen::VectorXd quat_des_i = x_des.segment(index, 4);

  Eigen::MatrixXd quat_hessian_i = hessian_of_squared_quaternion_angle_difference(quat_curr_i, quat_des_i);

  // Regularize hessian so Q is always PSD
  double min_eigenval = quat_hessian_i.eigenvalues().real().minCoeff();
  Eigen::MatrixXd quat_regularizer_1 = std::max(0.0, -min_eigenval) * Eigen::MatrixXd::Identity(4, 4);
  Eigen::MatrixXd quat_regularizer_2 = quat_des_i * quat_des_i.transpose();
  Eigen::MatrixXd quat_regularizer_3 = 1e-8 * Eigen::MatrixXd::Identity(4, 4);

  double Q_quaternion_weight = 5000;
  double quaternion_regularizer_fraction = 0.0;

  discount_factor = 1;
  for (int i = 0; i < N_ + 1; i++) {
    Q_[i].block(index, index, 4, 4) = 
      discount_factor * Q_quaternion_weight * 
      (quat_hessian_i + quat_regularizer_1 + 
        quaternion_regularizer_fraction * quat_regularizer_2 + quat_regularizer_3);
    discount_factor *= c3_options_.gamma;

    double Q_min_eigenval = Q_[i].eigenvalues().real().minCoeff();
  }
  
}

void iC3TrackingController::OutputC3Solution(
    const drake::systems::Context<double>& context,
    C3Output::C3Solution* c3_solution) const {
  double t = context.get_discrete_state(plan_start_time_index_)[0];
  double solve_time = context.get_discrete_state(filtered_solve_time_index_)[0];

  auto z_sol = c3_->GetFullSolution();
  for (int i = 0; i < N_; i++) {
    c3_solution->time_vector_(i) = solve_time + t + i * dt_;
    c3_solution->x_sol_.col(i) = z_sol[i].segment(0, n_x_).cast<float>();
    c3_solution->lambda_sol_.col(i) =
        z_sol[i].segment(n_x_, n_lambda_).cast<float>();
    c3_solution->u_sol_.col(i) =
        z_sol[i].segment(n_x_ + n_lambda_, n_u_).cast<float>();

  } 
}

void iC3TrackingController::OutputC3Intermediates(
    const drake::systems::Context<double>& context,
    C3Output::C3Intermediates* c3_intermediates) const {
  double solve_time = context.get_discrete_state(filtered_solve_time_index_)[0];
  double t = context.get_discrete_state(plan_start_time_index_)[0] + solve_time;
  auto z = c3_->GetFullSolution();
  auto delta = c3_->GetDualDeltaSolution();
  auto w = c3_->GetDualWSolution();

  for (int i = 0; i < N_; i++) {
    c3_intermediates->time_vector_(i) = solve_time + t + i * dt_;
    c3_intermediates->z_.col(i) = z[i].cast<float>();
    c3_intermediates->delta_.col(i) = delta[i].cast<float>();
    c3_intermediates->w_.col(i) = w[i].cast<float>();
  }
}

drake::systems::EventStatus iC3TrackingController::SetFirstCallTime(
    const drake::systems::Context<double>& context,
    drake::systems::DiscreteValues<double>* discrete_state) const {
  auto& vec = discrete_state->get_mutable_vector(t0_idx_);
  if (!called_) {  
    vec.SetAtIndex(0, context.get_time());
		called_ = true;
  }
  return drake::systems::EventStatus::Succeeded();
}


}  // namespace systems
}  // namespace dairlib
