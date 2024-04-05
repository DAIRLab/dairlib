#include "c3_controller.h"

#include <utility>

#include "multibody/multibody_utils.h"
#include "solvers/c3_miqp.h"
#include "solvers/c3_qp.h"
#include "solvers/lcs_factory.h"

namespace dairlib {

using drake::multibody::ModelInstanceIndex;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteValues;
using Eigen::MatrixXd;
using Eigen::MatrixXf;
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

C3Controller::C3Controller(
    const drake::multibody::MultibodyPlant<double>& plant, C3Options c3_options)
    : plant_(plant),
      c3_options_(std::move(c3_options)),
      G_(std::vector<MatrixXd>(c3_options_.N, c3_options_.G)),
      U_(std::vector<MatrixXd>(c3_options_.N, c3_options_.U)),
      N_(c3_options_.N) {
  this->set_name("c3_controller");

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
  }
  VectorXd zeros = VectorXd::Zero(n_x_ + n_lambda_ + n_u_);

  n_u_ = plant_.num_actuators();

  // Creates placeholder lcs to construct base C3 problem
  // Placeholder LCS will have correct size as it's already determined by the
  // contact model
  auto lcs_placeholder = CreatePlaceholderLCS();
  auto x_desired_placeholder =
      std::vector<VectorXd>(N_ + 1, VectorXd::Zero(n_x_));
  auto cost_matrices_placeholder = CreatePlaceholderCostMatrices();
  if (c3_options_.projection_type == "MIQP") {
    c3_ = std::make_unique<C3MIQP>(lcs_placeholder,
                                   C3::CostMatrices(Q_, R_, G_, U_),
                                   x_desired_placeholder, c3_options_);

  } else if (c3_options_.projection_type == "QP") {
    c3_ = std::make_unique<C3QP>(lcs_placeholder,
                                 C3::CostMatrices(Q_, R_, G_, U_),
                                 x_desired_placeholder, c3_options_);

  } else {
    std::cerr << ("Unknown projection type") << std::endl;
    DRAKE_THROW_UNLESS(false);
  }

  c3_->SetOsqpSolverOptions(solver_options_);

  // Set actor bounds,
  // TODO(yangwill): move this out of here because it is task specific
  for (int i = 0; i < c3_options_.workspace_limits.size(); ++i) {
    Eigen::RowVectorXd A = VectorXd::Zero(n_x_);
    A.segment(0, 3) = c3_options_.workspace_limits[i].segment(0, 3);
    c3_->AddLinearConstraint(A, c3_options_.workspace_limits[i][3],
                             c3_options_.workspace_limits[i][4], 1);
  }
  for (int i : vector<int>({0, 1})) {
    Eigen::RowVectorXd A = VectorXd::Zero(n_u_);
    A(i) = 1.0;
    c3_->AddLinearConstraint(A, c3_options_.u_horizontal_limits[0],
                             c3_options_.u_horizontal_limits[1], 2);
  }
  for (int i : vector<int>({2})) {
    Eigen::RowVectorXd A = VectorXd::Zero(n_u_);
    A(i) = 1.0;
    c3_->AddLinearConstraint(A, c3_options_.u_vertical_limits[0],
                             c3_options_.u_vertical_limits[1], 2);
  }

  lcs_state_input_port_ =
      this->DeclareVectorInputPort("x_lcs", TimestampedVector<double>(n_x_))
          .get_index();
  lcs_input_port_ =
      this->DeclareAbstractInputPort("lcs", drake::Value<LCS>(lcs_placeholder))
          .get_index();
  target_input_port_ =
      this->DeclareVectorInputPort("x_lcs_des", n_x_)
          .get_index();
  cost_matrices_input_port_ =
      this->DeclareAbstractInputPort("c3_cost_matrices", drake::Value<C3::CostMatrices>(cost_matrices_placeholder))
          .get_index();

  auto c3_solution = C3Output::C3Solution();
  c3_solution.x_sol_ = MatrixXf::Zero(n_q_ + n_v_, N_);
  c3_solution.lambda_sol_ = MatrixXf::Zero(n_lambda_, N_);
  c3_solution.u_sol_ = MatrixXf::Zero(n_u_, N_);
  c3_solution.time_vector_ = VectorXf::Zero(N_);
  auto c3_intermediates = C3Output::C3Intermediates();
  c3_intermediates.w_ = MatrixXf::Zero(n_x_ + n_lambda_ + n_u_, N_);
  c3_intermediates.delta_ = MatrixXf::Zero(n_x_ + n_lambda_ + n_u_, N_);
  c3_intermediates.time_vector_ = VectorXf::Zero(N_);
  c3_solution_port_ =
      this->DeclareAbstractOutputPort("c3_solution", c3_solution,
                                      &C3Controller::OutputC3Solution)
          .get_index();
  c3_intermediates_port_ =
      this->DeclareAbstractOutputPort("c3_intermediates", c3_intermediates,
                                      &C3Controller::OutputC3Intermediates)
          .get_index();

  solve_time_port_ =
      this->DeclareVectorOutputPort("solve_time", BasicVector<double>(1),
                                            &C3Controller::OutputC3Solvetime)
          .get_index();

  plan_start_time_index_ = DeclareDiscreteState(1);
  x_pred_index_ = DeclareDiscreteState(n_x_);
  filtered_solve_time_index_ = DeclareDiscreteState(1);

  if (c3_options_.publish_frequency > 0) {
    DeclarePeriodicDiscreteUpdateEvent(1 / c3_options_.publish_frequency, 0.0,
                                       &C3Controller::ComputePlan);
  } else {
    DeclareForcedDiscreteUpdateEvent(&C3Controller::ComputePlan);
  }
}

LCS C3Controller::CreatePlaceholderLCS() const {
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

C3::CostMatrices C3Controller::CreatePlaceholderCostMatrices() const {
    std::vector<MatrixXd> Q(N_+1, MatrixXd::Zero(n_x_, n_x_));
    std::vector<MatrixXd> R(N_, MatrixXd::Zero(n_u_, n_u_));
    std::vector<MatrixXd> G(N_, MatrixXd::Zero(n_x_ + n_u_ + n_lambda_, n_x_ + n_u_ + n_lambda_));
    std::vector<MatrixXd> U(N_, MatrixXd::Zero(n_x_ + n_u_ + n_lambda_, n_x_ + n_u_ + n_lambda_));
    return C3::CostMatrices(Q, R, G, U);
}

drake::systems::EventStatus C3Controller::ComputePlan(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {

  auto start = std::chrono::high_resolution_clock::now();
  const BasicVector<double>& x_des =
      *this->template EvalVectorInput<BasicVector>(context, target_input_port_);
  const TimestampedVector<double>* lcs_x =
      (TimestampedVector<double>*)this->EvalVectorInput(context,
                                                        lcs_state_input_port_);
  auto& lcs =
      this->EvalAbstractInput(context, lcs_input_port_)->get_value<LCS>();
  drake::VectorX<double> x_lcs = lcs_x->get_data();
  auto& cost_matrices = this->EvalAbstractInput(context, cost_matrices_input_port_)
          ->get_value<C3::CostMatrices>();

  auto& x_pred = context.get_discrete_state(x_pred_index_).value();
  auto mutable_x_pred = discrete_state->get_mutable_value(x_pred_index_);
  auto mutable_solve_time =
      discrete_state->get_mutable_value(filtered_solve_time_index_);

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
  // Force Checking of Workspace Limits
  for (int i = 0; i < c3_options_.workspace_limits.size(); ++i) {
    DRAKE_DEMAND(lcs_x->get_data().segment(0, 3).transpose() *
                     c3_options_.workspace_limits[i].segment(0, 3) >
                 c3_options_.workspace_limits[i][3] -
                     c3_options_.workspace_margins);
    DRAKE_DEMAND(lcs_x->get_data().segment(0, 3).transpose() *
                     c3_options_.workspace_limits[i].segment(0, 3) <
                 c3_options_.workspace_limits[i][4] +
                     c3_options_.workspace_margins);
  }

  c3_->UpdateLCS(lcs);
  c3_->UpdateCostMatrices(cost_matrices);
  c3_->UpdateTarget(x_desired);
  c3_->Solve(x_lcs);

  auto finish = std::chrono::high_resolution_clock::now();
  auto elapsed = finish - start;
  double solve_time =
      std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() /
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
    mutable_x_pred = z_sol[-1].segment(0, n_x_);
  }

  return drake::systems::EventStatus::Succeeded();
}

void C3Controller::OutputC3Solution(
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

void C3Controller::OutputC3Intermediates(
    const drake::systems::Context<double>& context,
    C3Output::C3Intermediates* c3_intermediates) const {
  double solve_time = context.get_discrete_state(filtered_solve_time_index_)[0];
  double t = context.get_discrete_state(plan_start_time_index_)[0] + solve_time;
  auto delta = c3_->GetDualDeltaSolution();
  auto w = c3_->GetDualWSolution();

  for (int i = 0; i < N_; i++) {
    c3_intermediates->time_vector_(i) = solve_time + t + i * dt_;
    c3_intermediates->w_.col(i) = delta[i].cast<float>();
    c3_intermediates->delta_.col(i) = w[i].cast<float>();
  }
}

void C3Controller::OutputC3Solvetime(const drake::systems::Context<double> &context,
                                     drake::systems::BasicVector<double> *solve_time) const {
    VectorXd filtered_solve_time = VectorXd::Zero(1);
    filtered_solve_time << context.get_discrete_state(filtered_solve_time_index_)[0];
    solve_time->SetFromVector(filtered_solve_time);
}
}  // namespace systems
}  // namespace dairlib
