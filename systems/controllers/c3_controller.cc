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
using solvers::C3MIQP;
using solvers::C3QP;
using solvers::LCS;
using solvers::LCSFactory;
using std::vector;
using systems::TimestampedVector;

namespace systems {

C3Controller::C3Controller(
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context,
    C3Options c3_options)
    : plant_(plant),
      context_(context),
      c3_options_(std::move(c3_options)),
      Q_(std::vector<MatrixXd>(c3_options_.N + 1, c3_options_.Q)),
      R_(std::vector<MatrixXd>(c3_options_.N, c3_options_.R)),
      G_(std::vector<MatrixXd>(c3_options_.N, c3_options_.G)),
      U_(std::vector<MatrixXd>(c3_options_.N, c3_options_.U)),
      N_(c3_options_.N) {
  this->set_name("c3_controller");

  n_q_ = plant_.num_positions();
  n_v_ = plant_.num_velocities();
  n_x_ = n_q_ + n_v_;
  dt_ = c3_options_.dt;
  if (c3_options_.contact_model == "stewart_and_trinkle") {
    n_lambda_ =
        2 * c3_options_.num_contacts +
        2 * c3_options_.num_friction_directions * c3_options_.num_contacts;
  } else if (c3_options_.contact_model == "anitescu") {
    n_lambda_ =
        2 * c3_options_.num_friction_directions * c3_options_.num_contacts;
  }

  n_u_ = plant_.num_actuators();
  u_prev_ = VectorXd::Zero(n_u_);

  int x_des_size = plant_.num_positions(ModelInstanceIndex(2)) +
                   plant_.num_positions(ModelInstanceIndex(3)) +
                   plant_.num_velocities(ModelInstanceIndex(2)) +
                   plant_.num_velocities(ModelInstanceIndex(3));
  lcs_state_input_port_ =
      this->DeclareVectorInputPort(
              "x_lcs", TimestampedVector<double>(
                           x_des_size))
          .get_index();

  MatrixXd A = MatrixXd::Zero(n_x_, n_x_);
  MatrixXd B = MatrixXd::Zero(n_x_, n_u_);
  VectorXd d = VectorXd::Zero(n_x_);
  MatrixXd D = MatrixXd::Zero(n_x_, n_lambda_);
  MatrixXd E = MatrixXd::Zero(n_lambda_, n_x_);
  MatrixXd F = MatrixXd::Zero(n_lambda_, n_lambda_);
  MatrixXd H = MatrixXd::Zero(n_lambda_, n_u_);
  VectorXd c = VectorXd::Zero(n_lambda_);
  auto lcs = LCS(A, B, D, d, E, F, H, c, N_, dt_);

  lcs_input_port_ =
      this->DeclareAbstractInputPort("lcs", drake::Value<LCS>(lcs)).get_index();

  target_input_port_ =
      this->DeclareVectorInputPort("x_lcs_des", x_des_size).get_index();

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

  plan_start_time_index_ = DeclareDiscreteState(1);
  DeclareForcedDiscreteUpdateEvent(&C3Controller::ComputePlan);
}

drake::systems::EventStatus C3Controller::ComputePlan(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  auto start = std::chrono::high_resolution_clock::now();

  const BasicVector<double>& x_des =
      *this->template EvalVectorInput<BasicVector>(context, target_input_port_);
  const TimestampedVector<double>* lcs_x =
      (TimestampedVector<double>*)this->EvalVectorInput(
          context, lcs_state_input_port_);
  auto& lcs = this->EvalAbstractInput(context, lcs_input_port_)->get_value<LCS>();

  discrete_state->get_mutable_value(plan_start_time_index_)[0] =
      lcs_x->get_timestamp();


  std::vector<VectorXd> x_desired =
      std::vector<VectorXd>(N_ + 1, x_des.value());


  DRAKE_DEMAND(Q_.front().rows() == lcs.n_);
  DRAKE_DEMAND(Q_.front().cols() == lcs.n_);
  DRAKE_DEMAND(R_.front().rows() == lcs.k_);
  DRAKE_DEMAND(R_.front().cols() == lcs.k_);
  DRAKE_DEMAND(G_.front().rows() == lcs.n_ + lcs.m_ + lcs.k_);
  DRAKE_DEMAND(G_.front().cols() == lcs.n_ + lcs.m_ + lcs.k_);
  DRAKE_DEMAND(U_.front().rows() == lcs.n_ + lcs.m_ + lcs.k_);
  DRAKE_DEMAND(U_.front().cols() == lcs.n_ + lcs.m_ + lcs.k_);

  if (c3_options_.projection_type == "MIQP") {
    c3_ = std::make_unique<C3MIQP>(lcs, Q_, R_, G_, U_, x_desired, c3_options_);

  } else if (c3_options_.projection_type == "QP") {
    c3_ = std::make_unique<C3QP>(lcs, Q_, R_, G_, U_, x_desired, c3_options_);

  } else {
    std::cerr << ("Unknown projection type") << std::endl;
    DRAKE_THROW_UNLESS(false);
  }
  c3_->SetOsqpSolverOptions(solver_options_);

  VectorXd delta_init = VectorXd::Zero(n_x_ + n_lambda_ + n_u_);
  delta_init.head(n_x_) = lcs_x->get_data();
  std::vector<VectorXd> delta(N_, delta_init);
  std::vector<VectorXd> w(N_, VectorXd::Zero(n_x_ + n_lambda_ + n_u_));

  // Set actor bounds
  for (int i : vector<int>({0, 2})) {
    Eigen::RowVectorXd A = VectorXd::Zero(n_x_);
    A(i) = 1.0;
    c3_->AddLinearConstraint(A, 0.35, 0.6, 1);
  }
  for (int i : vector<int>({1})) {
    Eigen::RowVectorXd A = VectorXd::Zero(n_x_);
    A(i) = 1.0;
    c3_->AddLinearConstraint(A, -0.2, 0.2, 1);
  }
  for (int i : vector<int>({0, 1})) {
    Eigen::RowVectorXd A = VectorXd::Zero(n_u_);
    A(i) = 1.0;
    c3_->AddLinearConstraint(A, -10, 10, 2);
  }
  for (int i : vector<int>({2})) {
    Eigen::RowVectorXd A = VectorXd::Zero(n_u_);
    A(i) = 1.0;
    c3_->AddLinearConstraint(A, 0, 20, 2);
  }
  auto z_sol = c3_->Solve(lcs_x->get_data(), delta, w);
  auto finish = std::chrono::high_resolution_clock::now();
  delta_ = delta;
  w_ = w;
  auto elapsed = finish - start;
  solve_time_ =
      std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() /
      1e6;
  return drake::systems::EventStatus::Succeeded();
}

void C3Controller::OutputC3Solution(
    const drake::systems::Context<double>& context,
    C3Output::C3Solution* c3_solution) const {
  double t = context.get_discrete_state(plan_start_time_index_)[0];

  auto z_sol = c3_->GetFullSolution();
  for (int i = 0; i < N_; i++) {
    c3_solution->time_vector_(i) = t + i * c3_options_.dt;
    c3_solution->x_sol_.col(i) = z_sol[i].segment(0, n_x_).cast<float>();
    c3_solution->lambda_sol_.col(i) =
        z_sol[i].segment(n_x_, n_lambda_).cast<float>();
    c3_solution->u_sol_.col(i) =
        z_sol[i].segment(n_x_ + n_lambda_, n_u_).cast<float>();
  }
  u_prev_ = z_sol[0].segment(n_x_ + n_lambda_, n_u_);
}

void C3Controller::OutputC3Intermediates(
    const drake::systems::Context<double>& context,
    C3Output::C3Intermediates* c3_intermediates) const {
  double t = context.get_discrete_state(plan_start_time_index_)[0];

  for (int i = 0; i < N_; i++) {
    c3_intermediates->time_vector_(i) = t + i * c3_options_.dt;
    c3_intermediates->w_.col(i) = w_[i].cast<float>();
    c3_intermediates->delta_.col(i) = delta_[i].cast<float>();
  }
}

}  // namespace systems
}  // namespace dairlib
