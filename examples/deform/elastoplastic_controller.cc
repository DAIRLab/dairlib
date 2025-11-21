#include "examples/deform/elastoplastic_controller.h"

#include <Eigen/Dense>
#include <drake/common/trajectories/piecewise_polynomial.h>

#include "common/update_context.h"
#include "dairlib/lcmt_force.hpp"
#include "systems/framework/timestamped_vector.h"

namespace dairlib {
namespace examples {
namespace deform {

using dairlib::solvers::LCS;
using drake::SortedPair;
using drake::geometry::GeometryId;
using drake::multibody::JointActuatorIndex;
using drake::systems::BasicVector;
using drake::trajectories::PiecewisePolynomial;
using Eigen::MatrixXd;
using Eigen::MatrixXf;
using Eigen::VectorXd;
using Eigen::VectorXf;
using solvers::C3Plus;
using solvers::LCS;
using solvers::LCSFactory;
using std::vector;
using systems::TimestampedVector;

ElastoPlasticController::ElastoPlasticController(
    drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context,
    drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad,
    drake::systems::Context<drake::AutoDiffXd>* context_ad,
    const vector<vector<SortedPair<GeometryId>>>& contact_geoms,
    const ElastoPlasticC3Options& elastoplastic_c3_options)
    : plant_(plant),
      context_(context),
      plant_ad_(plant_ad),
      context_ad_(context_ad),
      contact_pairs_(contact_geoms),
      elastoplastic_c3_options_(std::move(elastoplastic_c3_options)),
      N_(elastoplastic_c3_options.N),
      dt_(elastoplastic_c3_options.dt),
      solve_time_filter_alpha_(
          elastoplastic_c3_options.solve_time_filter_alpha) {
  this->set_name("elastoplastic_controller");

  n_q_ = plant_.num_positions();
  n_v_ = plant_.num_velocities();
  n_u_ = plant_.num_actuators();
  n_x_ = n_q_ + n_v_;

  // TODO @bibit change this size based on robot model
  teleop_target_.resize(n_u_);

  // Determine n_lambda_ from the contact model.
  if (elastoplastic_c3_options_.contact_model == "stewart_and_trinkle") {
    contact_model_ = solvers::ContactModel::kStewartAndTrinkle;
    n_lambda_ = 2 * elastoplastic_c3_options_.num_contacts +
                2 * elastoplastic_c3_options_.num_friction_directions *
                    elastoplastic_c3_options_.num_contacts;
  } else if (elastoplastic_c3_options_.contact_model == "anitescu") {
    contact_model_ = solvers::ContactModel::kAnitescu;
    n_lambda_ = 2 * elastoplastic_c3_options_.num_friction_directions *
                elastoplastic_c3_options_.num_contacts;
  } else {
    std::cerr << "Unknown or unsupported contact model: "
              << elastoplastic_c3_options_.contact_model << std::endl;
    DRAKE_THROW_UNLESS(false);
  }

  // Initialize cost matrices.
  double discount_factor = 1;
  C3Options c3_options = elastoplastic_c3_options_.GetC3Options();
  for (int i = 0; i < N_ + 1; ++i) {
    Q_.push_back(discount_factor * c3_options.Q);
    if (i < N_) {
      R_.push_back(discount_factor * c3_options.R);
      G_.push_back(c3_options.G);
      U_.push_back(c3_options.U);
    } else {
      Q_.back() *= elastoplastic_c3_options_.w_Q_final;
    }
    discount_factor *= c3_options.gamma;
  }

  // Create placeholder LCS for initializing C3+.
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
  x_lcs_input_port_ =
      this->DeclareVectorInputPort("x_lcs", TimestampedVector<double>(n_x_))
          .get_index();
  x_lcs_target_input_port_ =
      this->DeclareVectorInputPort("x_lcs_des", n_x_).get_index();
  radio_input_port_ = this->DeclareAbstractInputPort(
                              "radio", drake::Value<dairlib::lcmt_radio_out>{})
                          .get_index();

  // Output ports
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
  auto lcs_contact_jacobian = std::pair(Eigen::MatrixXd(n_x_, n_lambda_),
                                        std::vector<Eigen::VectorXd>());
  c3_solution_port_ = this->DeclareAbstractOutputPort(
                              "c3_solution", c3_solution,
                              &ElastoPlasticController::OutputC3Solution)
                          .get_index();
  c3_intermediates_port_ =
      this->DeclareAbstractOutputPort(
              "c3_intermediates", c3_intermediates,
              &ElastoPlasticController::OutputC3Intermediates)
          .get_index();
  c3_forces_port_ =
      this->DeclareAbstractOutputPort("c3_forces", dairlib::lcmt_c3_forces(),
                                      &ElastoPlasticController::OutputC3Forces)
          .get_index();
  efforts_port_ = this->DeclareAbstractOutputPort(
                          "efforts", dairlib::lcmt_robot_input(),
                          &ElastoPlasticController::OutputRobotEfforts)
                      .get_index();
  c3_costs_port_ =
      this->DeclareAbstractOutputPort("c3_costs", dairlib::lcmt_c3_costs(),
                                      &ElastoPlasticController::OutputC3Costs)
          .get_index();

  // Discrete state indices
  plan_start_time_index_ = DeclareDiscreteState(1);
  x_pred_index_ = DeclareDiscreteState(n_x_);
  filtered_solve_time_index_ = DeclareDiscreteState(1);

  this->DeclareForcedDiscreteUpdateEvent(&ElastoPlasticController::ComputePlan);
}

drake::systems::EventStatus ElastoPlasticController::ComputePlan(
    const drake::systems::Context<double>& context,
    drake::systems::DiscreteValues<double>* discrete_state) const {
  auto start = std::chrono::high_resolution_clock::now();

  // Evaluate input ports.
  const TimestampedVector<double>* lcs_x_curr =
      (TimestampedVector<double>*)this->EvalVectorInput(context,
                                                        x_lcs_input_port_);
  const BasicVector<double>& lcs_x_des =
      *this->template EvalVectorInput<BasicVector>(context,
                                                   x_lcs_target_input_port_);
  drake::VectorX<double> x_lcs_des = lcs_x_des.get_value();
  drake::VectorX<double> x_lcs_curr = lcs_x_curr->get_data();
  double t_context = lcs_x_curr->get_timestamp();
  discrete_state->get_mutable_value(plan_start_time_index_)[0] = t_context;

  // Resolve contact pairs and create LCS.
  UpdateContext(n_q_, n_v_, n_u_, plant_, context_, plant_ad_, context_ad_,
                x_lcs_curr);
  vector<SortedPair<GeometryId>> resolved_contact_pairs =
      LCSFactory::PreProcessor(
          plant_, *context_, contact_pairs_,
          elastoplastic_c3_options_.resolve_contacts_to,
          elastoplastic_c3_options_.num_friction_directions, false);
  LCS lcs_object = LCSFactory::LinearizePlantToLCS(
      plant_, *context_, plant_ad_, *context_ad_, resolved_contact_pairs,
      elastoplastic_c3_options_.mu, dt_, N_,
      elastoplastic_c3_options_.n_lambda_with_tangential,
      elastoplastic_c3_options_.num_friction_directions_per_contact,
      elastoplastic_c3_options_.starting_index_per_contact_in_lambda_t_vector,
      contact_model_);

  // Solve C3.
  std::vector<VectorXd> x_desired = std::vector<VectorXd>(N_ + 1, x_lcs_des);
  c3_mpc_->UpdateLCS(lcs_object);
  c3_mpc_->UpdateTarget(x_desired);
  c3_mpc_->Solve(x_lcs_curr, false);  // verbose off

  // End of control loop cleanup:  update filtered solve time.
  double old_solve_time =
      discrete_state->get_value(filtered_solve_time_index_)[0];
  auto finish = std::chrono::high_resolution_clock::now();
  auto elapsed = finish - start;
  double solve_time =
      std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() /
      1e6;

  double new_solve_time = (1 - solve_time_filter_alpha_) * solve_time +
                          (solve_time_filter_alpha_)*old_solve_time;
  discrete_state->get_mutable_value(filtered_solve_time_index_)[0] =
      new_solve_time;

  return drake::systems::EventStatus::Succeeded();
}

// Output port functions
void ElastoPlasticController::OutputC3Solution(
    const drake::systems::Context<double>& context,
    C3Output::C3Solution* c3_solution) const {
  double plan_start_time =
      context.get_discrete_state(plan_start_time_index_)[0];
  double filtered_solve_time =
      context.get_discrete_state(filtered_solve_time_index_)[0];
  double t0 = plan_start_time + filtered_solve_time;

  auto z_sol = c3_mpc_->GetFullSolution();
  for (int i = 0; i < N_; i++) {
    c3_solution->time_vector_(i) = t0 + i * dt_;
    c3_solution->x_sol_.col(i) = z_sol[i].segment(0, n_x_).cast<float>();
    c3_solution->lambda_sol_.col(i) =
        z_sol[i].segment(n_x_, n_lambda_).cast<float>();
    c3_solution->u_sol_.col(i) =
        z_sol[i].segment(n_x_ + n_lambda_, n_u_).cast<float>();
  }
}

void ElastoPlasticController::OutputC3Intermediates(
    const drake::systems::Context<double>& context,
    C3Output::C3Intermediates* c3_intermediates) const {
  double plan_start_time =
      context.get_discrete_state(plan_start_time_index_)[0];
  double filtered_solve_time =
      context.get_discrete_state(filtered_solve_time_index_)[0];
  double t0 = plan_start_time + filtered_solve_time;

  auto z_sol = c3_mpc_->GetFullSolution();
  auto delta = c3_mpc_->GetDualDeltaSolution();
  auto w = c3_mpc_->GetDualWSolution();

  for (int i = 0; i < N_; i++) {
    c3_intermediates->time_vector_(i) = t0 + i * dt_;
    c3_intermediates->z_.col(i) = z_sol[i].cast<float>();
    c3_intermediates->w_.col(i) = w[i].cast<float>();
    c3_intermediates->delta_.col(i) = delta[i].cast<float>();
  }
}

void ElastoPlasticController::OutputC3Forces(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_c3_forces* lcmt_c3_forces) const {
  // Preprocess the contact pairs.
  const TimestampedVector<double>* lcs_x =
      (TimestampedVector<double>*)this->EvalVectorInput(context,
                                                        x_lcs_input_port_);
  UpdateContext(n_q_, n_v_, n_u_, plant_, context_, plant_ad_, context_ad_,
                lcs_x->get_data());
  vector<SortedPair<GeometryId>> resolved_contact_pairs;
  resolved_contact_pairs = LCSFactory::PreProcessor(
      plant_, *context_, contact_pairs_,
      elastoplastic_c3_options_.resolve_contacts_to,
      elastoplastic_c3_options_.num_friction_directions);

  // Grab the latest solve's contact forces.
  std::vector<VectorXd> lambda_sol = c3_mpc_->GetForceSolution();
  VectorXd lcs_forces = lambda_sol[0];

  // Start constructing the world frame contact forces.
  int num_contacts = resolved_contact_pairs.size();
  int cur_lambda_index = 0;

  lcmt_c3_forces->num_forces = num_contacts;
  lcmt_c3_forces->forces.resize(num_contacts);

  DRAKE_DEMAND(contact_model_ == solvers::ContactModel::kAnitescu);

  for (int i = 0; i < num_contacts; i++) {
    multibody::GeomGeomCollider collider(plant_, resolved_contact_pairs[i]);
    int num_force_basis =
        2 * elastoplastic_c3_options_.num_friction_directions_per_contact[i];
    bool is_planar_contact = num_force_basis == 2;
    auto [p_WCa, force_basis] =
        collider.CalcWitnessPointsAndForceBasisInWorldFrame(*context_,
                                                            is_planar_contact);
    // Reduce force_basis to Anitescu force basis.
    Eigen::Matrix<double, 4, 3> anitescu_force_basis;
    for (int j = 1; j < 5; j++) {
      anitescu_force_basis.row(j - 1) =
          force_basis.row(0) +
          elastoplastic_c3_options_.mu[i] * force_basis.row(j);
    }

    auto force_in_world_frame =
        anitescu_force_basis.transpose() *
        lcs_forces.segment(cur_lambda_index, num_force_basis);
    auto net_force = force_in_world_frame.rowwise().sum();

    cur_lambda_index += num_force_basis;

    auto force = dairlib::lcmt_force();
    force.contact_point[0] = p_WCa[0];
    force.contact_point[1] = p_WCa[1];
    force.contact_point[2] = p_WCa[2];
    force.contact_force[0] = net_force[0];
    force.contact_force[1] = net_force[1];
    force.contact_force[2] = net_force[2];
    lcmt_c3_forces->forces[i] = force;
  }
}

void ElastoPlasticController::OutputRobotEfforts(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_robot_input* lcmt_efforts) const {
  const TimestampedVector<double>* lcs_x_curr =
      (TimestampedVector<double>*)this->EvalVectorInput(context,
                                                        x_lcs_input_port_);
  drake::VectorX<double> x_lcs_curr = lcs_x_curr->get_data();

  // Begin constructing the efforts message.
  lcmt_efforts->utime = context.get_time() * 1e6;
  lcmt_efforts->num_efforts = n_u_;

  // Set the actuator names according to the plant.
  std::vector<std::string> actuator_names;
  for (JointActuatorIndex i(0); i < n_u_; ++i) {
    const drake::multibody::JointActuator<double>& actuator =
        plant_.get_joint_actuator(i);
    actuator_names.push_back(actuator.name());
  }
  lcmt_efforts->effort_names = actuator_names;

  // Begin determining the desired location to track.
  VectorXd q_des = VectorXd::Zero(n_u_);
  VectorXd v_des = VectorXd::Zero(n_u_);
  VectorXd u_feedforward = VectorXd::Zero(n_u_);

  // If in teleop mode, keep the robot in place.
  const auto& lcmt_radio =
      this->EvalInputValue<dairlib::lcmt_radio_out>(context, radio_input_port_);
  if (lcmt_radio->channel[kTeleopRadioChannel] == 1) {
    if (!was_teleop_last_step_) {
      std::cout << "Newly entering teleop!" << std::endl;
      teleop_target_.head(n_u_) = x_lcs_curr.head(n_u_);
      was_teleop_last_step_ = true;
    }
    q_des = teleop_target_;
  }
  // If not in teleop mode, track the C3 plan.
  else {
    was_teleop_last_step_ = false;

    // Perform closed-loop feedback on the EE state with C3 feed-forward term.
    // Get the C3 plan.
    std::vector<VectorXd> u_sol = c3_mpc_->GetInputSolution();
    std::vector<VectorXd> x_sol = c3_mpc_->GetStateSolution();
    u_feedforward = u_sol[0];

    // Define a state trajectory so we can interpolate it for the feedback term.
    Eigen::MatrixXd x_knots = Eigen::MatrixXd::Zero(n_x_, N_);
    Eigen::VectorXd timestamps = Eigen::VectorXd::Zero(N_);
    for (int i = 0; i < N_; i++) {
      x_knots.col(i) = x_sol[i];
      timestamps[i] = context.get_time() + i * dt_;
    }
    auto x_trajectory =
        PiecewisePolynomial<double>::FirstOrderHold(timestamps, x_knots);
    double solve_time =
        context.get_discrete_state(filtered_solve_time_index_)[0];
    VectorXd x_des = x_trajectory.value(context.get_time() + solve_time);
    q_des = x_des.head(n_u_);
    v_des = x_des.segment(n_q_, n_u_);
  }

  // Set the efforts according to feedforward + feedback.  Use a predicted next
  // state for feedback.
  std::vector<double> efforts_vector(n_u_);
  for (int i = 0; i < n_u_; i++) {
    double u_feedback =
        elastoplastic_c3_options_.Kp[i] * (q_des[i] - x_lcs_curr(i)) +
        elastoplastic_c3_options_.Kd[i] * (v_des[i] - x_lcs_curr(i + n_q_));
    efforts_vector[i] = u_feedforward[i] + u_feedback;
    // std::clamp(u_feedforward + u_feedback,
    //            elastoplastic_c3_options_.u_horizontal_limits[0],
    //            elastoplastic_c3_options_.u_horizontal_limits[1]);
    std::cout << "Feedforward = " << u_feedforward[i]
              << ", Feedback = " << u_feedback
              << ", Total = " << efforts_vector[i] << std::endl;
  }
  lcmt_efforts->efforts = efforts_vector;
}

void ElastoPlasticController::OutputC3Costs(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_c3_costs* lcmt_c3_costs) const {
  const BasicVector<double>& lcs_x_des =
      *this->template EvalVectorInput<BasicVector>(context,
                                                   x_lcs_target_input_port_);
  drake::VectorX<double> x_lcs_des = lcs_x_des.get_value();
  std::vector<VectorXd> x_desired = std::vector<VectorXd>(N_ + 1, x_lcs_des);

  // Get the C3 plan.
  std::vector<VectorXd> u_sol = c3_mpc_->GetInputSolution();
  std::vector<VectorXd> x_sol = c3_mpc_->GetStateSolution();

  lcmt_c3_costs->utime = context.get_time() * 1e6;
  lcmt_c3_costs->num_cost_chunks = 7;
  lcmt_c3_costs->cost_names = std::vector<std::string>{
      "all",           "state",     "inputs",    "robot_config",
      "object_config", "robot_vel", "object_vel"};

  double state_cost = 0.0;
  double input_cost = 0.0;
  double robot_config_cost = 0.0;
  double object_config_cost = 0.0;
  double robot_vel_cost = 0.0;
  double object_vel_cost = 0.0;

  for (int i = 0; i < N_; i++) {
    VectorXd x_err = x_sol[i] - x_desired[i];
    state_cost += x_err.transpose() * Q_[i] * x_err;
    input_cost += u_sol[i].transpose() * R_[i] * u_sol[i];

    // TODO @bibit:  this hard-coded indexing assumes robot has 1 DoF.
    robot_config_cost += x_err.segment(0, 1).transpose() *
                         Q_[i].block(0, 0, 1, 1) * x_err.segment(0, 1);
    object_config_cost += x_err.segment(1, n_q_ - 1).transpose() *
                          Q_[i].block(1, 1, n_q_ - 1, n_q_ - 1) *
                          x_err.segment(1, n_q_ - 1);
    robot_vel_cost += x_err.segment(n_q_, 1).transpose() *
                      Q_[i].block(n_q_, n_q_, 1, 1) * x_err.segment(n_q_, 1);
    object_vel_cost +=
        x_err.segment(n_q_ + n_v_ - 1, n_v_ - 1).transpose() *
        Q_[i].block(n_q_ + n_v_ - 1, n_q_ + n_v_ - 1, n_v_ - 1, n_v_ - 1) *
        x_err.segment(n_q_ + n_v_ - 1, n_v_ - 1);
  }

  lcmt_c3_costs->total_costs = std::vector<float>{
      static_cast<float>(state_cost + input_cost),
      static_cast<float>(state_cost),
      static_cast<float>(input_cost),
      static_cast<float>(robot_config_cost),
      static_cast<float>(object_config_cost),
      static_cast<float>(robot_vel_cost),
      static_cast<float>(object_vel_cost),
  };
}

}  // namespace deform
}  // namespace examples
}  // namespace dairlib
