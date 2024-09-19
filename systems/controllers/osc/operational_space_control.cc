#include "systems/controllers/osc/operational_space_control.h"

#include <iostream>

#include <drake/multibody/plant/multibody_plant.h>

#include "common/eigen_utils.h"
#include "multibody/multibody_utils.h"

#include "drake/common/text_logging.h"

using std::cout;
using std::endl;

using std::isnan;
using std::numeric_limits;
using std::string;
using std::vector;

using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::VectorXd;



using drake::multibody::JacobianWrtVariable;
using drake::multibody::JointActuatorIndex;
using drake::multibody::JointIndex;
using drake::multibody::MultibodyPlant;
using drake::solvers::MathematicalProgram;
using drake::solvers::MathematicalProgramResult;
using drake::solvers::SolutionResult;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::trajectories::ExponentialPlusPiecewisePolynomial;
using drake::trajectories::PiecewisePolynomial;

using drake::solvers::OsqpSolver;
using drake::solvers::OsqpSolverDetails;
using drake::solvers::Solve;

namespace dairlib::systems::controllers {

using solvers::FCCQPSolver;

using multibody::MakeNameToActuatorsMap;
using multibody::MakeNameToVelocitiesMap;
using multibody::SetPositionsIfNew;
using multibody::SetVelocitiesIfNew;
using multibody::WorldPointEvaluator;

OperationalSpaceControl::OperationalSpaceControl(
    const MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context,
    bool used_with_finite_state_machine,
    OscSolverChoice solver_choice)
    : plant_(plant),
      context_(context),
      used_with_finite_state_machine_(used_with_finite_state_machine),
      solver_choice_(solver_choice),
      id_qp_(plant_, context_) {
  this->set_name("OSC");

  n_q_ = plant.num_positions();
  n_v_ = plant.num_velocities();
  n_u_ = plant.num_actuated_dofs();

  // Input/Output Setup
  state_port_ =
      this->DeclareVectorInputPort(
              "x, u, t", OutputVector<double>(n_q_, n_v_, n_u_))
          .get_index();

  ff_input_port_ = this->DeclareVectorInputPort("udes", n_u_).get_index();
  W_input_ = MatrixXd::Zero(n_u_, n_u_);

  if (used_with_finite_state_machine) {
    fsm_port_ =
        this->DeclareVectorInputPort("fsm", BasicVector<double>(1)).get_index();
    clock_port_ = this->DeclareVectorInputPort("clock", BasicVector<double>(1))
                      .get_index();
    impact_info_port_ =
        this->DeclareVectorInputPort("next_fsm, t_to_impact",
                                     ImpactInfoVector<double>(0, 0, kSpaceDim))
            .get_index();

    // Discrete update to record the last state event time
    DeclarePerStepDiscreteUpdateEvent(
        &OperationalSpaceControl::DiscreteVariableUpdate);
    prev_fsm_state_idx_ = this->DeclareDiscreteState(-0.1 * VectorXd::Ones(1));
    prev_mode_fsm_state_idx_ = this->DeclareDiscreteState(-0.1 * VectorXd::Ones(1));
    prev_event_time_idx_ = this->DeclareDiscreteState(VectorXd::Zero(1));
  }

  osc_output_port_ = this->DeclareVectorOutputPort(
                             "u, t", TimestampedVector<double>(n_u_),
                             &OperationalSpaceControl::CalcOptimalInput)
                         .get_index();
  osc_debug_port_ =
      this->DeclareAbstractOutputPort(
              "lcmt_osc_debug", &OperationalSpaceControl::AssignOscLcmOutput)
          .get_index();

  failure_port_ = this->DeclareVectorOutputPort(
                          "failure_signal", TimestampedVector<double>(1),
                          &OperationalSpaceControl::CheckTracking)
                      .get_index();

  osc_solution_cache_ = DeclareCacheEntry(
      "osc_solution",
      drake::systems::ValueProducer(
          this, &OperationalSpaceControl::AllocateSolution,
          &OperationalSpaceControl::SolveIDQP)).cache_index();

  tracking_data_cache_ = DeclareCacheEntry(
      "tracking_data_states",
      drake::systems::ValueProducer(
          this, &OperationalSpaceControl::AllocateTrackingDataStates,
          &OperationalSpaceControl::UpdateTrackingData)).cache_index();

  const std::map<string, int>& vel_map =
      multibody::MakeNameToVelocitiesMap(plant);

  n_revolute_joints_ = 0;
  for (JointIndex i(0); i < plant_.num_joints(); ++i) {
    const drake::multibody::Joint<double>& joint = plant_.get_joint(i);
    if (joint.type_name() == "revolute") {
      n_revolute_joints_ += 1;
    }
  }
  VectorXd q_min(n_revolute_joints_);
  VectorXd q_max(n_revolute_joints_);
  int floating_base_offset = n_v_ - n_revolute_joints_;
  for (JointIndex i(0); i < plant_.num_joints(); ++i) {
    const drake::multibody::Joint<double>& joint = plant_.get_joint(i);
    if (joint.type_name() == "revolute") {
      q_min(vel_map.at(joint.name() + "dot") - floating_base_offset) =
          plant_.get_joint(i).position_lower_limits()[0];
      q_max(vel_map.at(joint.name() + "dot") - floating_base_offset) =
          plant_.get_joint(i).position_upper_limits()[0];
    }
    if (joint.type_name() == "prismatic" &&
        (joint.position_lower_limits()[0] !=
             -std::numeric_limits<double>::infinity() ||
         (joint.position_upper_limits()[0] !=
          std::numeric_limits<double>::infinity()))) {
      std::cerr << "Warning: joint limits have not been implemented for "
                   "prismatic joints: "
                << std::endl;
    }
  }
  q_min_ = q_min;
  q_max_ = q_max;
}

// Optional features
void OperationalSpaceControl::SetUpDoubleSupportPhaseBlending(
    double ds_duration, int left_support_state, int right_support_state,
    const std::vector<int>& ds_states) {
  DRAKE_DEMAND(ds_duration > 0);
  DRAKE_DEMAND(!ds_states.empty());
  ds_duration_ = ds_duration;
  left_support_state_ = left_support_state;
  right_support_state_ = right_support_state;
  ds_states_ = ds_states;
}

void OperationalSpaceControl::SetInputCostForJointAndFsmStateWeight(
    const std::string& joint_u_name, int fsm, double w) {
  if (W_input_.size() == 0) {
    W_input_ = Eigen::MatrixXd::Zero(n_u_, n_u_);
  }
  int idx = MakeNameToActuatorsMap(plant_).at(joint_u_name);
  fsm_to_w_input_map_[fsm] = std::pair<int, double>{idx, w};
}

// Constraint methods
void OperationalSpaceControl::AddContactPoint(
    const std::string& name,
    std::unique_ptr<const multibody::WorldPointEvaluator<double>> evaluator,
    std::vector<int> fsm_states) {

  DRAKE_DEMAND(not name.empty());

  if (fsm_states.empty()) {
    fsm_states.push_back(-1);
  }
  for (const auto& i : fsm_states) {
    if (contact_names_map_.count(i) > 0) {
      contact_names_map_.at(i).push_back(name);
    } else {
      contact_names_map_.insert({i, {name}});
    }
  }

  contact_name_to_lambda_des_port_map_[name] = DeclareVectorInputPort(
      "lamba_c_des_" + name, 3).get_index();

  DRAKE_DEMAND(mu_ > 0);
  id_qp_.AddContactConstraint(name, std::move(evaluator), mu_);
}

void OperationalSpaceControl::AddKinematicConstraint(
    std::unique_ptr<const multibody::KinematicEvaluatorSet<double>> evaluator) {
  id_qp_.AddHolonomicConstraint(std::move(evaluator));
}

// Tracking data methods
void OperationalSpaceControl::AddTrackingData(
    std::unique_ptr<OscTrackingData> tracking_data, double t_lb, double t_ub) {
  tracking_data_vec_.push_back(std::move(tracking_data));
  fixed_position_vec_.emplace_back(VectorXd::Zero(0));

  // Construct input ports and add element to traj_name_to_port_index_map_ if
  // the port for the traj is not created yet
  string traj_name = tracking_data_vec_.back()->GetName();
  if (traj_name_to_port_index_map_.find(traj_name) ==
      traj_name_to_port_index_map_.end()) {
    PiecewisePolynomial<double> pp = PiecewisePolynomial<double>();
    traj_name_to_port_index_map_[traj_name] = this->DeclareAbstractInputPort(
        traj_name, drake::Value<drake::trajectories::Trajectory<double>>(pp))
            .get_index();
  }
}
void OperationalSpaceControl::AddConstTrackingData(
    std::unique_ptr<OscTrackingData> tracking_data, const VectorXd& v,
    double t_lb, double t_ub) {
  tracking_data_vec_.push_back(std::move(tracking_data));
  fixed_position_vec_.push_back(v);
}

std::unique_ptr<std::vector<OscTrackingDataState>>
OperationalSpaceControl::AllocateTrackingDataStates() const {
  auto states = std::make_unique<std::vector<OscTrackingDataState>>();
  for (const auto& data: tracking_data_vec_) {
    states->push_back(data->AllocateState());
  }
  return states;
}

std::unique_ptr<OperationalSpaceControl::id_qp_solution>
OperationalSpaceControl::AllocateSolution() const {
  DRAKE_DEMAND(this->id_qp_.built());
  std::unique_ptr<id_qp_solution> sol =
      std::make_unique<id_qp_solution>();
  sol->u_sol_ = VectorXd::Zero(this->n_u_);
  sol->u_prev_ = VectorXd::Zero(this->n_u_);
  sol->dv_sol_ = VectorXd::Zero(this->n_v_);
  sol->lambda_h_sol_ = VectorXd::Zero(this->id_qp_.nh());
  sol->lambda_c_sol_ = VectorXd::Zero(this->id_qp_.nc());
  sol->epsilon_sol_ = VectorXd::Zero(this->id_qp_.nc_active());
  sol->solve_time_ = 0;
  return sol;
}

// Osc checkers and constructor
void OperationalSpaceControl::CheckCostSettings() {
  if (W_input_.size() != 0) {
    DRAKE_DEMAND((W_input_.rows() == n_u_) && (W_input_.cols() == n_u_));
  }
  if (W_input_smoothing_.size() != 0) {
    DRAKE_DEMAND((W_input_smoothing_.rows() == n_u_) &&
                 (W_input_smoothing_.cols() == n_u_));
  }
  if (W_joint_accel_.size() != 0) {
    DRAKE_DEMAND((W_joint_accel_.rows() == n_v_) &&
                 (W_joint_accel_.cols() == n_v_));
  }
}
void OperationalSpaceControl::CheckConstraintSettings() {
  if (!contact_names_map_.empty()) {
    DRAKE_DEMAND(mu_ != -1);
  }
}

void OperationalSpaceControl::Build() {
  // Checker
  CheckCostSettings();
  CheckConstraintSettings();
  for (auto& tracking_data : tracking_data_vec_) {
    tracking_data->CheckOscTrackingData();
    DRAKE_DEMAND(&tracking_data->plant() == &plant_);
  }

  // Construct QP
  id_qp_.Build(solver_choice_ == OscSolverChoice::kFastOSQP);

  // Add costs
  // 1. input cost
  if (W_input_.size() > 0) {
    id_qp_.AddQuadraticCost(
        "input_cost", W_input_, VectorXd::Zero(n_u_), id_qp_.u());
  }
  // 2. acceleration cost
  if (W_joint_accel_.size() > 0) {
    DRAKE_DEMAND(W_joint_accel_.rows() == n_v_);
    id_qp_.AddQuadraticCost(
        "acceleration_cost", W_joint_accel_, VectorXd::Zero(n_v_), id_qp_.dv());
  }
  if (W_input_smoothing_.size() > 0) {
    id_qp_.AddQuadraticCost(
        "input_smoothing_cost", W_input_smoothing_, VectorXd::Zero(n_u_), id_qp_.u());
  }
  // 3. contact force cost
  if (W_lambda_c_reg_.size() > 0) {
    int nc = id_qp_.lambda_c().rows();
    DRAKE_DEMAND(W_lambda_c_reg_.rows() == nc);
    id_qp_.AddQuadraticCost(
        "lambda_c_reg", W_lambda_c_reg_, VectorXd::Zero(nc), id_qp_.lambda_c());
  }
  // 3. constraint force cost
  if (W_lambda_h_reg_.size() > 0) {
    int nh = id_qp_.lambda_h().rows();
    DRAKE_DEMAND(W_lambda_h_reg_.rows() == nh);
    id_qp_.AddQuadraticCost(
        "lambda_h_cost", W_lambda_h_reg_, VectorXd::Zero(nh), id_qp_.lambda_h());
  }
  // 4. Soft constraint cost
  if (w_soft_constraint_ > 0) {
    int nca = id_qp_.nc_active();
    id_qp_.AddQuadraticCost(
        "soft_constraint_cost",
        w_soft_constraint_ * MatrixXd::Identity(nca, nca), VectorXd::Zero(nca),
        id_qp_.epsilon());
  }

  // 4. Tracking cost
  for (const auto& data : tracking_data_vec_) {
    id_qp_.AddQuadraticCost(data->GetName(), MatrixXd::Zero(n_v_, n_v_),
                               VectorXd::Zero(n_v_), id_qp_.dv());
  }

  // 5. Joint Limit cost
  // TODO(yangwill) discuss best way to implement joint limit cost
  if (w_joint_limit_ > 0) {
    K_joint_pos_ = w_joint_limit_ * W_joint_accel_.bottomRightCorner(
                                        n_revolute_joints_, n_revolute_joints_);
    id_qp_.AddQuadraticCost(
        "joint_limit_cost",
        MatrixXd::Zero(n_revolute_joints_,n_revolute_joints_),
        VectorXd::Zero(n_revolute_joints_),
        id_qp_.dv().tail(n_revolute_joints_));
  }

  // (Testing) 6. contact force blending
  if (ds_duration_ > 0) {
    const auto& lambda = id_qp_.lambda_c();
    id_qp_.AddQuadraticCost(
        "ds_blending", MatrixXd::Zero(4, 4), VectorXd::Zero(4),
        {lambda.segment(kSpaceDim * 0 + 2, 1),
         lambda.segment(kSpaceDim * 1 + 2, 1),
         lambda.segment(kSpaceDim * 2 + 2, 1),
         lambda.segment(kSpaceDim * 3 + 2, 1)});
  }

  fccqp_solver_ = std::make_unique<solvers::FCCQPSolver>();
  osqp_solver_ = std::make_unique<solvers::FastOsqpSolver>();
  id_qp_.get_mutable_prog().SetSolverOptions(fcc_qp_solver_options_);
  id_qp_.get_mutable_prog().SetSolverOptions(osqp_solver_options_);
}

drake::systems::EventStatus OperationalSpaceControl::DiscreteVariableUpdate(
    const drake::systems::Context<double>& context,
    drake::systems::DiscreteValues<double>* discrete_state) const {
  const BasicVector<double>* fsm_output =
      (BasicVector<double>*)this->EvalVectorInput(context, fsm_port_);
  VectorXd fsm_state = fsm_output->get_value();
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  double timestamp = robot_output->get_timestamp();

  auto prev_fsm_state =
      discrete_state->get_mutable_vector(
          prev_fsm_state_idx_).get_mutable_value();
  auto prev_mode_fsm_state =
      discrete_state->get_mutable_vector(
          prev_mode_fsm_state_idx_).get_mutable_value();

  if (fsm_state(0) != prev_fsm_state(0)) {
    prev_mode_fsm_state(0) = prev_fsm_state(0);
    prev_fsm_state(0) = fsm_state(0);

    discrete_state->get_mutable_vector(prev_event_time_idx_).get_mutable_value()
        << timestamp;
    if (osqp_solver_->IsInitialized()) {
      osqp_solver_->DisableWarmStart();
    }
  }
  return drake::systems::EventStatus::Succeeded();
}

void OperationalSpaceControl::UpdateTrackingData(
    const drake::systems::Context<double> &context,
    std::vector<OscTrackingDataState> *states) const {

  auto robot_output = dynamic_cast<const OutputVector<double>*>(
      EvalVectorInput(context, state_port_));

  VectorXd x = robot_output->GetState();

  // Update plant context
  SetPositionsIfNew<double>(
      plant_, x.head(plant_.num_positions()), context_);
  SetVelocitiesIfNew<double>(
      plant_, x.tail(plant_.num_velocities()), context_);

  double timestamp = robot_output->get_timestamp();
  int fsm_state = -1;
  int next_fsm_state = -1;
  double alpha = 0;
  double clock_time = timestamp;
  double prev_event_time = 0;

  VectorXd u_sol(n_u_);
  if (used_with_finite_state_machine_) {
    // Read in finite state machine
    auto fsm_output = this->EvalVectorInput(context, fsm_port_);
    fsm_state = fsm_output->get_value()(0);

    if (this->get_input_port(clock_port_).HasValue(context)) {
      auto clock = this->EvalVectorInput(context, clock_port_);
      clock_time = clock->get_value()(0);
    }

    if (this->get_input_port_impact_info().HasValue(context)) {
      auto impact_info = dynamic_cast<const ImpactInfoVector<double>*>(
          this->EvalVectorInput(context, impact_info_port_));
      alpha = impact_info->GetAlpha();
      next_fsm_state = impact_info->GetCurrentContactMode();
    }
    // Get discrete states
    prev_event_time =
        context.get_discrete_state(prev_event_time_idx_).get_value()(0);
  }

  // Update tracking data jacobians, etc.
  VectorXd v_proj = VectorXd::Zero(n_v_);
  for (unsigned int i = 0; i < tracking_data_vec_.size(); i++) {
    const auto tracking_data = tracking_data_vec_.at(i).get();
    auto &tracking_data_state = states->at(i);

    if (tracking_data->IsActive(fsm_state)) {
      if (fixed_position_vec_.at(i).size() != 0) {
        // Create constant trajectory and update
        tracking_data->Update(
            x, *context_,
            PiecewisePolynomial<double>(fixed_position_vec_.at(i)), clock_time,
            timestamp - prev_event_time, fsm_state, v_proj, tracking_data_state);
      } else {
        const string &traj_name = tracking_data->GetName();
        int port_index = traj_name_to_port_index_map_.at(traj_name);
        const drake::AbstractValue *input_traj =
            this->EvalAbstractInput(context, port_index);
        DRAKE_DEMAND(input_traj != nullptr);
        const auto &traj =
            input_traj->get_value<drake::trajectories::Trajectory<double>>();
        // Update
        tracking_data->Update(
            x, *context_, traj, clock_time, timestamp - prev_event_time,
            fsm_state, v_proj, tracking_data_state);
      }
    }
  }

  // Do impact invariant projection
  if (alpha != 0) {
    MatrixXd M(n_v_, n_v_);
    plant_.CalcMassMatrix(*context_, &M);

    VectorXd v_perp = CalcImpactInvariantProjection(
        fsm_state, next_fsm_state, M, x.tail(n_v_), *states);
    // Need to call Update before this to get the updated jacobian
    v_proj = alpha * v_perp;
  }

  for (unsigned int i = 0; i < tracking_data_vec_.size(); i++) {
    const auto tracking_data = tracking_data_vec_.at(i).get();
    auto &tracking_data_state = states->at(i);
    if (tracking_data->IsActive(fsm_state) && tracking_data->GetImpactInvariantProjection()) {
      tracking_data->MakeImpactInvariantCorrection(
          clock_time, timestamp - prev_event_time, fsm_state, v_proj,
          tracking_data_state);
    }
  }

}

void OperationalSpaceControl::SolveQp(
    const VectorXd& x,
    const drake::systems::Context<double>& context, double t, int fsm_state,
    double t_since_last_state_switch, double alpha, int next_fsm_state,
    OperationalSpaceControl::id_qp_solution* sol) const {

  DRAKE_DEMAND(osqp_solver_ != nullptr);
  DRAKE_DEMAND(fccqp_solver_ != nullptr);

  // Update context
  SetPositionsIfNew<double>(
      plant_, x.head(plant_.num_positions()), context_);
  SetVelocitiesIfNew<double>(
      plant_, x.tail(plant_.num_velocities()), context_);

  const auto active_contact_names = contact_names_map_.count(fsm_state) > 0 ?
      contact_names_map_.at(fsm_state) : std::vector<std::string>();
  id_qp_.UpdateDynamics(x, active_contact_names, {});

  const auto& tracking_data_states = get_cache_entry(
      tracking_data_cache_).Eval<std::vector<OscTrackingDataState>>(context);

  // Update costs
  // 4. Tracking cost
  for (unsigned int i = 0; i < tracking_data_vec_.size(); i++) {
    const auto tracking_data = tracking_data_vec_.at(i).get();
    const auto& tracking_data_state = tracking_data_states.at(i);

    if (tracking_data->IsActive(fsm_state)) {

      const VectorXd& ddy_t = tracking_data_state.yddot_command_;
      const MatrixXd& W = tracking_data_state.time_varying_weight_;
      const MatrixXd& J_t = tracking_data_state.J_;
      const VectorXd& JdotV_t = tracking_data_state.JdotV_;
      const VectorXd constant_term = (JdotV_t - ddy_t);

      // TODO (@Brian-Acosta) figure out why alip_lqr_data_collection sometimes
      //  sees nans in the swing foot traj
      if (ddy_t.hasNaN()) {
        continue;
      };

      id_qp_.UpdateCost(
          tracking_data->GetName(),
          2 * J_t.transpose() * W * J_t,
          2 * J_t.transpose() * W * (JdotV_t - ddy_t),
          constant_term.transpose() * W * constant_term);
    } else {
      id_qp_.UpdateCost(
          tracking_data->GetName(), MatrixXd::Zero(n_v_, n_v_),
          VectorXd::Zero(n_v_));
    }
  }

  // Add joint limit constraints
  if (w_joint_limit_ > 0) {
    VectorXd w_joint_limit =
        K_joint_pos_ * (x.head(plant_.num_positions())
                            .tail(n_revolute_joints_) -q_max_).cwiseMax(0) +
        K_joint_pos_ * (x.head(plant_.num_positions())
                            .tail(n_revolute_joints_) - q_min_).cwiseMin(0);
    id_qp_.UpdateCost(
        "joint_limit_cost",
        MatrixXd::Zero(n_revolute_joints_, n_revolute_joints_), w_joint_limit);
  }

  // TODO  (@Brian-Acosta) test double support blending as a force cost
  // (Testing) 6. blend contact forces during double support phase
  if (ds_duration_ > 0) {
    int nc = id_qp_.nc();
    MatrixXd A = MatrixXd::Zero(1, nc / kSpaceDim);
    if (std::find(ds_states_.begin(), ds_states_.end(), fsm_state) !=
        ds_states_.end()) {
      double s = std::clamp(t_since_last_state_switch / ds_duration_, 0.05, 0.95);

      int prev_mode_fsm_state = context.get_discrete_state(
          prev_mode_fsm_state_idx_).get_value()(0);

      double alpha_left = 0;
      double alpha_right = 0;
      if (prev_mode_fsm_state == right_support_state_) {
        // We want left foot force to gradually increase
        alpha_left = 1.0 - s;
        alpha_right = -s;
      } else if (prev_mode_fsm_state == left_support_state_) {
        alpha_left = -s;
        alpha_right = 1.0 - s;
      }
      A(0, 0) = alpha_left;
      A(0, 1) = alpha_left;
      A(0, 2) = alpha_right;
      A(0, 3) = alpha_right;
    }
    id_qp_.UpdateCost(
        "ds_blending",
        w_blend_constraint_ * A.transpose() * A, VectorXd::Zero(4), 0);
  }

  // test joint-level input cost by fsm state
  if (!fsm_to_w_input_map_.empty()) {
    MatrixXd W = W_input_;
    if (fsm_to_w_input_map_.count(fsm_state)) {
      int j = fsm_to_w_input_map_.at(fsm_state).first;
      double w = fsm_to_w_input_map_.at(fsm_state).second;
      W(j, j) += w;
    }
    VectorXd ud = (get_input_port(ff_input_port_).HasValue(context)) ?
                   get_input_port(ff_input_port_).Eval(context) :
                   VectorXd::Zero(n_u_);

    id_qp_.UpdateCost("input_cost", 2 * W, -2 * W * ud, ud.transpose() * W * ud);
  }

  // (Testing) 7. Cost for staying close to the previous input
  if (W_input_smoothing_.size() > 0 && sol->u_prev_.rows() == plant_.num_actuators()) {
    id_qp_.UpdateCost(
        "input_smoothing_cost", W_input_smoothing_, -W_input_smoothing_ * sol->u_prev_,
        0.5 * sol->u_prev_.transpose() * W_input_smoothing_ * sol->u_prev_);
  }

  if (W_lambda_c_reg_.size() > 0) {
    UpdateContactForceRegularization(context, active_contact_names);
  }

  if (W_lambda_h_reg_.size() > 0) {
    id_qp_.UpdateCost(
        "lambda_h_reg",
        (1 + alpha) * W_lambda_h_reg_,VectorXd::Zero(id_qp_.nh()));
  }

  if (!fccqp_solver_->is_initialized()) {
    int contact_start_idx = id_qp_.get_prog().FindDecisionVariableIndex(
        id_qp_.lambda_c()(0));
    fccqp_solver_->InitializeSolver(
        id_qp_.get_prog(), fcc_qp_solver_options_, id_qp_.nc(), contact_start_idx,
        id_qp_.get_ordered_friction_coeffs());
  }

  if (!osqp_solver_->IsInitialized()) {
    osqp_solver_->InitializeSolver(id_qp_.get_prog(), osqp_solver_options_);
  }

  // Solve the QP
  MathematicalProgramResult result;

  switch (solver_choice_) {
    case kFCCQP:
      result = fccqp_solver_->Solve(id_qp_.get_prog());
      sol->solve_time_ = result.get_solver_details<FCCQPSolver>().solve_time;
      break;
    case kFastOSQP:
      result = osqp_solver_->Solve(id_qp_.get_prog());
      sol->solve_time_ = result.get_solver_details<drake::solvers::OsqpSolver>().run_time;
      break;
    default:
      throw std::runtime_error("unrecognized solver option");
  }

  if (result.is_success()) {
    // Extract solutions
    sol->dv_sol_ = result.GetSolution(id_qp_.dv());
    sol->u_sol_ = result.GetSolution(id_qp_.u());
    sol->u_prev_ = sol->u_sol_;
    sol->lambda_c_sol_ = result.GetSolution(id_qp_.lambda_c());
    sol->lambda_h_sol_ = result.GetSolution(id_qp_.lambda_h());
    sol->epsilon_sol_ = result.GetSolution(id_qp_.epsilon());
    fccqp_solver_->set_warm_start(true);
    osqp_solver_->EnableWarmStart();
  } else {
    sol->u_prev_ = 0.99 * sol->u_sol_ + VectorXd::Random(n_u_);
    fccqp_solver_->set_warm_start(false);
    osqp_solver_->DisableWarmStart();
  }
}


void OperationalSpaceControl::SolveIDQP(
    const drake::systems::Context<double> &context,
    OperationalSpaceControl::id_qp_solution *solution) const {

  // Read in current state and time
  auto robot_output = dynamic_cast<const OutputVector<double>*>(
      EvalVectorInput(context, state_port_));

  VectorXd x = robot_output->GetState();

  double timestamp = robot_output->get_timestamp();

  double current_time = timestamp;

  VectorXd u_sol(n_u_);
  if (used_with_finite_state_machine_) {
    // Read in finite state machine
    auto fsm_output = this->EvalVectorInput(context, fsm_port_);
    double clock_time = current_time;
    if (this->get_input_port(clock_port_).HasValue(context)) {
      auto clock = this->EvalVectorInput(context, clock_port_);
      clock_time = clock->get_value()(0);
    }
    VectorXd fsm_state = fsm_output->get_value();

    double alpha = 0;
    int next_fsm_state = -1;
    if (this->get_input_port_impact_info().HasValue(context)) {
      auto impact_info = (ImpactInfoVector<double>*)this->EvalVectorInput(
          context, impact_info_port_);
      alpha = impact_info->GetAlpha();
      next_fsm_state = impact_info->GetCurrentContactMode();
    }
    // Get discrete states
    const auto prev_event_time =
        context.get_discrete_state(prev_event_time_idx_).get_value();
    SolveQp(x, context, clock_time, fsm_state(0),
                    current_time - prev_event_time(0), alpha, next_fsm_state, solution);
  } else {
    SolveQp(x, context, current_time, -1, current_time, 0, -1, solution);
  }
}

void OperationalSpaceControl::UpdateContactForceRegularization(
    const Context<double>& context,
    const vector<std::string> &contacts_currently_active) const {

  int n = contacts_currently_active.size();
  double lambda_z_des = n > 0 ? 9.81 * plant_.CalcTotalMass(*context_) / n : 0;

  if (id_qp_.has_cost_named("lambda_c_reg")) {
    Eigen::VectorXd lambda_des_stacked = VectorXd::Zero(id_qp_.nc());
    for (const auto& cname : contacts_currently_active) {
      int start = id_qp_.get_contact_variable_start(cname);
      if (get_input_port_lambda_c_des(cname).HasValue(context)) {
        lambda_des_stacked.segment<3>(start) = EvalVectorInput(
            context,
            contact_name_to_lambda_des_port_map_.at(cname))->value();
      } else {
        lambda_des_stacked(start + 2) = lambda_z_des;
      }
    }

    double c = lambda_des_stacked.transpose() * W_lambda_c_reg_ * lambda_des_stacked;
    id_qp_.UpdateCost(
        "lambda_c_reg",
        2 *  W_lambda_c_reg_,
        -2 * W_lambda_c_reg_ * lambda_des_stacked, c);
  }
}

namespace {
inline bool has_active_ii_proj(const OscTrackingData* data, int fsm_state) {
  return data->IsActive(fsm_state) and data->GetImpactInvariantProjection();
}
}

// TODO (@Yangwill) test that this is equivalent to the previous impact
//  invariant implementation
Eigen::VectorXd OperationalSpaceControl::CalcImpactInvariantProjection(
    int fsm_state, int next_fsm_state, const Eigen::MatrixXd &M,
    const VectorXd& v,
    const std::vector<OscTrackingDataState> &up_to_date_td_state) const {

  auto map_iterator = contact_names_map_.find(next_fsm_state);

  if (map_iterator == contact_names_map_.end()) {
    return VectorXd::Zero(n_v_);
  }

  std::vector<std::string> next_contact_set = map_iterator->second;

  int active_constraint_dim = kSpaceDim * next_contact_set.size() + id_qp_.nh();

  MatrixXd J_next = MatrixXd::Zero(active_constraint_dim, n_v_);
  int row_start = 0;
  for (const auto& cname : next_contact_set) {
    J_next.block(row_start, 0, kSpaceDim, n_v_) =
        id_qp_.get_contact_evaluator(cname).EvalFullJacobian(*context_);
    row_start += kSpaceDim;
  }
  // Holonomic constraints
  if (id_qp_.nh() > 0) {
    J_next.block(row_start, 0, id_qp_.nh(), n_v_) =
        id_qp_.get_holonomic_evaluators().EvalFullJacobian(*context_);
  }

  Eigen::MatrixXd M_Jt = M.llt().solve(J_next.transpose());

  int active_tracking_data_dim = 0;
  for (const auto& tracking_data : tracking_data_vec_) {
    if (has_active_ii_proj(tracking_data.get(), fsm_state)) {
      active_tracking_data_dim += tracking_data->GetYdotDim();
    }
  }

  MatrixXd A = MatrixXd::Zero(active_tracking_data_dim, active_constraint_dim);
  VectorXd ydot_err_vec = VectorXd::Zero(active_tracking_data_dim);
  int start_row = 0;
  for (int i = 0; i < tracking_data_vec_.size(); ++i) {
    if (has_active_ii_proj(tracking_data_vec_.at(i).get(), fsm_state)) {
      A.block(start_row, 0, tracking_data_vec_.at(i)->GetYdotDim(),
              active_constraint_dim) = up_to_date_td_state.at(i).J_ * M_Jt;
      ydot_err_vec.segment(start_row, tracking_data_vec_.at(i)->GetYdotDim()) =
          up_to_date_td_state.at(i).error_ydot_;
      start_row += tracking_data_vec_.at(i)->GetYdotDim();
    }
  }

  //  int n_holonomic_constraints = id_qp_.nh();
  MatrixXd A_constrained = MatrixXd::Zero(active_constraint_dim + id_qp_.nh(),
                                          active_constraint_dim + id_qp_.nh());
  A_constrained.block(0, 0, active_constraint_dim, active_constraint_dim) =
      A.transpose() * A;
  VectorXd b_constrained = VectorXd::Zero(active_constraint_dim + id_qp_.nh());
  VectorXd Ab = A.transpose() * ydot_err_vec;

  if (id_qp_.nh() > 0) {
    MatrixXd J_h = id_qp_.get_holonomic_evaluators().EvalFullJacobian(
        *context_);
    MatrixXd C = J_h * M_Jt;
    VectorXd d = J_h * v;
    A_constrained.block(active_constraint_dim, 0, id_qp_.nh(), active_constraint_dim) =
        C;
    A_constrained.block(0, active_constraint_dim, active_constraint_dim, id_qp_.nh()) =
        C.transpose();
    b_constrained << Ab, d;
  } else {
    b_constrained << Ab;
  }

  MatrixXd ii_lambda_sol = A_constrained.completeOrthogonalDecomposition()
                       .solve(b_constrained)
                       .head(active_constraint_dim);

  return M_Jt * ii_lambda_sol + 1e-13 * VectorXd::Ones(n_v_);
}

void OperationalSpaceControl::AssignOscLcmOutput(
    const Context<double>& context, dairlib::lcmt_osc_output* output) const {
  auto state =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  double total_cost = 0;
  int fsm_state = -1;
  if (used_with_finite_state_machine_) {
    auto fsm_output =
        (BasicVector<double>*)this->EvalVectorInput(context, fsm_port_);
    fsm_state = fsm_output->get_value()(0);
  }

  output->utime = state->get_timestamp() * 1e6;
  output->fsm_state = fsm_state;

  auto osc_solution = get_cache_entry(
      osc_solution_cache_).Eval<id_qp_solution>(context);

  const std::vector<std::pair<std::string, const Eigen::VectorXd&>>
  potential_regularization_cost_names_and_vars {
      {"input_cost", osc_solution.u_sol_},
      {"acceleration_cost", osc_solution.dv_sol_},
      {"soft_constraint_cost", osc_solution.epsilon_sol_},
      {"input_smoothing_cost", osc_solution.u_sol_},
      {"lambda_c_reg", osc_solution.lambda_c_sol_},
      {"lambda_h_cost", osc_solution.lambda_h_sol_}
  };

  output->regularization_cost_names.clear();
  output->regularization_costs.clear();
  for (const auto& [name, sol] : potential_regularization_cost_names_and_vars) {
    VectorXd y = VectorXd::Zero(1);
    if (id_qp_.has_cost_named(name)) {
      id_qp_.get_cost_evaluator(name).Eval(sol, &y);
    }
    output->regularization_cost_names.emplace_back(name);
    output->regularization_costs.emplace_back(y(0));
    total_cost += y(0);
  }
  output->num_regularization_costs = output->regularization_costs.size();

  lcmt_osc_qp_output qp_output;
  qp_output.solve_time = osc_solution.solve_time_;
  qp_output.u_dim = n_u_;
  qp_output.lambda_c_dim = id_qp_.nc();
  qp_output.lambda_h_dim = id_qp_.nh();
  qp_output.v_dim = n_v_;
  qp_output.epsilon_dim = id_qp_.nc_active();
  qp_output.u_sol = CopyVectorXdToStdFloatVector(osc_solution.u_sol_);
  qp_output.lambda_c_sol = CopyVectorXdToStdFloatVector(osc_solution.lambda_c_sol_);
  qp_output.lambda_h_sol = CopyVectorXdToStdFloatVector(osc_solution.lambda_h_sol_);
  qp_output.dv_sol = CopyVectorXdToStdFloatVector(osc_solution.dv_sol_);
  qp_output.epsilon_sol = CopyVectorXdToStdFloatVector(osc_solution.epsilon_sol_);
  output->qp_output = qp_output;

  output->tracking_data = std::vector<lcmt_osc_tracking_data>();
  output->tracking_costs = std::vector<float>();
  output->tracking_data_names = std::vector<std::string>();

  output->tracking_data.clear();
  output->tracking_costs.clear();
  output->tracking_data_names.clear();

  const auto& tracking_data_states = get_cache_entry(
      tracking_data_cache_).Eval<std::vector<OscTrackingDataState>>(context);

  for (unsigned int i = 0; i < tracking_data_vec_.size(); i++) {
    const auto& tracking_data = tracking_data_vec_.at(i).get();
    const auto& tracking_data_state = tracking_data_states.at(i);

    lcmt_osc_tracking_data osc_output;
    osc_output.y_dim = tracking_data->GetYDim();
    osc_output.ydot_dim = tracking_data->GetYdotDim();
    osc_output.name = tracking_data->GetName();
    osc_output.is_active = tracking_data->IsActive(fsm_state);
    osc_output.y = std::vector<float>(osc_output.y_dim);
    osc_output.y_des = std::vector<float>(osc_output.y_dim);
    osc_output.error_y = std::vector<float>(osc_output.ydot_dim);
    osc_output.ydot = std::vector<float>(osc_output.ydot_dim);
    osc_output.ydot_des = std::vector<float>(osc_output.ydot_dim);
    osc_output.error_ydot = std::vector<float>(osc_output.ydot_dim);
    osc_output.yddot_des = std::vector<float>(osc_output.ydot_dim);
    osc_output.yddot_command = std::vector<float>(osc_output.ydot_dim);
    osc_output.yddot_command_sol = std::vector<float>(osc_output.ydot_dim);

    if (tracking_data->IsActive(fsm_state)) {
      osc_output.y = CopyVectorXdToStdFloatVector(
          tracking_data_state.y_);
      osc_output.y_des = CopyVectorXdToStdFloatVector(
          tracking_data_state.y_des_);
      osc_output.error_y = CopyVectorXdToStdFloatVector(
          tracking_data_state.error_y_);
      osc_output.ydot = CopyVectorXdToStdFloatVector(
          tracking_data_state.ydot_);
      osc_output.ydot_des = CopyVectorXdToStdFloatVector(
          tracking_data_state.ydot_des_);
      osc_output.error_ydot = CopyVectorXdToStdFloatVector(
          tracking_data_state.error_ydot_);
      osc_output.yddot_des = CopyVectorXdToStdFloatVector(
          tracking_data_state.yddot_des_converted_);
      osc_output.yddot_command = CopyVectorXdToStdFloatVector(
          tracking_data_state.yddot_command_);
      osc_output.yddot_command_sol = CopyVectorXdToStdFloatVector(
          tracking_data_state.CalcYddotCommandSol(osc_solution.dv_sol_));

      VectorXd y_tracking_cost = VectorXd::Zero(1);
      id_qp_.get_cost_evaluator(
          tracking_data->GetName()
      ).Eval(osc_solution.dv_sol_, &y_tracking_cost);
      total_cost += y_tracking_cost[0];
      output->tracking_costs.push_back(y_tracking_cost[0]);
      output->tracking_data.push_back(osc_output);
      output->tracking_data_names.push_back(tracking_data->GetName());
    }
  }

  output->num_tracking_data = output->tracking_data_names.size();
  output->num_regularization_costs = output->regularization_cost_names.size();
}

void OperationalSpaceControl::CalcOptimalInput(
    const drake::systems::Context<double>& context,
    systems::TimestampedVector<double>* control) const {

  auto robot_output = dynamic_cast<const OutputVector<double>*>(
      EvalVectorInput(context, state_port_));

  auto osc_solution = get_cache_entry(
      osc_solution_cache_).Eval<id_qp_solution>(context);

  // Assign the control input
  control->SetDataVector(osc_solution.u_sol_);
  control->set_timestamp(robot_output->get_timestamp());
}

void OperationalSpaceControl::CheckTracking(
    const drake::systems::Context<double>& context,
    TimestampedVector<double>* output) const {

  auto robot_output = dynamic_cast<const OutputVector<double>*>(
      EvalVectorInput(context, state_port_));
  auto osc_solution = get_cache_entry(
      osc_solution_cache_).Eval<id_qp_solution>(context);

  output->set_timestamp(robot_output->get_timestamp());
  output->get_mutable_value()(0) = 0.0;
  VectorXd y_soft_constraint_cost = VectorXd::Zero(1);
  if (id_qp_.has_cost_named("soft_constraint_cost")) {
    id_qp_.get_cost_evaluator("soft_constraint_cost").Eval(
        osc_solution.epsilon_sol_, &y_soft_constraint_cost);
  }
  if (y_soft_constraint_cost[0] > 1e5 || isnan(y_soft_constraint_cost[0])) {
    output->get_mutable_value()(0) = 1.0;
  }
}

}  // namespace dairlib::systems::controllers
