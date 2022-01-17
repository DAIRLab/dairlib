#include "systems/controllers/osc/operational_space_control.h"

#include <drake/multibody/plant/multibody_plant.h>

#include "common/eigen_utils.h"
#include "multibody/multibody_utils.h"

#include "drake/common/text_logging.h"

using std::cout;
using std::endl;

using std::numeric_limits;
using std::string;
using std::vector;

using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::VectorXd;

using dairlib::multibody::createContext;
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

using multibody::CreateWithSpringsToWithoutSpringsMapPos;
using multibody::CreateWithSpringsToWithoutSpringsMapVel;
using multibody::makeNameToVelocitiesMap;
using multibody::SetPositionsIfNew;
using multibody::SetVelocitiesIfNew;
using multibody::WorldPointEvaluator;

OperationalSpaceControl::OperationalSpaceControl(
    const MultibodyPlant<double>& plant_w_spr,
    const MultibodyPlant<double>& plant_wo_spr,
    drake::systems::Context<double>* context_w_spr,
    drake::systems::Context<double>* context_wo_spr,
    bool used_with_finite_state_machine, bool print_tracking_info,
    double qp_time_limit)
    : plant_w_spr_(plant_w_spr),
      plant_wo_spr_(plant_wo_spr),
      context_w_spr_(context_w_spr),
      context_wo_spr_(context_wo_spr),
      world_w_spr_(plant_w_spr_.world_frame()),
      world_wo_spr_(plant_wo_spr_.world_frame()),
      used_with_finite_state_machine_(used_with_finite_state_machine),
      print_tracking_info_(print_tracking_info),
      qp_time_limit_(qp_time_limit) {
  this->set_name("OSC");

  n_q_ = plant_wo_spr.num_positions();
  n_v_ = plant_wo_spr.num_velocities();
  n_u_ = plant_wo_spr.num_actuators();

  int n_q_w_spr = plant_w_spr.num_positions();
  int n_v_w_spr = plant_w_spr.num_velocities();
  int n_u_w_spr = plant_w_spr.num_actuators();

  // Input/Output Setup
  state_port_ =
      this->DeclareVectorInputPort(
              "x, u, t", OutputVector<double>(n_q_w_spr, n_v_w_spr, n_u_w_spr))
          .get_index();
  if (used_with_finite_state_machine) {
    fsm_port_ =
        this->DeclareVectorInputPort("fsm", BasicVector<double>(1)).get_index();
    near_impact_port_ = this->DeclareVectorInputPort("next_fsm, t_to_impact",
                                                     BasicVector<double>(2))
                            .get_index();

    // Discrete update to record the last state event time
    DeclarePerStepDiscreteUpdateEvent(
        &OperationalSpaceControl::DiscreteVariableUpdate);
    prev_fsm_state_idx_ = this->DeclareDiscreteState(-0.1 * VectorXd::Ones(1));
    prev_event_time_idx_ = this->DeclareDiscreteState(VectorXd::Zero(1));
  }

  osc_output_port_ = this->DeclareVectorOutputPort(
                             "u, t", TimestampedVector<double>(n_u_w_spr),
                             &OperationalSpaceControl::CalcOptimalInput)
                         .get_index();
  osc_debug_port_ =
      this->DeclareAbstractOutputPort(
              "lcmt_osc_debug", &OperationalSpaceControl::AssignOscLcmOutput)
          .get_index();

  const std::map<string, int>& vel_map_wo_spr =
      multibody::makeNameToVelocitiesMap(plant_wo_spr);

  // Initialize the mapping from spring to no spring
  map_position_from_spring_to_no_spring_ =
      CreateWithSpringsToWithoutSpringsMapPos(plant_w_spr, plant_wo_spr);
  map_velocity_from_spring_to_no_spring_ =
      CreateWithSpringsToWithoutSpringsMapVel(plant_w_spr, plant_wo_spr);

  // Get input limits
  VectorXd u_min(n_u_);
  VectorXd u_max(n_u_);
  for (JointActuatorIndex i(0); i < n_u_; ++i) {
    u_min(i) = -plant_wo_spr_.get_joint_actuator(i).effort_limit();
    u_max(i) = plant_wo_spr_.get_joint_actuator(i).effort_limit();
  }
  u_min_ = u_min;
  u_max_ = u_max;

  n_revolute_joints_ = 0;
  for (JointIndex i(0); i < plant_wo_spr_.num_joints(); ++i) {
    const drake::multibody::Joint<double>& joint = plant_wo_spr_.get_joint(i);
    if (joint.type_name() == "revolute") {
      n_revolute_joints_ += 1;
    }
  }
  VectorXd q_min(n_revolute_joints_);
  VectorXd q_max(n_revolute_joints_);
  int floating_base_offset = n_v_ - n_revolute_joints_;
  for (JointIndex i(0); i < plant_wo_spr_.num_joints(); ++i) {
    const drake::multibody::Joint<double>& joint = plant_wo_spr_.get_joint(i);
    if (joint.type_name() == "revolute") {
      q_min(vel_map_wo_spr.at(joint.name() + "dot") - floating_base_offset) =
          plant_wo_spr.get_joint(i).position_lower_limits()[0];
      q_max(vel_map_wo_spr.at(joint.name() + "dot") - floating_base_offset) =
          plant_wo_spr.get_joint(i).position_upper_limits()[0];
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

  // Check if the model is floating based
  is_quaternion_ = multibody::isQuaternion(plant_w_spr);
}

// Optional features
void OperationalSpaceControl::SetUpDoubleSupportPhaseBlending(
    double ds_duration, int left_support_state, int right_support_state,
    std::vector<int> ds_states) {
  DRAKE_DEMAND(ds_duration > 0);
  DRAKE_DEMAND(!ds_states.empty());
  ds_duration_ = ds_duration;
  left_support_state_ = left_support_state;
  right_support_state_ = right_support_state;
  ds_states_ = ds_states;
}

// Cost methods
void OperationalSpaceControl::AddAccelerationCost(
    const std::string& joint_vel_name, double w) {
  if (W_joint_accel_.size() == 0) {
    W_joint_accel_ = Eigen::MatrixXd::Zero(n_v_, n_v_);
  }
  int idx = makeNameToVelocitiesMap(plant_wo_spr_).at(joint_vel_name);
  W_joint_accel_(idx, idx) += w;
}

// Constraint methods
void OperationalSpaceControl::AddContactPoint(
    const WorldPointEvaluator<double>* evaluator) {
  single_contact_mode_ = true;
  AddStateAndContactPoint(-1, evaluator);
}

void OperationalSpaceControl::AddStateAndContactPoint(
    int state, const WorldPointEvaluator<double>* evaluator) {
  DRAKE_DEMAND(&evaluator->plant() == &plant_wo_spr_);

  // Find the new contact in all_contacts_
  auto it_c = std::find(all_contacts_.begin(), all_contacts_.end(), evaluator);
  int contact_idx = std::distance(all_contacts_.begin(), it_c);
  // Add to contact list if the new contact doesn't exist in the list
  if (it_c == all_contacts_.cend()) {
    all_contacts_.push_back(evaluator);
  }

  // Find the finite state machine state in contact_indices_map_
  auto map_iterator = contact_indices_map_.find(state);
  if (map_iterator == contact_indices_map_.end()) {
    // state doesn't exist in the map
    contact_indices_map_[state] = {contact_idx};
  } else {
    // Add contact_idx to the existing set (note that std::set removes
    // duplicates automatically)
    map_iterator->second.insert(contact_idx);
  }
}

void OperationalSpaceControl::AddKinematicConstraint(
    const multibody::KinematicEvaluatorSet<double>* evaluators) {
  DRAKE_DEMAND(&evaluators->plant() == &plant_wo_spr_);
  kinematic_evaluators_ = evaluators;
}

// Tracking data methods
void OperationalSpaceControl::AddTrackingData(OscTrackingData* tracking_data,
                                              double t_lb, double t_ub) {
  tracking_data_vec_->push_back(tracking_data);
  fixed_position_vec_.push_back(VectorXd::Zero(0));
  t_s_vec_.push_back(t_lb);
  t_e_vec_.push_back(t_ub);

  // Construct input ports and add element to traj_name_to_port_index_map_ if
  // the port for the traj is not created yet
  string traj_name = tracking_data->GetName();
  if (traj_name_to_port_index_map_.find(traj_name) ==
      traj_name_to_port_index_map_.end()) {
    PiecewisePolynomial<double> pp = PiecewisePolynomial<double>();
    int port_index =
        this->DeclareAbstractInputPort(
                traj_name,
                drake::Value<drake::trajectories::Trajectory<double>>(pp))
            .get_index();
    traj_name_to_port_index_map_[traj_name] = port_index;
  }
}
void OperationalSpaceControl::AddConstTrackingData(
    OscTrackingData* tracking_data, const VectorXd& v, double t_lb,
    double t_ub) {
  tracking_data_vec_->push_back(tracking_data);
  fixed_position_vec_.push_back(v);
  t_s_vec_.push_back(t_lb);
  t_e_vec_.push_back(t_ub);
}

// Osc checkers and constructor
void OperationalSpaceControl::CheckCostSettings() {
  if (W_input_.size() != 0) {
    DRAKE_DEMAND((W_input_.rows() == n_u_) && (W_input_.cols() == n_u_));
  }
  if (W_joint_accel_.size() != 0) {
    DRAKE_DEMAND((W_joint_accel_.rows() == n_v_) &&
                 (W_joint_accel_.cols() == n_v_));
  }
}
void OperationalSpaceControl::CheckConstraintSettings() {
  if (!all_contacts_.empty()) {
    DRAKE_DEMAND(mu_ != -1);
  }
  if (single_contact_mode_) {
    DRAKE_DEMAND(contact_indices_map_.size() == 1);
  }
}

void OperationalSpaceControl::Build() {
  // Checker
  CheckCostSettings();
  CheckConstraintSettings();
  for (auto tracking_data : *tracking_data_vec_) {
    tracking_data->CheckOscTrackingData();
    DRAKE_DEMAND(&tracking_data->plant_w_spr() == &plant_w_spr_);
    DRAKE_DEMAND(&tracking_data->plant_wo_spr() == &plant_wo_spr_);
  }

  // Construct QP
  prog_ = std::make_unique<MathematicalProgram>();

  // Size of decision variable
  n_h_ = (kinematic_evaluators_ == nullptr)
             ? 0
             : kinematic_evaluators_->count_full();
  n_c_ = kSpaceDim * all_contacts_.size();
  n_c_active_ = 0;
  for (auto evaluator : all_contacts_) {
    n_c_active_ += evaluator->num_active();
  }

  // Record the contact dimension per state
  for (auto contact_map : contact_indices_map_) {
    int active_contact_dim = 0;
    for (unsigned int i = 0; i < all_contacts_.size(); i++) {
      if (contact_map.second.find(i) != contact_map.second.end()) {
        active_contact_dim +=
            all_contacts_[i]->EvalFullJacobian(*context_wo_spr_).rows();
      }
    }
    active_contact_dim_[contact_map.first] = active_contact_dim;
  }

  // Initialize solution
  dv_sol_ = std::make_unique<Eigen::VectorXd>(n_v_);
  u_sol_ = std::make_unique<Eigen::VectorXd>(n_u_);
  lambda_c_sol_ = std::make_unique<Eigen::VectorXd>(n_c_);
  lambda_h_sol_ = std::make_unique<Eigen::VectorXd>(n_h_);
  epsilon_sol_ = std::make_unique<Eigen::VectorXd>(n_c_active_);
  dv_sol_->setZero();
  u_sol_->setZero();
  lambda_c_sol_->setZero();
  lambda_h_sol_->setZero();
  epsilon_sol_->setZero();

  // Add decision variables
  dv_ = prog_->NewContinuousVariables(n_v_, "dv");
  u_ = prog_->NewContinuousVariables(n_u_, "u");
  lambda_c_ = prog_->NewContinuousVariables(n_c_, "lambda_contact");
  lambda_h_ = prog_->NewContinuousVariables(n_h_, "lambda_holonomic");
  epsilon_ = prog_->NewContinuousVariables(n_c_active_, "epsilon");

  // Add constraints
  // 1. Dynamics constraint
  dynamics_constraint_ =
      prog_
          ->AddLinearEqualityConstraint(
              MatrixXd::Zero(n_v_, n_v_ + n_c_ + n_h_ + n_u_),
              VectorXd::Zero(n_v_), {dv_, lambda_c_, lambda_h_, u_})
          .evaluator()
          .get();
  // 2. Holonomic constraint
  holonomic_constraint_ =
      prog_
          ->AddLinearEqualityConstraint(MatrixXd::Zero(n_h_, n_v_),
                                        VectorXd::Zero(n_h_), dv_)
          .evaluator()
          .get();
  // 3. Contact constraint
  if (all_contacts_.size() > 0) {
    if (w_soft_constraint_ <= 0) {
      contact_constraints_ =
          prog_
              ->AddLinearEqualityConstraint(MatrixXd::Zero(n_c_active_, n_v_),
                                            VectorXd::Zero(n_c_active_), dv_)
              .evaluator()
              .get();
    } else {
      // Relaxed version:
      contact_constraints_ =
          prog_
              ->AddLinearEqualityConstraint(
                  MatrixXd::Zero(n_c_active_, n_v_ + n_c_active_),
                  VectorXd::Zero(n_c_active_), {dv_, epsilon_})
              .evaluator()
              .get();
    }
  }
  if (!all_contacts_.empty()) {
    VectorXd mu_neg1(2);
    VectorXd mu_1(2);
    VectorXd one(1);
    MatrixXd A = MatrixXd(5, kSpaceDim);
    A << -1, 0, mu_, 0, -1, mu_, 1, 0, mu_, 0, 1, mu_, 0, 0, 1;

    for (unsigned int j = 0; j < all_contacts_.size(); j++) {
      friction_constraints_.push_back(
          prog_
              ->AddLinearConstraint(
                  A, VectorXd::Zero(5),
                  Eigen::VectorXd::Constant(
                      5, std::numeric_limits<double>::infinity()),
                  lambda_c_.segment(kSpaceDim * j, 3))
              .evaluator()
              .get());
    }
  }
  // 5. Input constraint
  if (with_input_constraints_) {
    prog_->AddLinearConstraint(MatrixXd::Identity(n_u_, n_u_), u_min_, u_max_,
                               u_);
  }
  // No joint position constraint in this implementation

  // Add costs
  // 1. input cost
  if (W_input_.size() > 0) {
    prog_->AddQuadraticCost(W_input_, VectorXd::Zero(n_u_), u_);
  }
  // 2. acceleration cost
  if (W_joint_accel_.size() > 0) {
    prog_->AddQuadraticCost(W_joint_accel_, VectorXd::Zero(n_v_), dv_);
  }
  // 3. Soft constraint cost
  if (w_soft_constraint_ > 0) {
    prog_->AddQuadraticCost(
        w_soft_constraint_ * MatrixXd::Identity(n_c_active_, n_c_active_),
        VectorXd::Zero(n_c_active_), epsilon_);
  }
  // 4. Tracking cost
  for (unsigned int i = 0; i < tracking_data_vec_->size(); i++) {
    tracking_cost_.push_back(prog_
                                 ->AddQuadraticCost(MatrixXd::Zero(n_v_, n_v_),
                                                    VectorXd::Zero(n_v_), dv_)
                                 .evaluator()
                                 .get());
  }

  // 5. Joint Limit cost
  w_joint_limit_ = VectorXd::Zero(n_revolute_joints_);
  K_joint_pos = MatrixXd::Identity(n_revolute_joints_, n_revolute_joints_);
  joint_limit_cost_.push_back(
      prog_->AddLinearCost(w_joint_limit_, 0, dv_.tail(n_revolute_joints_))
          .evaluator()
          .get());

  // (Testing) 6. contact force blending
  if (ds_duration_ > 0) {
    epsilon_blend_ =
        prog_->NewContinuousVariables(n_c_ / kSpaceDim, "epsilon_blend");
    blend_constraint_ =
        prog_
            ->AddLinearEqualityConstraint(
                MatrixXd::Zero(1, 2 * n_c_ / kSpaceDim), VectorXd::Zero(1),
                {lambda_c_.segment(kSpaceDim * 0 + 2, 1),
                 lambda_c_.segment(kSpaceDim * 1 + 2, 1),
                 lambda_c_.segment(kSpaceDim * 2 + 2, 1),
                 lambda_c_.segment(kSpaceDim * 3 + 2, 1), epsilon_blend_})
            .evaluator()
            .get();
    /// Soft constraint version
    //  DRAKE_DEMAND(w_blend_constraint_ > 0);
    //  prog_->AddQuadraticCost(
    //      w_blend_constraint_ *
    //          MatrixXd::Identity(n_c_ / kSpaceDim, n_c_ / kSpaceDim),
    //      VectorXd::Zero(n_c_ / kSpaceDim), epsilon_blend_);
    /// hard constraint version
    prog_->AddBoundingBoxConstraint(0, 0, epsilon_blend_);
  }

  // (Testing) 7. Cost for staying close to the previous input
  if (w_input_reg_ > 0) {
    W_input_reg_ = w_input_reg_ * MatrixXd::Identity(n_u_, n_u_);
    input_reg_cost_ =
        prog_->AddQuadraticCost(W_input_reg_, VectorXd::Zero(n_u_), u_)
            .evaluator()
            .get();
  }

  solver_ = std::make_unique<solvers::FastOsqpSolver>();
  drake::solvers::SolverOptions solver_options;
  solver_options.SetOption(OsqpSolver::id(), "verbose", 0);
//  solver_options.SetOption(OsqpSolver::id(), "time_limit", qp_time_limit_);
  solver_options.SetOption(OsqpSolver::id(), "eps_abs", 1e-7);
  solver_options.SetOption(OsqpSolver::id(), "eps_rel", 1e-7);
  solver_options.SetOption(OsqpSolver::id(), "eps_prim_inf", 1e-6);
  solver_options.SetOption(OsqpSolver::id(), "eps_dual_inf", 1e-6);
  solver_options.SetOption(OsqpSolver::id(), "polish", 1);
  solver_options.SetOption(OsqpSolver::id(), "scaled_termination", 1);
  solver_options.SetOption(OsqpSolver::id(), "adaptive_rho_fraction", 1);
  std::cout << solver_options << std::endl;
  solver_->InitializeSolver(*prog_, solver_options);
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

  auto prev_fsm_state = discrete_state->get_mutable_vector(prev_fsm_state_idx_)
                            .get_mutable_value();
  if (fsm_state(0) != prev_fsm_state(0)) {
    prev_distinct_fsm_state_ = prev_fsm_state(0);
    prev_fsm_state(0) = fsm_state(0);

    discrete_state->get_mutable_vector(prev_event_time_idx_).get_mutable_value()
        << timestamp;
  }
  return drake::systems::EventStatus::Succeeded();
}

VectorXd OperationalSpaceControl::SolveQp(
    const VectorXd& x_w_spr, const VectorXd& x_wo_spr,
    const drake::systems::Context<double>& context, double t, int fsm_state,
    double time_since_last_state_switch, double alpha,
    int next_fsm_state) const {
  // Get active contact indices
  std::set<int> active_contact_set = {};
  if (single_contact_mode_) {
    active_contact_set = contact_indices_map_.at(-1);
  } else {
    auto map_iterator = contact_indices_map_.find(fsm_state);
    if (map_iterator != contact_indices_map_.end()) {
      active_contact_set = map_iterator->second;
    } else {
      static const drake::logging::Warn log_once(const_cast<char*>(
          (std::to_string(fsm_state) +
           " is not a valid finite state machine state in OSC.")
              .c_str()));
    }
  }

  // Update context
  SetPositionsIfNew<double>(
      plant_w_spr_, x_w_spr.head(plant_w_spr_.num_positions()), context_w_spr_);
  SetVelocitiesIfNew<double>(plant_w_spr_,
                             x_w_spr.tail(plant_w_spr_.num_velocities()),
                             context_w_spr_);
  SetPositionsIfNew<double>(plant_wo_spr_,
                            x_wo_spr.head(plant_wo_spr_.num_positions()),
                            context_wo_spr_);
  SetVelocitiesIfNew<double>(plant_wo_spr_,
                             x_wo_spr.tail(plant_wo_spr_.num_velocities()),
                             context_wo_spr_);

  // Get M, f_cg, B matrices of the manipulator equation
  MatrixXd B = plant_wo_spr_.MakeActuationMatrix();
  MatrixXd M(n_v_, n_v_);
  plant_wo_spr_.CalcMassMatrix(*context_wo_spr_, &M);
  VectorXd bias(n_v_);
  plant_wo_spr_.CalcBiasTerm(*context_wo_spr_, &bias);
  drake::multibody::MultibodyForces<double> f_app(plant_wo_spr_);
  plant_wo_spr_.CalcForceElementsContribution(*context_wo_spr_, &f_app);
  VectorXd grav = plant_wo_spr_.CalcGravityGeneralizedForces(*context_wo_spr_);
  bias = bias - grav;
  // TODO (yangwill): Characterize damping in cassie model
  //  bias = bias - f_app.generalized_forces();

  // Get J and JdotV for holonomic constraint
  MatrixXd J_h(n_h_, n_v_);
  VectorXd JdotV_h(n_h_);
  if (kinematic_evaluators_ != nullptr) {
    J_h = kinematic_evaluators_->EvalFullJacobian(*context_wo_spr_);
    JdotV_h =
        kinematic_evaluators_->EvalFullJacobianDotTimesV(*context_wo_spr_);
  }

  // Get J for external forces in equations of motion
  MatrixXd J_c = MatrixXd::Zero(n_c_, n_v_);
  for (unsigned int i = 0; i < all_contacts_.size(); i++) {
    if (active_contact_set.find(i) != active_contact_set.end()) {
      J_c.block(kSpaceDim * i, 0, kSpaceDim, n_v_) =
          all_contacts_[i]->EvalFullJacobian(*context_wo_spr_);
    }
  }

  // Get J and JdotV for contact constraint
  MatrixXd J_c_active = MatrixXd::Zero(n_c_active_, n_v_);
  VectorXd JdotV_c_active = VectorXd::Zero(n_c_active_);
  int row_idx = 0;
  for (unsigned int i = 0; i < all_contacts_.size(); i++) {
    auto contact_i = all_contacts_[i];
    if (active_contact_set.find(i) != active_contact_set.end()) {
      // We don't call EvalActiveJacobian() because it'll repeat the computation
      // of the Jacobian. (J_c_active is just a stack of slices of J_c)
      for (int j = 0; j < contact_i->num_active(); j++) {
        J_c_active.row(row_idx + j) =
            J_c.row(kSpaceDim * i + contact_i->active_inds().at(j));
      }
      JdotV_c_active.segment(row_idx, contact_i->num_active()) =
          contact_i->EvalActiveJacobianDotTimesV(*context_wo_spr_);
    }
    row_idx += contact_i->num_active();
  }

  // Update constraints
  // 1. Dynamics constraint
  ///    M*dv + bias == J_c^T*lambda_c + J_h^T*lambda_h + B*u
  /// -> M*dv - J_c^T*lambda_c - J_h^T*lambda_h - B*u == - bias
  /// -> [M, -J_c^T, -J_h^T, -B]*[dv, lambda_c, lambda_h, u]^T = - bias
  MatrixXd A_dyn = MatrixXd::Zero(n_v_, n_v_ + n_c_ + n_h_ + n_u_);
  A_dyn.block(0, 0, n_v_, n_v_) = M;
  A_dyn.block(0, n_v_, n_v_, n_c_) = -J_c.transpose();
  A_dyn.block(0, n_v_ + n_c_, n_v_, n_h_) = -J_h.transpose();
  A_dyn.block(0, n_v_ + n_c_ + n_h_, n_v_, n_u_) = -B;
  dynamics_constraint_->UpdateCoefficients(A_dyn, -bias);
  // 2. Holonomic constraint
  ///    JdotV_h + J_h*dv == 0
  /// -> J_h*dv == -JdotV_h
  holonomic_constraint_->UpdateCoefficients(J_h, -JdotV_h);
  // 3. Contact constraint
  if (!all_contacts_.empty()) {
    if (w_soft_constraint_ <= 0) {
      ///    JdotV_c_active + J_c_active*dv == 0
      /// -> J_c_active*dv == -JdotV_c_active
      contact_constraints_->UpdateCoefficients(J_c_active, -JdotV_c_active);
    } else {
      // Relaxed version:
      ///    JdotV_c_active + J_c_active*dv == -epsilon
      /// -> J_c_active*dv + I*epsilon == -JdotV_c_active
      /// -> [J_c_active, I]* [dv, epsilon]^T == -JdotV_c_active
      MatrixXd A_c = MatrixXd::Zero(n_c_active_, n_v_ + n_c_active_);
      A_c.block(0, 0, n_c_active_, n_v_) = J_c_active;
      A_c.block(0, n_v_, n_c_active_, n_c_active_) =
          MatrixXd::Identity(n_c_active_, n_c_active_);
      contact_constraints_->UpdateCoefficients(A_c, -JdotV_c_active);
    }
  }
  // 4. Friction constraint (approximated firction cone)
  /// For i = active contact indices
  ///     mu_*lambda_c(3*i+2) >= lambda_c(3*i+0)
  ///    -mu_*lambda_c(3*i+2) <= lambda_c(3*i+0)
  ///     mu_*lambda_c(3*i+2) >= lambda_c(3*i+1)
  ///    -mu_*lambda_c(3*i+2) <= lambda_c(3*i+1)
  ///         lambda_c(3*i+2) >= 0
  /// ->
  ///     mu_*lambda_c(3*i+2) - lambda_c(3*i+0) >= 0
  ///     mu_*lambda_c(3*i+2) + lambda_c(3*i+0) >= 0
  ///     mu_*lambda_c(3*i+2) - lambda_c(3*i+1) >= 0
  ///     mu_*lambda_c(3*i+2) + lambda_c(3*i+1) >= 0
  ///                           lambda_c(3*i+2) >= 0
  if (!all_contacts_.empty()) {
    for (unsigned int i = 0; i < all_contacts_.size(); i++) {
      if (active_contact_set.find(i) != active_contact_set.end()) {
        friction_constraints_.at(i)->UpdateLowerBound(VectorXd::Zero(5));
      } else {
        friction_constraints_.at(i)->UpdateLowerBound(
            VectorXd::Constant(5, -std::numeric_limits<double>::infinity()));
      }
    }
  }

  //  Invariant Impacts
  //  Only update when near an impact
  bool near_impact = alpha != 0;
  VectorXd v_proj = VectorXd::Zero(n_v_);
  if (near_impact) {
    UpdateImpactInvariantProjection(x_w_spr, x_wo_spr, context, t,
                                    time_since_last_state_switch, fsm_state,
                                    next_fsm_state, M, J_h);
    // Need to call Update before this to get the updated jacobian
    v_proj = alpha * M_Jt_ * ii_lambda_sol_;
  }

  // Update costs
  // 4. Tracking cost
  for (unsigned int i = 0; i < tracking_data_vec_->size(); i++) {
    auto tracking_data = tracking_data_vec_->at(i);

    if (tracking_data->IsActive(fsm_state) &&
        time_since_last_state_switch >= t_s_vec_.at(i) &&
        time_since_last_state_switch <= t_e_vec_.at(i)) {
      // Check whether or not it is a constant trajectory, and update
      // TrackingData
      if (fixed_position_vec_.at(i).size() != 0) {
        // Create constant trajectory and update
        tracking_data->Update(
            x_w_spr, *context_w_spr_, x_wo_spr, *context_wo_spr_,
            PiecewisePolynomial<double>(fixed_position_vec_.at(i)), t,
            time_since_last_state_switch, fsm_state, v_proj);
      } else {
        // Read in traj from input port
        const string& traj_name = tracking_data->GetName();
        int port_index = traj_name_to_port_index_map_.at(traj_name);
        const drake::AbstractValue* input_traj =
            this->EvalAbstractInput(context, port_index);
        DRAKE_DEMAND(input_traj != nullptr);
        const auto& traj =
            input_traj->get_value<drake::trajectories::Trajectory<double>>();
        // Update
        tracking_data->Update(x_w_spr, *context_w_spr_, x_wo_spr,
                              *context_wo_spr_, traj, t,
                              time_since_last_state_switch, fsm_state, v_proj);
      }

      const VectorXd& ddy_t = tracking_data->GetYddotCommand();
      const MatrixXd& W = tracking_data->GetWeight();
      const MatrixXd& J_t = tracking_data->GetJ();
      const VectorXd& JdotV_t = tracking_data->GetJdotTimesV();
      // The tracking cost is
      // 0.5 * (J_*dv + JdotV - y_command)^T * W * (J_*dv + JdotV - y_command).
      // We ignore the constant term
      // 0.5 * (JdotV - y_command)^T * W * (JdotV - y_command),
      // since it doesn't change the result of QP.
      tracking_cost_.at(i)->UpdateCoefficients(
          J_t.transpose() * W * J_t, J_t.transpose() * W * (JdotV_t - ddy_t));
    } else {
      tracking_cost_.at(i)->UpdateCoefficients(MatrixXd::Zero(n_v_, n_v_),
                                               VectorXd::Zero(n_v_));
    }
  }

  // Add joint limit constraints
  VectorXd w_joint_limit =
      K_joint_pos * (x_wo_spr.head(plant_wo_spr_.num_positions())
                         .tail(n_revolute_joints_) -
                     q_max_)
                        .cwiseMax(0) +
      K_joint_pos * (x_wo_spr.head(plant_wo_spr_.num_positions())
                         .tail(n_revolute_joints_) -
                     q_min_)
                        .cwiseMin(0);
  joint_limit_cost_.at(0)->UpdateCoefficients(w_joint_limit, 0);

  // (Testing) 6. blend contact forces during double support phase
  if (ds_duration_ > 0) {
    MatrixXd A = MatrixXd::Zero(1, 2 * n_c_ / kSpaceDim);
    if (std::find(ds_states_.begin(), ds_states_.end(), fsm_state) !=
        ds_states_.end()) {
      double alpha_left = 0;
      double alpha_right = 0;
      if (prev_distinct_fsm_state_ == right_support_state_) {
        // We want left foot force to gradually increase
        alpha_left = -1;
        alpha_right = time_since_last_state_switch /
                      (ds_duration_ - time_since_last_state_switch);

      } else if (prev_distinct_fsm_state_ == left_support_state_) {
        alpha_left = time_since_last_state_switch /
                     (ds_duration_ - time_since_last_state_switch);
        alpha_right = -1;
      }
      A(0, 0) = alpha_left / 2;
      A(0, 1) = alpha_left / 2;
      A(0, 2) = alpha_right / 2;
      A(0, 3) = alpha_right / 2;
      A(0, 4) = 1;
      A(0, 5) = 1;
      A(0, 6) = 1;
      A(0, 7) = 1;
    }
    blend_constraint_->UpdateCoefficients(A, VectorXd::Zero(1));
  }

  // (Testing) 7. Cost for staying close to the previous input
  if (w_input_reg_ > 0) {
    input_reg_cost_->UpdateCoefficients(W_input_reg_,
                                        -W_input_reg_ * (*u_sol_));
  }

  // Solve the QP
  const MathematicalProgramResult result = solver_->Solve(*prog_);

  solve_time_ = result.get_solver_details<OsqpSolver>().run_time;

  if (!result.is_success()) {
    std::cout << "reverting to old sol - "
              <<  result.get_solution_result() << std::endl;
    return *u_sol_;
  }

  // Extract solutions
  *dv_sol_ = result.GetSolution(dv_);
  *u_sol_ = result.GetSolution(u_);
  *lambda_c_sol_ = result.GetSolution(lambda_c_);
  *lambda_h_sol_ = result.GetSolution(lambda_h_);
  *epsilon_sol_ = result.GetSolution(epsilon_);

  for (auto tracking_data : *tracking_data_vec_) {
    if (tracking_data->IsActive(fsm_state)) {
      tracking_data->StoreYddotCommandSol(*dv_sol_);
    }
  }

  return *u_sol_;
}
void OperationalSpaceControl::UpdateImpactInvariantProjection(
    const VectorXd& x_w_spr, const VectorXd& x_wo_spr,
    const Context<double>& context, double t, double t_since_last_state_switch,
    int fsm_state, int next_fsm_state, const MatrixXd& M,
    const MatrixXd& J_h) const {
  auto map_iterator = contact_indices_map_.find(next_fsm_state);
  if (map_iterator == contact_indices_map_.end()) {
    throw std::out_of_range("Contact mode: " + std::to_string(next_fsm_state) +
                            " was not found in the OSC");
  }
  std::set<int> next_contact_set = map_iterator->second;
  int active_contact_dim = active_contact_dim_.at(next_fsm_state) + n_h_;
  MatrixXd J_c_next = MatrixXd::Zero(active_contact_dim, n_v_);
  int row_start = 0;
  for (unsigned int i = 0; i < all_contacts_.size(); i++) {
    if (next_contact_set.find(i) != next_contact_set.end()) {
      J_c_next.block(row_start, 0, kSpaceDim, n_v_) =
          all_contacts_[i]->EvalFullJacobian(*context_wo_spr_);
      row_start += kSpaceDim;
    }
  }
  // Holonomic constraints
  if (n_h_ > 0) {
    J_c_next.block(row_start, 0, n_h_, n_v_) = J_h;
  }
  M_Jt_ = M.llt().solve(J_c_next.transpose());

  int active_tracking_data_dim = 0;
  for (unsigned int i = 0; i < tracking_data_vec_->size(); i++) {
    auto tracking_data = tracking_data_vec_->at(i);

    if (tracking_data->IsActive(fsm_state) &&
        tracking_data->GetImpactInvariantProjection()) {
      VectorXd v_proj = VectorXd::Zero(n_v_);
      active_tracking_data_dim += tracking_data->GetYDim();
      if (fixed_position_vec_.at(i).size() != 0) {
        // Create constant trajectory and update
        tracking_data->Update(
            x_w_spr, *context_w_spr_, x_wo_spr, *context_wo_spr_,
            PiecewisePolynomial<double>(fixed_position_vec_.at(i)), t,
            t_since_last_state_switch, fsm_state, v_proj);
      } else {
        // Read in traj from input port
        const string& traj_name = tracking_data->GetName();
        int port_index = traj_name_to_port_index_map_.at(traj_name);
        const drake::AbstractValue* input_traj =
            EvalAbstractInput(context, port_index);
        const auto& traj =
            input_traj->get_value<drake::trajectories::Trajectory<double>>();
        tracking_data->Update(x_w_spr, *context_w_spr_, x_wo_spr,
                              *context_wo_spr_, traj, t,
                              t_since_last_state_switch, fsm_state, v_proj);
      }
    }
  }
  MatrixXd A = MatrixXd::Zero(active_tracking_data_dim, active_contact_dim);
  VectorXd ydot_err_vec = VectorXd::Zero(active_tracking_data_dim);
  int start_row = 0;
  for (auto tracking_data : *tracking_data_vec_) {
    if (tracking_data->IsActive(fsm_state) &&
        tracking_data->GetImpactInvariantProjection()) {
      A.block(start_row, 0, tracking_data->GetYDim(), active_contact_dim) =
          tracking_data->GetJ() * M_Jt_;
      ydot_err_vec.segment(start_row, tracking_data->GetYDim()) =
          tracking_data->GetErrorYdot();
      start_row += tracking_data->GetYDim();
    }
  }

  ii_lambda_sol_ = A.completeOrthogonalDecomposition().solve(ydot_err_vec);
}

void OperationalSpaceControl::AssignOscLcmOutput(
    const Context<double>& context, dairlib::lcmt_osc_output* output) const {
  auto state =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  int fsm_state = -1;
  if (used_with_finite_state_machine_) {
    auto fsm_output =
        (BasicVector<double>*)this->EvalVectorInput(context, fsm_port_);
    fsm_state = fsm_output->get_value()(0);
  }

  double time_since_last_state_switch =
      used_with_finite_state_machine_
          ? state->get_timestamp() -
                context.get_discrete_state(prev_event_time_idx_).get_value()(0)
          : state->get_timestamp();

  output->utime = state->get_timestamp() * 1e6;
  output->fsm_state = fsm_state;
  output->input_cost =
      (W_input_.size() > 0)
          ? (0.5 * (*u_sol_).transpose() * W_input_ * (*u_sol_))(0)
          : 0;
  output->acceleration_cost =
      (W_joint_accel_.size() > 0)
          ? (0.5 * (*dv_sol_).transpose() * W_joint_accel_ * (*dv_sol_))(0)
          : 0;
  output->soft_constraint_cost =
      (w_soft_constraint_ > 0)
          ? (0.5 * w_soft_constraint_ * (*epsilon_sol_).transpose() *
             (*epsilon_sol_))(0)
          : 0;

  output->tracking_data_names.clear();
  output->tracking_data.clear();
  output->tracking_cost.clear();

  lcmt_osc_qp_output qp_output;
  qp_output.solve_time = solve_time_;
  qp_output.u_dim = n_u_;
  qp_output.lambda_c_dim = n_c_;
  qp_output.lambda_h_dim = n_h_;
  qp_output.v_dim = n_v_;
  qp_output.epsilon_dim = n_c_active_;
  qp_output.u_sol = CopyVectorXdToStdVector(*u_sol_);
  qp_output.lambda_c_sol = CopyVectorXdToStdVector(*lambda_c_sol_);
  qp_output.lambda_h_sol = CopyVectorXdToStdVector(*lambda_h_sol_);
  qp_output.dv_sol = CopyVectorXdToStdVector(*dv_sol_);
  qp_output.epsilon_sol = CopyVectorXdToStdVector(*epsilon_sol_);
  output->qp_output = qp_output;

  output->tracking_data.reserve(tracking_data_vec_->size());
  output->tracking_cost = std::vector<double>(tracking_data_vec_->size());

  for (unsigned int i = 0; i < tracking_data_vec_->size(); i++) {
    auto tracking_data = tracking_data_vec_->at(i);

    if (tracking_data->IsActive(fsm_state) &&
        time_since_last_state_switch >= t_s_vec_.at(i) &&
        time_since_last_state_switch <= t_e_vec_.at(i)) {
      output->tracking_data_names.push_back(tracking_data->GetName());
      lcmt_osc_tracking_data osc_output;
      osc_output.y_dim = tracking_data->GetYDim();
      osc_output.ydot_dim = tracking_data->GetYdotDim();
      osc_output.name = tracking_data->GetName();
      // This should always be true
      osc_output.is_active = tracking_data->IsActive(fsm_state);
      osc_output.y = CopyVectorXdToStdVector(tracking_data->GetY());
      osc_output.y_des = CopyVectorXdToStdVector(tracking_data->GetYDes());
      osc_output.error_y = CopyVectorXdToStdVector(tracking_data->GetErrorY());
      osc_output.ydot = CopyVectorXdToStdVector(tracking_data->GetYdot());
      osc_output.ydot_des =
          CopyVectorXdToStdVector(tracking_data->GetYdotDes());
      osc_output.error_ydot =
          CopyVectorXdToStdVector(tracking_data->GetErrorYdot());
      osc_output.yddot_des =
          CopyVectorXdToStdVector(tracking_data->GetYddotDes());
      osc_output.yddot_command =
          CopyVectorXdToStdVector(tracking_data->GetYddotCommand());
      osc_output.yddot_command_sol =
          CopyVectorXdToStdVector(tracking_data->GetYddotCommandSol());
      output->tracking_data.push_back(osc_output);

      const VectorXd& ddy_t = tracking_data->GetYddotCommand();
      const MatrixXd& W = tracking_data->GetWeight();
      const MatrixXd& J_t = tracking_data->GetJ();
      const VectorXd& JdotV_t = tracking_data->GetJdotTimesV();
      output->tracking_cost[i] =
          (0.5 * (J_t * (*dv_sol_) + JdotV_t - ddy_t).transpose() * W *
           (J_t * (*dv_sol_) + JdotV_t - ddy_t))(0);
    }
  }

  output->num_tracking_data = output->tracking_data_names.size();
}

void OperationalSpaceControl::CalcOptimalInput(
    const drake::systems::Context<double>& context,
    systems::TimestampedVector<double>* control) const {
  // Read in current state and time
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  VectorXd q_w_spr = robot_output->GetPositions();
  VectorXd v_w_spr = robot_output->GetVelocities();

  VectorXd x_w_spr(plant_w_spr_.num_positions() +
                   plant_w_spr_.num_velocities());
  x_w_spr << q_w_spr, v_w_spr;

  double timestamp = robot_output->get_timestamp();
  double current_time = timestamp;
  if (print_tracking_info_) {
    cout << "\n\ncurrent_time = " << current_time << endl;
  }

  VectorXd x_wo_spr(n_q_ + n_v_);
  x_wo_spr << map_position_from_spring_to_no_spring_ * q_w_spr,
      map_velocity_from_spring_to_no_spring_ * v_w_spr;

  VectorXd u_sol(n_u_);
  if (used_with_finite_state_machine_) {
    // Read in finite state machine
    const BasicVector<double>* fsm_output =
        (BasicVector<double>*)this->EvalVectorInput(context, fsm_port_);
    VectorXd fsm_state = fsm_output->get_value();

    double alpha = 0;
    int next_fsm_state = -1;
    if (this->get_near_impact_input_port().HasValue(context)) {
      const BasicVector<double>* near_impact =
          (BasicVector<double>*)this->EvalVectorInput(context,
                                                      near_impact_port_);
      alpha = near_impact->get_value()(0);
      next_fsm_state = near_impact->get_value()(1);
    }

    // Get discrete states
    const auto prev_event_time =
        context.get_discrete_state(prev_event_time_idx_).get_value();

    u_sol = SolveQp(x_w_spr, x_wo_spr, context, current_time, fsm_state(0),
                    current_time - prev_event_time(0), alpha, next_fsm_state);
  } else {
    u_sol = SolveQp(x_w_spr, x_wo_spr, context, current_time, -1, current_time,
                    0, -1);
  }

  // Assign the control input
  control->SetDataVector(u_sol);
  control->set_timestamp(robot_output->get_timestamp());
}

}  // namespace dairlib::systems::controllers
