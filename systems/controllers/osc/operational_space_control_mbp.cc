#include "systems/controllers/osc/operational_space_control_mbp.h"
#include <drake/multibody/plant/multibody_plant.h>
#include "multibody/multibody_utils.h"

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

using drake::solvers::Solve;

namespace dairlib::systems::controllers {

using multibody::makeNameToVelocitiesMap;

OperationalSpaceControlMBP::OperationalSpaceControlMBP(
    const MultibodyPlant<double>& plant_w_spr,
    const MultibodyPlant<double>& plant_wo_spr,
    bool used_with_finite_state_machine, bool print_tracking_info)
    : plant_w_spr_(plant_w_spr),
      plant_wo_spr_(plant_wo_spr),
      world_w_spr_(plant_w_spr_.world_frame()),
      world_wo_spr_(plant_wo_spr_.world_frame()),
      used_with_finite_state_machine_(used_with_finite_state_machine),
      print_tracking_info_(print_tracking_info) {
  this->set_name("OSC");

  n_q_ = plant_wo_spr.num_positions();
  n_v_ = plant_wo_spr.num_velocities();
  n_u_ = plant_wo_spr.num_actuators();

  int n_q_w_spr = plant_w_spr.num_positions();
  int n_v_w_spr = plant_w_spr.num_velocities();
  int n_u_w_spr = plant_w_spr.num_actuators();

  // Input/Output Setup
  state_port_ = this->DeclareVectorInputPort(
                        OutputVector<double>(n_q_w_spr, n_v_w_spr, n_u_w_spr))
                    .get_index();
  this->DeclareVectorOutputPort(TimestampedVector<double>(n_u_w_spr),
                                &OperationalSpaceControlMBP::CalcOptimalInput);
  if (used_with_finite_state_machine) {
    fsm_port_ =
        this->DeclareVectorInputPort(BasicVector<double>(1)).get_index();

    // Discrete update to record the last state event time
    DeclarePerStepDiscreteUpdateEvent(
        &OperationalSpaceControlMBP::DiscreteVariableUpdate);
    prev_fsm_state_idx_ = this->DeclareDiscreteState(-0.1 * VectorXd::Ones(1));
    prev_event_time_idx_ = this->DeclareDiscreteState(VectorXd::Zero(1));
  }

  const std::map<string, int>& pos_map_w_spr =
      multibody::makeNameToPositionsMap(plant_w_spr);
  const std::map<string, int>& vel_map_w_spr =
      multibody::makeNameToVelocitiesMap(plant_w_spr);
  const std::map<string, int>& pos_map_wo_spr =
      multibody::makeNameToPositionsMap(plant_wo_spr);
  const std::map<string, int>& vel_map_wo_spr =
      multibody::makeNameToVelocitiesMap(plant_wo_spr);

  // Initialize the mapping from spring to no spring
  map_position_from_spring_to_no_spring_ = MatrixXd::Zero(n_q_, n_q_w_spr);
  map_velocity_from_spring_to_no_spring_ = MatrixXd::Zero(n_v_, n_v_w_spr);

  for (auto pos_pair_wo_spr : pos_map_wo_spr) {
    bool successfully_added = false;
    for (auto pos_pair_w_spr : pos_map_w_spr) {
      if (pos_pair_wo_spr.first == pos_pair_w_spr.first) {
        map_position_from_spring_to_no_spring_(pos_pair_wo_spr.second,
            pos_pair_w_spr.second) = 1;
        successfully_added = true;
      }
    }
    DRAKE_DEMAND(successfully_added);
  }

  for (auto vel_pair_wo_spr : vel_map_wo_spr) {
    bool successfully_added = false;
    for (auto vel_pair_w_spr : vel_map_w_spr) {
      if (vel_pair_wo_spr.first == vel_pair_w_spr.first) {
        map_velocity_from_spring_to_no_spring_(vel_pair_wo_spr.second,
            vel_pair_w_spr.second) = 1;
        successfully_added = true;
      }
    }
    DRAKE_DEMAND(successfully_added);
  }

  // Get input limits
  VectorXd u_min(n_u_);
  VectorXd u_max(n_u_);
  //  for (int i = 0; i < n_u_; i++) {
  for (JointActuatorIndex i(0); i < n_u_; ++i) {
    u_min(i) = -300;
    u_max(i) = 300;
    //    u_min(i) = -plant_wo_spr_.get_joint_actuator(i).effort_limit();
    //    u_max(i) = plant_wo_spr_.get_joint_actuator(i).effort_limit();
  }
  u_min_ = u_min;
  u_max_ = u_max;

  // Check if the model is floating based
  //  is_quaternion_ = multibody::IsFloatingBase(plant_w_spr);
  is_quaternion_ = multibody::isQuaternion(plant_w_spr);
}

// Cost methods
void OperationalSpaceControlMBP::AddAccelerationCost(
    const std::string& joint_vel_name, double w) {
  if (W_joint_accel_.size() == 0) {
    W_joint_accel_ = Eigen::MatrixXd::Zero(n_v_, n_v_);
  }
  int idx = makeNameToVelocitiesMap(plant_wo_spr_).at(joint_vel_name);
  W_joint_accel_(idx, idx) += w;
}

// Constraint methods
void OperationalSpaceControlMBP::AddContactPoint(const std::string& body_name,
                                                 const VectorXd& pt_on_body) {
  body_indices_.push_back(plant_wo_spr_.GetBodyByName(body_name).index());
  pts_on_body_.push_back(pt_on_body);
}

void OperationalSpaceControlMBP::AddStateAndContactPoint(int state,
                                                         std::string body_name,
                                                         VectorXd pt_on_body) {
  fsm_state_when_active_.push_back(state);
  AddContactPoint(body_name, pt_on_body);
}

//
void OperationalSpaceControlMBP::AddDistanceConstraint(
    multibody::MultibodyDistanceConstraint& constraint) {
  distance_constraints_.push_back(&constraint);
}

// Tracking data methods
void OperationalSpaceControlMBP::AddTrackingData(
    OscTrackingDataMBP* tracking_data, double t_lb, double t_ub) {
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
void OperationalSpaceControlMBP::AddConstTrackingData(
    OscTrackingDataMBP* tracking_data, VectorXd v, double t_lb, double t_ub) {
  tracking_data_vec_->push_back(tracking_data);
  fixed_position_vec_.push_back(v);
  t_s_vec_.push_back(t_lb);
  t_e_vec_.push_back(t_ub);
}

// Osc checkers and constructor
void OperationalSpaceControlMBP::CheckCostSettings() {
  if (W_input_.size() != 0) {
    DRAKE_DEMAND((W_input_.rows() == n_u_) && (W_input_.cols() == n_u_));
  }
  if (W_joint_accel_.size() != 0) {
    DRAKE_DEMAND((W_joint_accel_.rows() == n_v_) &&
                 (W_joint_accel_.cols() == n_v_));
  }
}
void OperationalSpaceControlMBP::CheckConstraintSettings() {
  if (!body_indices_.empty()) {
    DRAKE_DEMAND(mu_ != -1);
    DRAKE_DEMAND(body_indices_.size() == pts_on_body_.size());
  }
  if (!fsm_state_when_active_.empty()) {
    DRAKE_DEMAND(fsm_state_when_active_.size() == body_indices_.size());
    DRAKE_DEMAND(fsm_state_when_active_.size() == pts_on_body_.size());
  }
}

void OperationalSpaceControlMBP::Build() {
  // Checker
  CheckCostSettings();
  CheckConstraintSettings();
  for (auto tracking_data : *tracking_data_vec_) {
    tracking_data->CheckOscTrackingData();
  }

  // Construct QP
  //  n_h_ = plant_wo_spr_.getNumPositionConstraints();
  //  n_h_ = plant_wo_spr_.num_constraints();
  n_h_ = distance_constraints_.size();
  n_c_ = 3 * body_indices_.size();
  prog_ = std::make_unique<MathematicalProgram>();

  // Add decision variables
  dv_ = prog_->NewContinuousVariables(n_v_, "dv");
  u_ = prog_->NewContinuousVariables(n_u_, "u");
  lambda_c_ = prog_->NewContinuousVariables(n_c_, "lambda_contact");
  lambda_h_ = prog_->NewContinuousVariables(n_h_, "lambda_holonomic");
  epsilon_ = prog_->NewContinuousVariables(n_c_, "epsilon");

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
  if (body_indices_.size() > 0) {
    if (w_soft_constraint_ <= 0) {
      contact_constraints_ =
          prog_
              ->AddLinearEqualityConstraint(MatrixXd::Zero(n_c_, n_v_),
                                            VectorXd::Zero(n_c_), dv_)
              .evaluator()
              .get();
    } else {
      // Relaxed version:
      contact_constraints_ = prog_
          ->AddLinearEqualityConstraint(
                                     MatrixXd::Zero(n_c_, n_v_ + n_c_),
                                     VectorXd::Zero(n_c_), {dv_, epsilon_})
                                 .evaluator()
                                 .get();
    }
  }
  // 4. Friction constraint (approximated firction cone)
  if (!body_indices_.empty()) {
    VectorXd mu_minus1(2);
    mu_minus1 << mu_, -1;
    VectorXd mu_plus1(2);
    mu_plus1 << mu_, 1;
    VectorXd one(1);
    one << 1;
    for (unsigned int j = 0; j < body_indices_.size(); j++) {
      friction_constraints_.push_back(
          prog_
              ->AddLinearConstraint(mu_minus1.transpose(), 0,
                                    numeric_limits<double>::infinity(),
                                    {lambda_c_.segment(3 * j + 2, 1),
                                     lambda_c_.segment(3 * j + 0, 1)})
              .evaluator()
              .get());
      friction_constraints_.push_back(
          prog_
              ->AddLinearConstraint(mu_plus1.transpose(), 0,
                                    numeric_limits<double>::infinity(),
                                    {lambda_c_.segment(3 * j + 2, 1),
                                     lambda_c_.segment(3 * j + 0, 1)})
              .evaluator()
              .get());
      friction_constraints_.push_back(
          prog_
              ->AddLinearConstraint(mu_minus1.transpose(), 0,
                                    numeric_limits<double>::infinity(),
                                    {lambda_c_.segment(3 * j + 2, 1),
                                     lambda_c_.segment(3 * j + 1, 1)})
              .evaluator()
              .get());
      friction_constraints_.push_back(
          prog_
              ->AddLinearConstraint(mu_plus1.transpose(), 0,
                                    numeric_limits<double>::infinity(),
                                    {lambda_c_.segment(3 * j + 2, 1),
                                     lambda_c_.segment(3 * j + 1, 1)})
              .evaluator()
              .get());
      friction_constraints_.push_back(
          prog_
              ->AddLinearConstraint(one.transpose(), 0,
                                    numeric_limits<double>::infinity(),
                                    lambda_c_.segment(3 * j + 2, 1))
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
    prog_->AddQuadraticCost(w_soft_constraint_ * MatrixXd::Identity(n_c_, n_c_),
                            VectorXd::Zero(n_c_), epsilon_);
  }
  // 4. Tracking cost
  for (unsigned int i = 0; i < tracking_data_vec_->size(); i++) {
    tracking_cost_.push_back(prog_
        ->AddQuadraticCost(MatrixXd::Zero(n_v_, n_v_),
                                                    VectorXd::Zero(n_v_), dv_)
                                 .evaluator()
                                 .get());
  }
}

std::vector<bool> OperationalSpaceControlMBP::CalcActiveContactIndices(
    int fsm_state) const {
  std::vector<bool> active_contact_flags;
  for (unsigned int i = 0; i < body_indices_.size(); i++) {
    if (fsm_state_when_active_.empty()) {
      active_contact_flags.push_back(true);
    } else {
      active_contact_flags.push_back(fsm_state_when_active_[i] == fsm_state);
    }
  }
  return active_contact_flags;
}

drake::systems::EventStatus OperationalSpaceControlMBP::DiscreteVariableUpdate(
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
    prev_fsm_state(0) = fsm_state(0);

    discrete_state->get_mutable_vector(prev_event_time_idx_).get_mutable_value()
        << timestamp;
  }
  return drake::systems::EventStatus::Succeeded();
}

VectorXd OperationalSpaceControlMBP::SolveQp(
    const VectorXd& x_w_spr, const VectorXd& x_wo_spr,
    const drake::systems::Context<double>& context, double t, int fsm_state,
    double time_since_last_state_switch) const {
  vector<bool> active_contact_flags = CalcActiveContactIndices(fsm_state);

  std::unique_ptr<drake::systems::Context<double>> context_w_spr =
      plant_w_spr_.CreateDefaultContext();
  std::unique_ptr<drake::systems::Context<double>> context_wo_spr =
      plant_wo_spr_.CreateDefaultContext();
  plant_w_spr_.SetPositionsAndVelocities(context_w_spr.get(), x_w_spr);
  plant_wo_spr_.SetPositionsAndVelocities(context_wo_spr.get(), x_wo_spr);

  // Get M, f_cg, B matrices of the manipulator equation
  MatrixXd B = plant_wo_spr_.MakeActuationMatrix();
  //  MatrixXd B = plant_wo_spr_.B;
  MatrixXd M(n_v_, n_v_);
  plant_wo_spr_.CalcMassMatrixViaInverseDynamics(*context_wo_spr, &M);
  const RigidBodyTree<double>::BodyToWrenchMap no_external_wrenches;
  VectorXd bias(n_v_);
  plant_wo_spr_.CalcBiasTerm(*context_wo_spr, &bias);
  VectorXd grav = plant_wo_spr_.CalcGravityGeneralizedForces(*context_wo_spr);
  bias = bias - grav;
  //  VectorXd bias =
  //      plant_wo_spr_.dynamicsBiasTerm(cache_wo_spr, no_external_wrenches);

  // Get J and JdotV for holonomic constraint

  MatrixXd J_h(distance_constraints_.size(), n_v_);
  VectorXd JdotV_h(distance_constraints_.size());
  for (unsigned int i = 0; i < distance_constraints_.size(); ++i) {
    distance_constraints_[i]->updateConstraint(*context_wo_spr);
    J_h.row(i) = distance_constraints_[i]->getJ();
    JdotV_h.segment(i, 1) = distance_constraints_[i]->getJdotv();
  }

  // Get J and JdotV for contact constraint
  MatrixXd J_c = MatrixXd::Zero(n_c_, n_v_);
  VectorXd JdotV_c = VectorXd::Zero(n_c_);
  for (unsigned int i = 0; i < active_contact_flags.size(); i++) {
    if (active_contact_flags[i]) {
      //      J_c.block(3 * i, 0, 3, n_v_) =
      //      plant_wo_spr_.transformPointsJacobian(
      //          cache_wo_spr, pts_on_body_[i], body_indices_[i], 0, false);
      MatrixXd J = MatrixXd::Zero(3, n_v_);
      plant_wo_spr_.CalcJacobianTranslationalVelocity(
          *context_wo_spr, JacobianWrtVariable::kV,
          plant_wo_spr_.get_body(body_indices_[i]).body_frame(),
          pts_on_body_[i], world_wo_spr_, world_wo_spr_, &J);
      J_c.block(3 * i, 0, 3, n_v_) = J;
      JdotV_c.segment(3 * i, 3) =
          plant_wo_spr_
              .CalcBiasForJacobianSpatialVelocity(
                  *context_wo_spr, JacobianWrtVariable::kV,
                  plant_wo_spr_.get_body(body_indices_[i]).body_frame(),
                  pts_on_body_[i], world_wo_spr_, world_wo_spr_)
              .tail(3);
    }
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
  if (w_soft_constraint_ <= 0) {
    ///    JdotV_c + J_c*dv == 0
    /// -> J_c*dv == -JdotV_c
    contact_constraints_->UpdateCoefficients(J_c, -JdotV_c);
  } else {
    // Relaxed version:
    ///    JdotV_c + J_c*dv == -epsilon
    /// -> J_c*dv + I*epsilon == -JdotV_c
    /// -> [J_c, I]* [dv, epsilon]^T == -JdotV_c
    MatrixXd A_c = MatrixXd::Zero(n_c_, n_v_ + n_c_);
    A_c.block(0, 0, n_c_, n_v_) = J_c;
    A_c.block(0, n_v_, n_c_, n_c_) = MatrixXd::Identity(n_c_, n_c_);
    contact_constraints_->UpdateCoefficients(A_c, -JdotV_c);
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
  if (!body_indices_.empty()) {
    VectorXd inf_vectorxd(1);
    inf_vectorxd << numeric_limits<double>::infinity();
    for (unsigned int i = 0; i < active_contact_flags.size(); i++) {
      // If the contact is inactive, we assign zeros to A matrix. (lb<=Ax<=ub)
      // The number "5" in "5 * i" below comes from the fact that there are five
      // constraints for each contact point.
      if (active_contact_flags[i]) {
        friction_constraints_.at(5 * i)->UpdateLowerBound(VectorXd::Zero(1));
        friction_constraints_.at(5 * i + 1)->UpdateLowerBound(
            VectorXd::Zero(1));
        friction_constraints_.at(5 * i + 2)->UpdateLowerBound(
            VectorXd::Zero(1));
        friction_constraints_.at(5 * i + 3)->UpdateLowerBound(
            VectorXd::Zero(1));
        friction_constraints_.at(5 * i + 4)->UpdateLowerBound(
            VectorXd::Zero(1));
      } else {
        friction_constraints_.at(5 * i)->UpdateLowerBound(-inf_vectorxd);
        friction_constraints_.at(5 * i + 1)->UpdateLowerBound(-inf_vectorxd);
        friction_constraints_.at(5 * i + 2)->UpdateLowerBound(-inf_vectorxd);
        friction_constraints_.at(5 * i + 3)->UpdateLowerBound(-inf_vectorxd);
        friction_constraints_.at(5 * i + 4)->UpdateLowerBound(-inf_vectorxd);
      }
    }
  }

  // Update costs
  // 4. Tracking cost
  for (unsigned int i = 0; i < tracking_data_vec_->size(); i++) {
    auto tracking_data = tracking_data_vec_->at(i);

    // Check whether or not it is a constant trajectory, and update TrackingData
    if (fixed_position_vec_.at(i).size() > 0) {
      // Create constant trajectory and update
      tracking_data->Update(
          x_w_spr, *context_w_spr, x_wo_spr, *context_wo_spr,
          PiecewisePolynomial<double>(fixed_position_vec_.at(i)), t, fsm_state);
    } else {
      // Read in traj from input port
      string traj_name = tracking_data->GetName();
      int port_index = traj_name_to_port_index_map_.at(traj_name);
      const drake::AbstractValue* traj_intput =
          this->EvalAbstractInput(context, port_index);
      DRAKE_DEMAND(traj_intput != nullptr);
      const auto& traj =
          traj_intput->get_value<drake::trajectories::Trajectory<double>>();
      // Update
      tracking_data->Update(x_w_spr, *context_w_spr, x_wo_spr, *context_wo_spr,
                            traj, t, fsm_state);
    }

    if (tracking_data->GetTrackOrNot() &&
        time_since_last_state_switch >= t_s_vec_.at(i) &&
        time_since_last_state_switch <= t_e_vec_.at(i)) {
      VectorXd ddy_t = tracking_data->GetCommandOutput();
      MatrixXd W = tracking_data->GetWeight();
      MatrixXd J_t = tracking_data->GetJ();
      VectorXd JdotV_t = tracking_data->GetJdotTimesV();
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

  // Solve the QP
  const MathematicalProgramResult result = Solve(*prog_);
  SolutionResult solution_result = result.get_solution_result();
  if (print_tracking_info_) {
    cout << "\n" << to_string(solution_result) << endl;
    cout << "fsm_state = " << fsm_state << endl;
  }

  // Extract solutions
  VectorXd u_sol = result.GetSolution(u_);
  VectorXd lambda_c_sol = result.GetSolution(lambda_c_);
  VectorXd lambda_h_sol = result.GetSolution(lambda_h_);
  VectorXd dv_sol = result.GetSolution(dv_);
  VectorXd epsilon_sol = result.GetSolution(epsilon_);
  if (print_tracking_info_) {
    cout << "**********************\n";
    cout << "u_sol = " << u_sol.transpose() << endl;
    cout << "lambda_c_sol = " << lambda_c_sol.transpose() << endl;
    cout << "lambda_h_sol = " << lambda_h_sol.transpose() << endl;
    cout << "dv_sol = " << dv_sol.transpose() << endl;
    cout << "epsilon_sol = " << epsilon_sol.transpose() << endl;
  }

  // Print QP result
  if (print_tracking_info_) {
    cout << "**********************\n";
    // 1. input cost
    if (W_input_.size() > 0) {
      cout << "input cost = " << 0.5 * u_sol.transpose() * W_input_ * u_sol
           << endl;
    }
    // 2. acceleration cost
    if (W_joint_accel_.size() > 0) {
      cout << "acceleration cost = "
           << 0.5 * dv_sol.transpose() * W_joint_accel_ * dv_sol << endl;
    }
    // 3. Soft constraint cost
    if (w_soft_constraint_ > 0) {
      cout << "soft constraint cost = "
           << 0.5 * w_soft_constraint_ * epsilon_sol.transpose() * epsilon_sol
           << endl;
    }
    // 4. Tracking cost
    for (auto tracking_data : *tracking_data_vec_) {
      if (tracking_data->GetTrackOrNot()) {
        VectorXd ddy_t = tracking_data->GetCommandOutput();
        MatrixXd W = tracking_data->GetWeight();
        MatrixXd J_t = tracking_data->GetJ();
        VectorXd JdotV_t = tracking_data->GetJdotTimesV();
        // Note that the following cost also includes the constant term, so that
        // the user can differentiate which error norm is bigger. The constant
        // term was not added to the QP since it doesn't change the result.
        cout << "Tracking cost (" << tracking_data->GetName() << ") = "
             << 0.5 * (J_t * dv_sol + JdotV_t - ddy_t).transpose() * W *
                    (J_t * dv_sol + JdotV_t - ddy_t)
             << endl;
      }
    }

    // Target acceleration
    cout << "**********************\n";
    for (auto tracking_data : *tracking_data_vec_) {
      if (tracking_data->GetTrackOrNot()) {
        tracking_data->PrintFeedbackAndDesiredValues(dv_sol);
      }
    }
    cout << "**********************\n\n";
  }

  return u_sol;
}

void OperationalSpaceControlMBP::CalcOptimalInput(
    const drake::systems::Context<double>& context,
    systems::TimestampedVector<double>* control) const {
  // Read in current state and time
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  VectorXd q_w_spr = robot_output->GetPositions();
  //  if (is_quaternion_) {
  //    multibody::SetZeroQuaternionToIdentity(&q_w_spr);
  //  }
  VectorXd v_w_spr = robot_output->GetVelocities();
  VectorXd x_w_spr(plant_w_spr_.num_positions() +
                   plant_w_spr_.num_velocities());
  x_w_spr << q_w_spr, v_w_spr;

  double timestamp = robot_output->get_timestamp();
  auto current_time = static_cast<double>(timestamp);
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

    // Get discrete states
    const auto prev_event_time =
        context.get_discrete_state(prev_event_time_idx_).get_value();

    u_sol = SolveQp(x_w_spr, x_wo_spr, context, current_time, fsm_state(0),
                    current_time - prev_event_time(0));
  } else {
    u_sol = SolveQp(x_w_spr, x_wo_spr, context, current_time, -1, current_time);
  }

  // Assign the control input
  control->SetDataVector(u_sol);
  control->set_timestamp(robot_output->get_timestamp());
}

}  // namespace dairlib::systems::controllers
