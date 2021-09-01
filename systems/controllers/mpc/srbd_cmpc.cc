//
// Created by brian on 3/8/21.
//

#include "srbd_cmpc.h"
#include "common/file_utils.h"

using drake::multibody::JacobianWrtVariable;
using drake::multibody::MultibodyPlant;
using drake::multibody::BodyFrame;

using drake::systems::Context;
using drake::systems::BasicVector;
using drake::systems::EventStatus;
using drake::trajectories::PiecewisePolynomial;
using drake::AbstractValue;

using drake::EigenPtr;

using drake::solvers::OsqpSolver;
using drake::solvers::Solve;
using drake::solvers::OsqpSolverDetails;
using drake::solvers::MathematicalProgram;
using drake::solvers::MathematicalProgramResult;

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix3d;
using Eigen::Quaterniond;

using dairlib::multibody::WorldPointEvaluator;
using dairlib::multibody::makeNameToPositionsMap;
using dairlib::multibody::makeNameToVelocitiesMap;
using dairlib::multibody::SetPositionsAndVelocitiesIfNew;
using dairlib::systems::OutputVector;
using dairlib::LcmTrajectory;

namespace dairlib{

Matrix3d HatOperator3x3(const Vector3d& v){
  Eigen::Matrix3d v_hat = Eigen::Matrix3d::Zero();
  v_hat << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
  return v_hat;
}

SrbdCMPC::SrbdCMPC(const MultibodyPlant<double>& plant,
                   Context<double> *plant_context, double dt,
                   double swing_ft_height, bool planar, bool traj,
                   bool used_with_finite_state_machine,
                   bool use_com) :
    plant_(plant),
    plant_context_(plant_context),
    world_frame_(plant_.world_frame()),
    dt_(dt),
    swing_ft_ht_(swing_ft_height),
    planar_(planar),
    traj_tracking_(traj),
    use_com_(use_com),
    use_fsm_(used_with_finite_state_machine),
    rpy_(drake::math::RollPitchYaw<double>(0, 0, 0)){

  nq_ = plant.num_positions();
  nv_ = plant.num_velocities();
  nu_p_ = plant.num_actuators();

  nx_ = planar ? kNxPlanar : kNx3d;
  nu_ = planar ? kNuPlanar : kNu3d;
  kLinearDim_ = planar ? 2 : 3;
  kAngularDim_ = planar ? 1 : 3;

  nxi_ = nx_ + 2*kLinearDim_;
  Q_ = MatrixXd::Zero(nxi_, nxi_);

  // Create Ports
  state_port_ = this->DeclareVectorInputPort(
      OutputVector<double>(nq_, nv_, nu_p_)).get_index();

  if (!traj_tracking_) {
    x_des_port_ = this->DeclareVectorInputPort(
        BasicVector<double>(nxi_)).get_index();
  }

  traj_out_port_ = this->DeclareAbstractOutputPort(&SrbdCMPC::GetMostRecentMotionPlan).get_index();

  // Discrete update
  DeclarePerStepDiscreteUpdateEvent(&SrbdCMPC::DiscreteVariableUpdate);
  DeclarePeriodicDiscreteUpdateEvent(dt_, 0, &SrbdCMPC::PeriodicUpdate);

  if ( use_fsm_ ) {
    fsm_port_ = this->DeclareVectorInputPort(
        BasicVector<double>(1)).get_index();

    current_fsm_state_idx_ =
        this->DeclareDiscreteState(VectorXd::Zero(1));
    prev_event_time_idx_ = this->DeclareDiscreteState(-0.1 * VectorXd::Ones(1));
  }
  x_des_ = VectorXd ::Zero(nxi_);
}

void SrbdCMPC::AddMode(const SrbdDynamics&  dynamics, BipedStance stance, int N) {

  SrbdMode mode;
  mode.stance = stance;
  mode.dynamics = dynamics;
  mode.N = N;

  mode.A_collocation = MakeCollocationConstraintAMatrix(dynamics.A, dynamics.B);
  mode.b_collocation = (3.0/2.0) * dynamics.b;

  mode_knot_counts_.push_back(mode.N);
  total_knots_ += N;

  for ( int i = 0; i < N+1; i++) {
    mode.xx.push_back(prog_.NewContinuousVariables(nxi_, "x_" + std::to_string(i)));
    mode.uu.push_back(prog_.NewContinuousVariables(nu_, "u_" + std::to_string(i)));
    mode.kin_slack.push_back(
        prog_.NewContinuousVariables(kLinearDim_, "kin_slack" + std::to_string(i)));
  }
  modes_.push_back(mode);
  nmodes_++;
}

void SrbdCMPC::AddContactPoint(std::pair<const drake::multibody::BodyFrame<double>&,
                                         Eigen::Vector3d> pt, BipedStance stance) {
  DRAKE_ASSERT(contact_points_.size() == stance);
  contact_points_.push_back(pt);
}

void SrbdCMPC::AddJointToTrackBaseAngle(const std::string& joint_pos_name,
                                        const std::string& joint_vel_name) {
  DRAKE_ASSERT(planar_);
  base_angle_pos_idx_ = makeNameToPositionsMap(plant_)[joint_pos_name];
  base_angle_vel_idx_ = makeNameToVelocitiesMap(plant_)[joint_vel_name];
}

void SrbdCMPC::AddBaseFrame(const std::string &body_name,
                            const Eigen::Vector3d& offset, const Eigen::Isometry3d& frame_pose) {
  DRAKE_ASSERT(!(planar_ && use_com_));
  base_ = body_name;
  frame_pose_ = frame_pose;
  com_from_base_origin_ = offset;
}

double SrbdCMPC::SetMassFromListOfBodies(const std::vector<std::string>& bodies) {
  double mass = CalcCentroidalMassFromListOfBodies(bodies);
  mass_ = mass;
  return mass;
}

void SrbdCMPC::SetReachabilityLimit(const Vector3d& kinematic_limit,
                                    const std::vector<Eigen::VectorXd> &nominal_relative_pos,
                                    const MatrixXd& kin_reach_soft_contraint_w) {
  DRAKE_DEMAND(kinematic_limit.size() == kLinearDim_);
  DRAKE_DEMAND(kin_reach_soft_contraint_w.rows() == kLinearDim_);
  DRAKE_DEMAND(kin_reach_soft_contraint_w.cols() == kLinearDim_);

  kin_lim_ = kinematic_limit;
  W_kin_reach_ = kin_reach_soft_contraint_w;
  for (auto pos : nominal_relative_pos) {
    kin_nominal_.push_back(pos);
  }
}

void SrbdCMPC::CheckProblemDefinition() {
  DRAKE_DEMAND(!modes_.empty());
  DRAKE_DEMAND(mu_ > 0 );
  DRAKE_DEMAND(!contact_points_.empty());
  DRAKE_DEMAND(!kin_nominal_.empty());
  DRAKE_DEMAND(mass_ > 0);
  DRAKE_DEMAND(x_des_.rows() == nxi_);
  DRAKE_DEMAND(Q_.rows() == nxi_);
  DRAKE_DEMAND(R_.rows() == nu_);
}

void SrbdCMPC::Build() {
  CheckProblemDefinition();
  MakeStanceFootConstraints();
  MakeDynamicsConstraints();
  MakeFrictionConeConstraints();
  MakeKinematicReachabilityConstraints();
  MakeStateKnotConstraints();
  MakeInitialStateConstraints();
  MakeTrackingCost();

  std::cout << "Built Koopman Mpc QP: \nModes: " << std::to_string(nmodes_) <<
            "\nTotal Knots: " << std::to_string(total_knots_) << std::endl;

  drake::solvers::SolverOptions solver_options;
  solver_options.SetOption(OsqpSolver::id(), "verbose", 0);
  solver_options.SetOption(OsqpSolver::id(), "eps_abs", 1e-7);
  solver_options.SetOption(OsqpSolver::id(), "eps_rel", 1e-7);
  solver_options.SetOption(OsqpSolver::id(), "eps_prim_inf", 1e-5);
  solver_options.SetOption(OsqpSolver::id(), "eps_dual_inf", 1e-5);
  solver_options.SetOption(OsqpSolver::id(), "polish", 1);
  solver_options.SetOption(OsqpSolver::id(), "scaled_termination", 1);
  solver_options.SetOption(OsqpSolver::id(), "adaptive_rho_fraction", 1);
  prog_.SetSolverOptions(solver_options);
  std::cout << solver_options << std::endl;

}

void SrbdCMPC::MakeStanceFootConstraints() {
  MatrixXd S = MatrixXd::Identity(kLinearDim_, kLinearDim_);
  for (auto & mode : modes_) {
    // Loop over N inputs
    for (int i = 0; i <= mode.N; i++) {
      mode.stance_foot_constraints.push_back(
          prog_.AddLinearEqualityConstraint(
                  S, VectorXd::Zero(kLinearDim_),
                  mode.uu.at(i).segment(mode.stance * kLinearDim_, kLinearDim_ ))
              .evaluator()
              .get());
      mode.stance_foot_constraints.at(i)->set_description(
          "stance_ft"  + std::to_string(mode.stance) + "." + std::to_string(i));
    }
  }
}

void SrbdCMPC::MakeDynamicsConstraints() {
  for (auto & mode : modes_) {
    for (int i = 0; i < mode.N; i++) {
      mode.dynamics_constraints.push_back(
      prog_.AddLinearEqualityConstraint( mode.A_collocation, mode.b_collocation,
              {mode.xx.at(i), mode.xx.at(i+1), mode.uu.at(i), mode.uu.at(i+1)})
          .evaluator()
          .get());

      mode.dynamics_constraints.at(i)->set_description(
          "dyn" + std::to_string(mode.stance) + "." + std::to_string(i));
    }
  }
}

void SrbdCMPC::MakeInitialStateConstraints() {
  for (auto & mode : modes_) {
    for (int i = 0; i < mode.N; i++) {
      mode.init_state_constraint_.push_back(
          prog_.AddLinearEqualityConstraint(
                  MatrixXd::Zero(nxi_, nxi_), VectorXd::Zero(nxi_),
                  mode.xx.at(i))
              .evaluator()
              .get());
      mode.init_state_constraint_.at(i)->set_description(
          "x0" + std::to_string(mode.stance) + "." + std::to_string(i));
    }
  }
}

void SrbdCMPC::MakeKinematicReachabilityConstraints() {
  MatrixXd S = MatrixXd::Zero(kLinearDim_, kLinearDim_ * 3);

  S.block(0, 0, kLinearDim_, kLinearDim_) =
      MatrixXd::Identity(kLinearDim_, kLinearDim_);
  S.block(0, kLinearDim_, kLinearDim_, kLinearDim_) =
      -MatrixXd::Identity(kLinearDim_, kLinearDim_);
  S.block(0, 2*kLinearDim_, kLinearDim_, kLinearDim_) =
      MatrixXd::Identity(kLinearDim_, kLinearDim_);


  std::cout << "Kinematic Upper Limit:\n" << kin_lim_ + kin_nominal_.at(0) << std::endl;
  std::cout << "Kinematic Lower Limit:\n" << kin_nominal_.at(0) - kin_lim_ << std::endl;
  for (auto & mode : modes_) {
    // Loop over N+1 states
    for (int i = 0; i <= mode.N; i++) {
      mode.reachability_constraints.push_back(
          prog_.AddLinearConstraint(S,
                                    -kin_lim_ + kin_nominal_.at(mode.stance),
                                    kin_lim_ + kin_nominal_.at(mode.stance),
                                    {mode.xx.at(i).head(kLinearDim_),
                                     mode.xx.at(i).segment(nx_ + kLinearDim_ * mode.stance,
                                                           kLinearDim_), mode.kin_slack.at(i)})
              .evaluator()
              .get());

      mode.reachability_constraints.at(i)->set_description(
          "Reachability" + std::to_string(mode.stance) + "." + std::to_string(i));

      mode.kin_reach_slack_cost.push_back(
          prog_.AddQuadraticCost(W_kin_reach_, VectorXd::Zero(kLinearDim_), mode.kin_slack.at(i))
              .evaluator()
              .get());
    }
  }
}

void SrbdCMPC::MakeFrictionConeConstraints() {
  for (auto & mode : modes_) {
    for (int i = 0; i < mode.N+1; i++) {
      if (! planar_) {
        mode.friction_constraints.push_back(
            prog_.AddConstraint(
                    solvers::CreateLinearFrictionConstraint(mu_),
                    mode.uu.at(i).segment(nu_ - (kLinearDim_+1), kLinearDim_))
                .evaluator().get());
      } else {
        MatrixXd cone(kLinearDim_, kLinearDim_);
        cone << -1, mu_, 1, mu_;
        mode.friction_constraints.push_back(
            prog_.AddLinearConstraint(
                    cone, VectorXd::Zero(kLinearDim_),
                    VectorXd::Constant(kLinearDim_, std::numeric_limits<double>::infinity()),
                    mode.uu.at(i).segment(nu_ - kLinearDim_, kLinearDim_))
                .evaluator()
                .get());
      }

      mode.friction_constraints.at(i)->set_description(
          "Friction" + std::to_string(mode.stance) + "." + std::to_string(i));
    }
  }
}

void SrbdCMPC::MakeFlatGroundConstraints(const MatrixXd& W) {
  DRAKE_DEMAND(W.cols() == 1);
  DRAKE_DEMAND(W.rows() == 1);

  for (auto & mode : modes_) {
    for (int i = 0; i < mode.N + 1; i++) {
      mode.flat_ground_soft_constraints.push_back(
          prog_.AddQuadraticCost(
                  W, VectorXd::Zero(1),
                  mode.xx.at(i).segment(nx_ + kLinearDim_ * (mode.stance + 1)-1, 1))
              .evaluator()
              .get());
      mode.flat_ground_soft_constraints.at(i)->set_description(
          "FlatGroundConst" + std::to_string(mode.stance) + "." + std::to_string(i));
    }
  }
}

void SrbdCMPC::MakeStateKnotConstraints() {
  // Dummy constraint to be used when cycling modes
  state_knot_constraints_.push_back(
      prog_.AddLinearEqualityConstraint(
              MatrixXd::Zero(1, 2*nxi_),
              VectorXd::Zero(1),
              {modes_.back().xx.back(), modes_.front().xx.front()})
          .evaluator()
          .get());

  MatrixXd Aeq = MatrixXd::Identity(nxi_, 2 * nxi_);
  Aeq.block(0, nxi_, nxi_, nxi_) = -MatrixXd::Identity(nxi_, nxi_);
  for (int i = 0; i < modes_.size() - 1; i++) {
    state_knot_constraints_.push_back(
        prog_.AddLinearEqualityConstraint( Aeq, VectorXd::Zero(nxi_),
                                           {modes_.at(i).xx.back(), modes_.at(i+1).xx.front()})
            .evaluator()
            .get());
    state_knot_constraints_.at(i)->set_description(
        "KnotConstraint" + std::to_string(i));
  }
}

void SrbdCMPC::MakeTrackingCost(){
  for (auto & mode : modes_) {
    mode.Q_total_col_cost = MakeSimpsonIntegratedTrackingAndInputCost(mode.dynamics.A, mode.dynamics.B);
    mode.y_col_cost = MakeSplineSegmentReferenceStateAndInput();
    for (int i = 0; i < mode.N; i++) {
      mode.tracking_cost.push_back(
          prog_.AddQuadraticCost(2.0 * mode.Q_total_col_cost,
                                 -2.0 * mode.Q_total_col_cost * mode.y_col_cost,
                                 {mode.xx.at(i), mode.xx.at(i + 1), mode.uu.at(i), mode.uu.at(i + 1)})
              .evaluator()
              .get());
    }
  }
}

void SrbdCMPC::AddTrackingObjective(const VectorXd &xdes, const MatrixXd &Q) {
  DRAKE_DEMAND(Q.rows() == Q.cols());
  DRAKE_DEMAND(Q.rows() == nxi_);
  DRAKE_DEMAND(xdes.rows() == nxi_);

  Q_ = Q;
  x_des_ = xdes;
}

void SrbdCMPC::SetTerminalCost(const MatrixXd& Qf) {
  DRAKE_DEMAND(Qf.rows() == Qf.cols());
  DRAKE_DEMAND(Qf.rows() == nxi_);

  Qf_ = Qf;
  for (auto & mode : modes_) {
    for (int i = 0; i <= mode.N; i++) {
      mode.terminal_cost.push_back(
          // terminal cost will be applied to the correct node in the update function - set to zero for now
          prog_.AddQuadraticCost(MatrixXd::Zero(nxi_, nxi_),
                                 VectorXd::Zero(nxi_),mode.xx.at(i).head(nxi_))
              .evaluator().get());
    }
  }
}

void SrbdCMPC::AddInputRegularization(const Eigen::MatrixXd &R) {
  DRAKE_DEMAND(R.cols() == nu_);
  DRAKE_DEMAND(R.rows() == nu_);
  R_ = R;
}

VectorXd SrbdCMPC::CalcCentroidalStateFromPlant(const VectorXd& x,
                                                double t) const {

  Vector3d com_pos;
  Vector3d com_vel;
  MatrixXd J_CoM_v = MatrixXd::Zero(3, nv_);

  Vector3d left_pos;
  Vector3d right_pos;
  VectorXd lambda;

  SetPositionsAndVelocitiesIfNew<double>(plant_, x, plant_context_);

  if (use_com_) {
    com_pos = plant_.CalcCenterOfMassPositionInWorld(*plant_context_);
    plant_.CalcJacobianCenterOfMassTranslationalVelocity(*plant_context_,
                                                         JacobianWrtVariable::kV, world_frame_,
                                                         world_frame_, &J_CoM_v);
  } else {
    plant_.CalcPointsPositions(*plant_context_, plant_.GetBodyByName(base_).body_frame(),
        com_from_base_origin_, world_frame_, &com_pos);
    plant_.CalcJacobianTranslationalVelocity(*plant_context_,
                                             JacobianWrtVariable::kV,
                                             plant_.GetBodyByName(base_).body_frame(),
                                             com_from_base_origin_, world_frame_, world_frame_, &J_CoM_v);
  }

  com_vel = J_CoM_v * x.tail(nv_);

  plant_.CalcPointsPositions(*plant_context_, contact_points_.at(kLeft).first,
                             contact_points_.at(kLeft).second, world_frame_, &left_pos);
  plant_.CalcPointsPositions(*plant_context_, contact_points_.at(kRight).first,
                             contact_points_.at(kRight).second, world_frame_, &right_pos);
  Vector3d base_orientation;
  VectorXd base_omega;

  /*** Adjustment for slope ***/
//  com_pos(vertical_idx()) += (modes_.at(x0_idx_[0]).stance == kLeft) ?
//      left_pos(vertical_idx()) : right_pos(vertical_idx());

  if (planar_) {
    base_orientation = x.head(nq_).segment(base_angle_pos_idx_, kAngularDim_);
    base_omega = x.tail(nv_).segment(base_angle_vel_idx_, kAngularDim_);
  } else {
    auto base_transform = plant_.EvalBodyPoseInWorld(*plant_context_,
                                                     plant_.GetBodyByName(base_));

    rpy_.SetFromRotationMatrix(base_transform.rotation());
    base_orientation << rpy_.vector();
    MatrixXd J_spatial(6, nv_);
    plant_.CalcJacobianSpatialVelocity(*plant_context_, JacobianWrtVariable::kV,
                                       plant_.GetBodyByName(base_).body_frame(), frame_pose_.translation(),
                                       world_frame_, world_frame_, &J_spatial);

    base_omega = J_spatial.block(0, 0, 3, J_spatial.cols()) * x.tail(nv_);
  }


  VectorXd x_srbd(nxi_);

  if (planar_) {
    x_srbd << MakePlanarVectorFrom3d(com_pos), base_orientation,
        MakePlanarVectorFrom3d(com_vel), base_omega, MakePlanarVectorFrom3d(left_pos),
        MakePlanarVectorFrom3d(right_pos); // MakePlanarVectorFrom3d(lambda);
  } else {
    x_srbd << com_pos, base_orientation, com_vel, base_omega,
        left_pos, right_pos;
  }
  return x_srbd;
}

void SrbdCMPC::GetMostRecentMotionPlan(const drake::systems::Context<double> &context,
                                       dairlib::lcmt_saved_traj *traj_msg) const {
  *traj_msg = most_recent_sol_;
}

EventStatus SrbdCMPC::DiscreteVariableUpdate(
    const drake::systems::Context<double> &context,
    drake::systems::DiscreteValues<double> *discrete_state) const {

  const BasicVector<double>* fsm_output =
  (BasicVector<double>*)this->EvalVectorInput(context, fsm_port_);
  VectorXd fsm_state = fsm_output->get_value();

  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  double timestamp = robot_output->get_timestamp();

  VectorXd xdes = this->EvalVectorInput(context, x_des_port_)->get_value();

  if (xdes != x_des_) {
    UpdateTrackingObjective(xdes);
  }

  if (use_fsm_) {
    auto current_fsm_state =
        discrete_state->get_mutable_vector(current_fsm_state_idx_)
            .get_mutable_value();

    if (fsm_state(0) != current_fsm_state(0)) {

      current_fsm_state(0) = fsm_state(0);

      discrete_state->get_mutable_vector(prev_event_time_idx_).get_mutable_value()
          << timestamp;
    }
  }
  return EventStatus::Succeeded();
}

EventStatus SrbdCMPC::PeriodicUpdate(
    const drake::systems::Context<double> &context,
    drake::systems::DiscreteValues<double>* discrete_state) const {

  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);

  VectorXd q = robot_output->GetPositions();
  VectorXd v = robot_output->GetVelocities();
  VectorXd x(nq_+nv_);
  x << q, v;

  double timestamp = robot_output->get_timestamp();

  double time_since_last_event = timestamp;

  if (use_fsm_) {
    int fsm_state = (int) (discrete_state->get_vector(current_fsm_state_idx_).get_value()(0) + 1e-6);
    double last_event_time = discrete_state->get_vector(prev_event_time_idx_).get_value()(0);
    time_since_last_event = (last_event_time <= 0) ? timestamp : timestamp - last_event_time;

    UpdateInitialStateConstraint(CalcCentroidalStateFromPlant(x, timestamp),
                                 fsm_state, time_since_last_event);
  } else {
    UpdateInitialStateConstraint(
        CalcCentroidalStateFromPlant(x, timestamp), 0, 0);
  }

  solver_.Solve(prog_, {}, {}, &result_);

//  if (!result_.is_success()) {
//    std::cout << "Infeasible\n";
//    print_current_init_state_constraint();
//  }

  // DRAKE_DEMAND(result_.is_success());
  /* Debugging - (not that helpful) */
  if (result_.get_solution_result() == drake::solvers::kInfeasibleConstraints) {
    std::cout << "Infeasible problem! See infeasible constraints below:\n";
    for (auto name : result_.GetInfeasibleConstraintNames(prog_)) {
      std::cout << name << std::endl;
    }
  }

  most_recent_sol_ = MakeLcmTrajFromSol(
      result_, timestamp, time_since_last_event,  x);

  //print_current_init_state_constraint();
  //print_initial_state_constraints();
  //print_state_knot_constraints();
  //print_dynamics_constraints();

  return EventStatus::Succeeded();
}

void SrbdCMPC::UpdateTrackingObjective(const VectorXd& xdes) const {
  std::cout << "Updating tracking objective!" << std::endl;
  x_des_ = xdes;
  for (auto & mode : modes_) {
    for (auto & cost : mode.tracking_cost) {
      cost->UpdateCoefficients(2.0 * mode.Q_total_col_cost,
          - 2.0 *  mode.Q_total_col_cost * MakeSplineSegmentReferenceStateAndInput());
    }
  }
}

void SrbdCMPC::UpdateInitialStateConstraint(const VectorXd& x0,
                                            const int fsm_state, const double t_since_last_switch) const {

  if (!use_fsm_) {
    modes_.front().init_state_constraint_.front()->UpdateCoefficients(MatrixXd::Identity(nxi_, nxi_), x0);
    return;
  }

  // Remove current final cost
  auto xf_idx = GetTerminalStepIdx();
  modes_.at(xf_idx.first).terminal_cost.at(xf_idx.second)->
      UpdateCoefficients(
          MatrixXd::Zero(Qf_.rows(), Qf_.cols()), VectorXd::Zero(Qf_.rows(), 0));

  // Remove current initial state constraint
  modes_.at(x0_idx_[0]).init_state_constraint_.at(x0_idx_[1])->
      UpdateCoefficients(MatrixXd::Zero(nxi_, nxi_), VectorXd::Zero(nxi_));


  // Re-create current knot constraint if necessary.
  // Otherwise re-create dynamics constraint
  if (x0_idx_[1] == 0) {
    MatrixXd Aeq = MatrixXd::Identity(nxi_, 2 * nxi_);
    Aeq.block(0, nxi_, nxi_, nxi_) = -MatrixXd::Identity(nxi_, nxi_);
    state_knot_constraints_.at(x0_idx_[0])->UpdateCoefficients(Aeq, VectorXd::Zero(nxi_));
  } else {
    modes_.at(x0_idx_[0]).dynamics_constraints.at(x0_idx_[1]-1)->
        UpdateCoefficients(modes_.at(x0_idx_[0]).A_collocation,
                           modes_.at(x0_idx_[0]).b_collocation);
  }


  /// TODO (@Brian-Acosta) - Add check to be able to start controller after sim
  // Update the initial state index based on the timestamp
  x0_idx_[0] = fsm_state;
  x0_idx_[1] = std::floor(t_since_last_switch / dt_);

  if (x0_idx_[1] == modes_.at(x0_idx_[0]).N) {
    std::cout << "Shouldn't be here!" << std::endl;
    x0_idx_[1] = 0;
    x0_idx_[0] += 1;
    if (x0_idx_[0] == nmodes_) {
      x0_idx_[0] = 0;
    }
  }

  // Add new initial state constraint
  modes_.at(x0_idx_[0]).init_state_constraint_.at(x0_idx_[1])->UpdateCoefficients(MatrixXd::Identity(nxi_, nxi_), x0);

  // remove one constraint to break circular dependency
  if (x0_idx_[1] == 0) {
    state_knot_constraints_.at(x0_idx_[0])->UpdateCoefficients(
        MatrixXd::Zero(nxi_, 2 * nxi_), VectorXd::Zero(nxi_));
  } else {
    modes_.at(x0_idx_[0]).dynamics_constraints.at(x0_idx_[1]-1)->
        UpdateCoefficients(MatrixXd::Zero(nxi_, 2*nxi_ + 2*nu_), VectorXd::Zero(nxi_));
  }

  // Add terminal cost to new x_f
  xf_idx = GetTerminalStepIdx();
  modes_.at(xf_idx.first).terminal_cost.at(xf_idx.second)->
      UpdateCoefficients(2 * Qf_, -2 * Qf_ * x_des_, x_des_.transpose() * x_des_);
}

double SrbdCMPC::CalcCentroidalMassFromListOfBodies(std::vector<std::string> bodies) {
  double mass = 0;
  for (auto & name : bodies) {
    mass += plant_.GetBodyByName(name).get_mass(*plant_context_);
  }
  return mass;
}

/// TODO(@Brian-Acosta) Update trajectory to be pelvis trajectory given offset
lcmt_saved_traj SrbdCMPC::MakeLcmTrajFromSol(const drake::solvers::MathematicalProgramResult& result,
                                             double time, double time_since_last_touchdown,
                                             const VectorXd& state) const {
  DRAKE_ASSERT(result.is_success());

  LcmTrajectory::Trajectory CoMTraj;
  LcmTrajectory::Trajectory AngularTraj;
  LcmTrajectory::Trajectory SwingFootTraj;

  CoMTraj.traj_name = "com_traj";
  AngularTraj.traj_name = "orientation";
  SwingFootTraj.traj_name = "swing_foot_traj";


  /** need to set datatypes for LcmTrajectory to save properly **/
  for (int i = 0; i < 2*kLinearDim_; i++) {
    CoMTraj.datatypes.emplace_back("double");
    SwingFootTraj.datatypes.emplace_back("double");
  }
  for (int i = 0; i < 2*kAngularDim_; i++) {
    AngularTraj.datatypes.emplace_back("double");
  }

  /** preallocate Eigen matrices for trajectory blocks **/
  MatrixXd x = MatrixXd::Zero(nx_ ,  total_knots_);
  VectorXd x_time_knots = VectorXd::Zero(total_knots_);

  int idx_x0 = x0_idx_[0] * modes_.front().N + x0_idx_[1];
  for (int i = 0; i < nmodes_; i++) {
    auto& mode = modes_.at(i);

    for (int j = 0; j < mode.N; j++) {
      int idx = i * mode.N + j;
      int col_i = idx - idx_x0;
      col_i = (col_i < 0) ? (col_i + total_knots_) : col_i;

      x.block(0, col_i, nx_, 1) =
          result.GetSolution(mode.xx.at(j).head(nx_));
      x_time_knots(col_i) = time + dt_ * col_i;
    }
  }

  MatrixXd x_com_knots(2*kLinearDim_, x.cols());
  MatrixXd orientation_knots(2*kAngularDim_, x.cols());
  x_com_knots << x.block(0, 0, kLinearDim_, x.cols()),
                 x.block(kLinearDim_ + kAngularDim_, 0, kLinearDim_, x.cols());
  orientation_knots << x.block(kLinearDim_, 0, kAngularDim_, x.cols()),
                       x.block(2*kLinearDim_ + kAngularDim_, 0, kAngularDim_, x.cols());

  CoMTraj.time_vector = x_time_knots;
  CoMTraj.datapoints = x_com_knots;

  AngularTraj.time_vector = x_time_knots;
  AngularTraj.datapoints = orientation_knots;

  double next_touchdown_time = time +
      dt_ * (modes_.front().N + 1 - x0_idx_[1]);

  Vector3d swing_ft_traj_breaks(time, 0.5*(time + next_touchdown_time), next_touchdown_time);
  SwingFootTraj.time_vector = swing_ft_traj_breaks;
  SwingFootTraj.datapoints = CalcSwingFootKnotPoints(state, result, time_since_last_touchdown);;

  LcmTrajectory lcm_traj;
  lcm_traj.AddTrajectory(CoMTraj.traj_name, CoMTraj);
  lcm_traj.AddTrajectory(AngularTraj.traj_name, AngularTraj);
  lcm_traj.AddTrajectory(SwingFootTraj.traj_name, SwingFootTraj);

  return lcm_traj.GenerateLcmObject();
}

MatrixXd SrbdCMPC::CalcSwingFootKnotPoints(const VectorXd& x,
                                           const MathematicalProgramResult& result, double time_since_last_touchdown) const {

  int next_mode_idx = (x0_idx_[0] == nmodes_ - 1) ? 0 : x0_idx_[0] + 1;
  auto& curr_mode = modes_.at(x0_idx_[0]);
  auto& next_mode = modes_.at(next_mode_idx);

  VectorXd curr_pos = VectorXd::Zero(kLinearDim_), mid_pos = VectorXd::Zero(kLinearDim_),
      end_pos = VectorXd::Zero(kLinearDim_), curr_vel = VectorXd::Zero(kLinearDim_),
      mid_vel = VectorXd::Zero(kLinearDim_);

  Vector3d swing_ft_plant = Vector3d::Zero(), swing_vel_plant = Vector3d::Zero();

  MatrixXd J_swing_vel = MatrixXd::Zero(3, nv_);
  MatrixXd swing_ft_traj = MatrixXd::Zero(2*kLinearDim_, 3);

  /** Calculate current swing foot position and velocity **/
  auto& swing_pt = contact_points_.at(1 - curr_mode.stance);
  SetPositionsAndVelocitiesIfNew<double>(plant_, x, plant_context_);
  plant_.CalcPointsPositions(*plant_context_, swing_pt.first,
                             swing_pt.second, world_frame_, &swing_ft_plant);
  plant_.CalcJacobianTranslationalVelocity(*plant_context_,
                                           JacobianWrtVariable::kV, swing_pt.first,Vector3d::Zero(),
                                           world_frame_, world_frame_, &J_swing_vel);

  swing_vel_plant = J_swing_vel * x.tail(nv_);

  if (planar_) {
    curr_pos = MakePlanarVectorFrom3d(swing_ft_plant);
    curr_vel = MakePlanarVectorFrom3d(swing_vel_plant);
  } else {
    curr_pos = swing_ft_plant;
    curr_vel = swing_vel_plant;
  }

  end_pos = result.GetSolution(next_mode.xx.at(0)
                                   .segment(nx_ + kLinearDim_ * next_mode.stance, kLinearDim_));

  mid_pos = (curr_pos  + end_pos) / 2.0;
  mid_pos (kLinearDim_-1) = swing_ft_ht_ - pow((time_since_last_touchdown - (dt_* curr_mode.N)/2.0), 2);

  swing_ft_traj.block(0, 0, kLinearDim_, 1) = curr_pos;
  swing_ft_traj.block(0, 1, kLinearDim_, 1) = mid_pos;
  swing_ft_traj.block(0,2, kLinearDim_, 1) = end_pos;
  swing_ft_traj.block(kLinearDim_, 0, kLinearDim_, 1) = curr_vel;
  swing_ft_traj.block(kLinearDim_, 1, kLinearDim_, 1) = mid_vel;
  swing_ft_traj.block(kLinearDim_, 2, kLinearDim_, 1) = VectorXd::Zero(kLinearDim_);

  return swing_ft_traj;
}

Vector2d SrbdCMPC::MakePlanarVectorFrom3d(Vector3d vec) const {
  return Vector2d(vec(saggital_idx_), vec(vertical_idx_));
}

void SrbdCMPC::print_initial_state_constraints() const {
  for (auto& mode : modes_) {
    for (auto& x0_const : mode.init_state_constraint_){
      std::cout << x0_const->get_description() <<":\n A:\n" <<  x0_const->A()
                << "\nub:\n" << x0_const->upper_bound() <<
                "\nlb\n" << x0_const->lower_bound() << std::endl;
    }
  }
}

void SrbdCMPC::print_dynamics_constraints() const{
  for (auto& mode : modes_) {
    for (auto& dyn : mode.dynamics_constraints){
      std::cout << dyn->get_description()
                << "\nub:\n" << dyn->upper_bound() <<
                "\nlb\n" << dyn->lower_bound() << std::endl;
    }
  }
}

void SrbdCMPC::print_state_knot_constraints()const {
  for (auto& knot : state_knot_constraints_) {
    std::cout << knot->get_description() <<":\n A:\n" <<  knot->A()
              << "\nub:\n" << knot->upper_bound() <<
              "\nlb\n" << knot->lower_bound() << std::endl;
  }
}

void SrbdCMPC::print_current_init_state_constraint() const {
  std::cout << "x0 index:" <<
            std::to_string(x0_idx_[0]) << " : " <<
            std::to_string(x0_idx_[1]) << std::endl;

  auto constraint = modes_.at(x0_idx_[0]).init_state_constraint_.at(x0_idx_[1]);
  std::cout << "A:\n" << constraint->A() <<
            "\nlb:\n" << constraint->lower_bound() <<
            "\nub:\n"<< constraint->upper_bound() << std::endl;
}

std::pair<int,int> SrbdCMPC::GetTerminalStepIdx() const {
  if (x0_idx_[1] != 0) {
    return std::pair<int,int>(x0_idx_[0], x0_idx_[1] - 1);
  }
  if (x0_idx_[0] == 0) {
    return std::pair<int,int>(nmodes_ - 1, modes_.back().N);
  }
  return std::pair<int,int>(x0_idx_[0]-1, modes_.at(x0_idx_[0]-1).N);
}

void SrbdCMPC::CopyContinuous3dSrbDynamics(double m, double yaw, BipedStance stance,
                                       const Eigen::MatrixXd &b_I,
                                       const Eigen::Vector3d &eq_com_pos,
                                       const Eigen::Vector3d &eq_foot_pos,
                                       const drake::EigenPtr<MatrixXd> &Ad,
                                       const drake::EigenPtr<MatrixXd> &Bd,
                                       const drake::EigenPtr<VectorXd> &bd) {

  const Eigen::Vector3d g = {0.0, 0.0, 9.81};
  drake::math::RollPitchYaw rpy(0.0, 0.0, yaw);
  Matrix3d R_yaw = rpy.ToMatrix3ViaRotationMatrix();
  Vector3d tau = {0.0,0.0,1.0};
  Vector3d mg = {0.0, 0.0, m * 9.81};
  Matrix3d lambda_hat = HatOperator3x3(m * g);
  Matrix3d g_I_inv = (R_yaw * b_I * R_yaw.transpose()).inverse();


  MatrixXd A = MatrixXd::Zero(18,18);
  MatrixXd B = MatrixXd::Zero(18,10);
  VectorXd b = VectorXd::Zero(18);


  // Continuous A matrix
  A.block(0, 6, 3, 3) = Matrix3d::Identity();
  A.block(3, 9, 3, 3) = R_yaw;
  A.block(9, 0, 3, 3) = g_I_inv * lambda_hat;
  if (stance == BipedStance::kLeft) {
    A.block(9, 12, 3, 3) = - g_I_inv * lambda_hat;
  } else if (stance == BipedStance::kRight) {
    A.block(9, 15, 3, 3) = -g_I_inv * lambda_hat;
  }

  // Continuous B matrix
  B.block(6, 6, 3, 3) = (1.0 / m) * Matrix3d::Identity();
  B.block(9, 6, 3, 3) = g_I_inv * HatOperator3x3(R_yaw * (eq_foot_pos - eq_com_pos));
  B.block(9, 9, 3, 1) = g_I_inv * Vector3d(0.0, 0.0, 1.0);
  B.block(12, 0, 6, 6) = MatrixXd::Identity(6, 6);

  // Continuous Affine portion (b)
  b.segment(6, 3) = -g;
  b.segment(9, 3) = g_I_inv * HatOperator3x3(R_yaw * (eq_foot_pos - eq_com_pos)) * (m * g);

  *Ad = A;
  *Bd = B;
  *bd = b;
}

MatrixXd SrbdCMPC::MakeCollocationConstraintAMatrix(const MatrixXd& A,
                                                    const MatrixXd& B) {
  DRAKE_ASSERT(A.rows() == A.cols());
  DRAKE_ASSERT(A.cols() == nxi_);
  DRAKE_ASSERT(B.rows() == A.rows());
  DRAKE_ASSERT(B.cols() == nu_);

  std::cout << "Probe 1\n";
  // useful constants
  int n = A.cols();
  int m = B.cols();
  MatrixXd I = MatrixXd::Identity(n, n);
  double h = dt_;

  MatrixXd A_col = MatrixXd::Zero(n, 2 * n + 2 * m);
  A_col.block(0, 0, n, n) = (-3.0 / (2.0 * h)) * I - A * ((h / 8.0) * A + (3.0 / 4.0) * I);
  A_col.block(0, n, n, n) = (3.0 / (2.0 * h)) * I - A * ((-h / 8.0) * A + (3.0 / 4.0) * I);
  A_col.block(0, 2*n, n, m) = (-h/8.0) * A * B - (3.0/4.0) * B;
  A_col.block(0, 2*n +m, n, m) = (h/8.0) * A * B - (3.0/4.0) * B;
  return A_col;
}

MatrixXd SrbdCMPC::MakeSimpsonIntegratedTrackingAndInputCost(const Eigen::MatrixXd &A,
                                                             const Eigen::MatrixXd &B) const {
  //useful constants
  int n = nxi_;
  int m = nu_;
  double h = dt_;

  MatrixXd In = MatrixXd::Identity(n, n);
  MatrixXd Im = MatrixXd::Identity(m, m);
  MatrixXd P = MatrixXd::Zero(n, 2*n+2*m);
  MatrixXd Z = MatrixXd::Zero(m, 2*m);
  MatrixXd Q_tot = MatrixXd::Zero(2*(n+m), 2*(n+m));

  P.block(0, 0, n, n) = 0.5 * In + (h/8) * A;
  P.block(0, n, n, n) = 0.5 * In -(h/8) * A;
  P.block(0, 2*n, n, m) = (h/8) * B;
  P.block(0, 2*n + m, n, m) = -(h/8) * B;

  Z.block(0, 0, m, m) = 0.5 * Im;
  Z.block(0, m, m, m) = 0.5 * Im;

  Q_tot.block(0, 0, n, n) = Q_;
  Q_tot.block(n, n, n, n) = Q_;
  Q_tot.block(2*n, 2*n, m, m) = R_;
  Q_tot.block(2*n+m, 2*n+m, m, m) = R_;

  Q_tot += 4.0 * P.transpose() * Q_ * P;
  Q_tot.block(2*n, 2*n, 2*m, 2*m) += 4.0 * Z.transpose() * R_ * Z;

  return Q_tot;
}

VectorXd SrbdCMPC::MakeSplineSegmentReferenceStateAndInput() const {
  VectorXd yref(2*nxi_ + 2* nu_);
  VectorXd ustar = VectorXd::Zero(nu_);
  ustar(nu_-2) = 9.81 * mass_;
  yref << x_des_, x_des_, ustar, ustar;
  return yref;
}

} // dairlib