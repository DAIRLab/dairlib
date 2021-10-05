//
// Created by brian on 3/8/21.
//

#include "srbd_cmpc.h"
#include "common/file_utils.h"
#include <numeric>

using drake::multibody::JacobianWrtVariable;
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

using dairlib::multibody::SingleRigidBodyPlant;
using dairlib::multibody::makeNameToPositionsMap;
using dairlib::multibody::makeNameToVelocitiesMap;
using dairlib::multibody::SetPositionsAndVelocitiesIfNew;
using dairlib::systems::OutputVector;
using dairlib::LcmTrajectory;

namespace dairlib{

SrbdCMPC::SrbdCMPC(const SingleRigidBodyPlant& plant, double dt,
                   double swing_ft_height, bool traj,
                   bool used_with_finite_state_machine,
                   bool use_com) :
    plant_(plant),
    use_fsm_(used_with_finite_state_machine),
    use_com_(use_com),
    traj_tracking_(traj),
    swing_ft_ht_(swing_ft_height),
    dt_(dt){

  // Create Ports
  state_port_ = this->DeclareVectorInputPort(
      "x, u, t",
      OutputVector<double>(plant_.nq(), plant_.nv(), plant_.nu()))
  .get_index();

  if (!traj_tracking_) {
    x_des_port_ = this->DeclareVectorInputPort("x_des",
        BasicVector<double>(nx_)).get_index();
  }

  traj_out_port_ = this->DeclareAbstractOutputPort("y(t)",
      &SrbdCMPC::GetMostRecentMotionPlan).get_index();

  // Discrete update
  DeclarePerStepDiscreteUpdateEvent(&SrbdCMPC::DiscreteVariableUpdate);
  DeclarePeriodicDiscreteUpdateEvent(dt_, 0, &SrbdCMPC::PeriodicUpdate);

  if ( use_fsm_ ) {
    fsm_port_ = this->DeclareVectorInputPort(
        "fsm",
        BasicVector<double>(1))
    .get_index();

    current_fsm_state_idx_ =
        this->DeclareDiscreteState(VectorXd::Zero(1));
    prev_event_time_idx_ = this->DeclareDiscreteState(-0.1 * VectorXd::Ones(1));
  }
  x_des_ = VectorXd ::Zero(nx_);
}

void SrbdCMPC::AddMode(
    const LinearSrbdDynamics&  dynamics, BipedStance stance, int N) {

  SrbdMode mode;
  mode.stance = stance;
  mode.dynamics = dynamics;
  mode.N = N;
  total_knots_ += N;

  for ( int i = 0; i < N+1; i++) {
    mode.xx.push_back(
        prog_.NewContinuousVariables(nx_, "x_" + std::to_string(i)));
  }
  for (int i = 0; i < N; i++) {
    mode.uu.push_back(
        prog_.NewContinuousVariables(nu_, "u_" + std::to_string(i)));
  }

  mode.pp = prog_.NewContinuousVariables(kLinearDim_, "stance_pos");
  modes_.push_back(mode);
  nmodes_ ++;
}


void SrbdCMPC::SetReachabilityBoundingBox(
    const Vector3d& bounds,
    const std::vector<Eigen::VectorXd> &nominal_relative_pos,
    const MatrixXd& kin_reach_soft_contraint_w) {
  kin_bounds_ = bounds;
  W_kin_reach_ = kin_reach_soft_contraint_w;
  for (const auto& pos : nominal_relative_pos) {
    nominal_foot_pos_.push_back(pos);
  }
}

void SrbdCMPC::CheckProblemDefinition() {
  // Check that at least one mode is defined
  DRAKE_DEMAND(!modes_.empty());
  DRAKE_DEMAND(!nominal_foot_pos_.empty());
  // Check plant definition
  DRAKE_DEMAND(plant_.mass() > 0);
  DRAKE_DEMAND(mu_ > 0 );
  // Check problem matrix sizes
  DRAKE_DEMAND(x_des_.rows() == nx_);
  CheckSquareMatrixDimensions(Q_, nx_);
  CheckSquareMatrixDimensions(W_kin_reach_, kLinearDim_);
  CheckSquareMatrixDimensions(R_, nu_);

}

void SrbdCMPC::Build() {
  CheckProblemDefinition();
  MakeDynamicsConstraints();
  MakeFrictionConeConstraints();
  MakeKinematicReachabilityConstraints();
  MakeStateKnotConstraints();
  MakeInitialStateConstraints();
  MakeTerrainConstraints();
  MakeTrackingCost();

  std::cout << "Built SRBD MPC QP - Modes: "
            << std::to_string(nmodes_) << ", Total Knots: "
            << std::to_string(total_knots_) << std::endl;

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
}

void SrbdCMPC::MakeTerrainConstraints(
    const Vector3d& normal, const Vector3d& origin) {
  /// TODO: @Brian-Acosta add update function to adapt to terrain
  MatrixXd TerrainNormal = normal.transpose();
  VectorXd TerrainPoint = normal.dot(origin) * VectorXd::Ones(1);
  for (auto& mode : modes_) {
    mode.terrain_constraint =
        prog_.AddLinearEqualityConstraint(
            TerrainNormal, TerrainPoint, mode.pp)
        .evaluator().get();
  }
}

void SrbdCMPC::MakeDynamicsConstraints() {
  for (auto& mode : modes_) {
    for (int i = 0; i < mode.N; i++) {
      MatrixXd Aeq = MatrixXd::Zero(2*nx_ + nu_ + kLinearDim_);
      VectorXd beq = VectorXd::Zero(nx_);
      Vector3d pos = Vector3d::Zero();
      CopyDiscreteDynamicsConstraint(mode, false, pos, &Aeq, &beq);
      mode.dynamics_constraints.push_back(
          prog_.AddLinearEqualityConstraint(Aeq, beq,
              {mode.xx.at(i), mode.pp, mode.uu.at(i), mode.xx.at(i+1)})
          .evaluator().get());
    }
  }
}

void SrbdCMPC::MakeInitialStateConstraints() {
  for (auto & mode : modes_) {
    for (int i = 0; i < mode.N; i++) {
      mode.init_state_constraint.push_back(
          prog_.AddLinearEqualityConstraint(
              MatrixXd::Zero(nx_, nx_),
              VectorXd::Zero(nx_),
              mode.xx.at(i))
      .evaluator().get());
    }
  }
}

void SrbdCMPC::MakeKinematicReachabilityConstraints() {
  MatrixXd A = MatrixXd::Zero(kLinearDim_, 2*kLinearDim_);
  A << Matrix3d::Identity() , -Matrix3d::Identity();
  for (auto & mode : modes_) {
    for (int i = 0; i <= mode.N; i++) {
      mode.reachability_constraints.push_back(
          prog_.AddLinearConstraint(
              A,
              nominal_foot_pos_.at(mode.stance) - kin_bounds_,
              nominal_foot_pos_.at(mode.stance) + kin_bounds_,
              {mode.pp, mode.xx.at(i)})
          .evaluator().get());
    }
  }
}

void SrbdCMPC::MakeFrictionConeConstraints() {
  for (auto &mode : modes_) {
    for (int i = 0; i < mode.N + 1; i++) {
      mode.friction_constraints.push_back(
          solvers::CreateLinearFrictionConstraint(mu_).get());
    }
  }
}

void SrbdCMPC::MakeStateKnotConstraints() {
  MatrixXd Aeq = MatrixXd::Identity(nx_, 2 * nx_);
  Aeq.block(0, nx_, nx_, nx_) = -MatrixXd::Identity(nx_, nx_);

  state_knot_constraints_.push_back(
      prog_.AddLinearEqualityConstraint(
              MatrixXd::Zero(Aeq.rows(), Aeq.cols()),
              VectorXd::Zero(nx_),
              {modes_.back().xx.back(), modes_.front().xx.front()})
          .evaluator().get());

  for (int i = 1; i < nmodes_; i++) {
    state_knot_constraints_.push_back(
        prog_.AddLinearEqualityConstraint(
            Aeq,
            VectorXd::Zero(nx_),
            {modes_.at(i-1).xx.back(), modes_.at(i).xx.front()})
        .evaluator().get());
  }
}

void SrbdCMPC::MakeTrackingCost(){
  for (auto & mode : modes_) {
    for (int i = 0; i < mode.N; i++) {
      mode.tracking_cost.push_back(
          prog_.AddQuadraticErrorCost(Q_, x_des_, mode.xx.at(i))
          .evaluator().get());
    }
  }
}

void SrbdCMPC::AddTrackingObjective(const VectorXd &xdes, const MatrixXd &Q) {
  Q_ = Q;
  x_des_ = xdes;
}

void SrbdCMPC::SetTerminalCost(const MatrixXd& Qf) {
  Qf_ = Qf;
}

void SrbdCMPC::AddInputRegularization(const Eigen::MatrixXd &R) {
  R_ = R;
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
  VectorXd x(plant_.nq() + plant_.nv());
  x << q, v;

  double timestamp = robot_output->get_timestamp();

  double time_since_last_event = timestamp;

  if (use_fsm_) {
    int fsm_state =
        (int) (discrete_state->get_vector(current_fsm_state_idx_)
            .get_value()(0) + 1e-6);
    double last_event_time =
        discrete_state->get_vector(prev_event_time_idx_).get_value()(0);

    time_since_last_event = (last_event_time <= 0) ?
        timestamp : timestamp - last_event_time;

    UpdateInitialStateConstraint(plant_.CalcSRBStateFromPlantState(x),
                                 fsm_state, time_since_last_event);
  } else {
    UpdateInitialStateConstraint(
        plant_.CalcSRBStateFromPlantState(x), 0, 0);
  }

  solver_.Solve(prog_, {}, {}, &result_);

  /* Debugging - (not that helpful) */
  if (result_.get_solution_result() == drake::solvers::kInfeasibleConstraints) {
    std::cout << "Infeasible problem! See infeasible constraints below:\n";
    for (auto & name : result_.GetInfeasibleConstraintNames(prog_)) {
      std::cout << name << std::endl;
    }
  }

  most_recent_sol_ = MakeLcmTrajFromSol(
      result_, timestamp, time_since_last_event,  x);

  return EventStatus::Succeeded();
}

void SrbdCMPC::UpdateTrackingObjective(const VectorXd& xdes) const {
  x_des_ = xdes;
  for (auto & mode : modes_) {
    for (auto & cost : mode.tracking_cost) {
      cost->UpdateCoefficients(2.0 * Q_, - 2.0 * Q_ * xdes);
    }
  }
}

void SrbdCMPC::UpdateInitialStateConstraint(
    const VectorXd& x0, const int fsm_state,
    const double t_since_last_switch) const {

  if (!use_fsm_) {
    modes_.front().init_state_constraint.front()->
    UpdateCoefficients(MatrixXd::Identity(nx_, nx_), x0);
    return;
  }

  // Remove current final cost
  auto xf_idx = GetTerminalStepIdx();
  modes_.at(xf_idx.first).terminal_cost.at(xf_idx.second)->UpdateCoefficients(
          MatrixXd::Zero(Qf_.rows(),Qf_.cols()),
          VectorXd::Zero(Qf_.rows()));

  // Remove current initial state constraint
  modes_.at(x0_mode_idx_).init_state_constraint.at(x0_knot_idx_)->
      UpdateCoefficients(MatrixXd::Zero(nx_, nx_),VectorXd::Zero(nx_));


  // Re-create current knot constraint if necessary.
  // Otherwise re-create dynamics constraint
  if (x0_knot_idx_ == 0) {
    MatrixXd Aeq = MatrixXd::Identity(nx_, 2 * nx_);
    Aeq.block(0, nx_, nx_, nx_) = -MatrixXd::Identity(nx_, nx_);
    state_knot_constraints_.at(x0_mode_idx_)->UpdateCoefficients(Aeq, VectorXd::Zero(nx_));
  } else {
    MatrixXd
  }

  // Update the initial state index based on the timestamp
  x0_mode_idx_ = fsm_state;
  x0_knot_idx_ = (int)(t_since_last_switch / dt_) % modes_.at(x0_mode_idx_).N;

  // Add new initial state constraint
  modes_.at(x0_mode_idx_).init_state_constraint.at(x0_knot_idx_)->
      UpdateCoefficients(MatrixXd::Identity(nx_, nx_), x0);

  // remove one constraint to break circular dependency
  if (x0_knot_idx_ == 0) {
    state_knot_constraints_.at(x0_mode_idx_)->UpdateCoefficients(
        MatrixXd::Zero(nx_, 2 * nx_), VectorXd::Zero(nx_));
  } else {
    modes_.at(x0_mode_idx_).dynamics_constraints.at(x0_knot_idx_-1)->
        UpdateCoefficients(MatrixXd::Zero(nx_, 2*nx_ + 2*nu_), VectorXd::Zero(nx_));
  }

  // Add terminal cost to new x_f
  xf_idx = GetTerminalStepIdx();
  modes_.at(xf_idx.first).terminal_cost.at(xf_idx.second)->
      UpdateCoefficients(2 * Qf_, -2 * Qf_ * x_des_, x_des_.transpose() * x_des_);
}

lcmt_saved_traj SrbdCMPC::MakeLcmTrajFromSol(
    const drake::solvers::MathematicalProgramResult& result,
    double time, double time_since_last_touchdown,
    const VectorXd& state) const {

  DRAKE_DEMAND(result.is_success());

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


  /** Allocate Eigen matrices for trajectory blocks **/
  int n_knot = (x0_knot_idx_ == 0) ? total_knots_ + 1 : total_knots_;
  MatrixXd com = MatrixXd::Zero(2*kLinearDim_ , n_knot);
  MatrixXd orientation = MatrixXd::Zero(2*kAngularDim_ , n_knot);
  VectorXd time_knots = VectorXd::Zero(n_knot);

  auto map = MapDecisionVariablesToKnots();
  for (int i = 0; i < map.size(); i++) {
    auto idx = map.at(i);
    VectorXd xi = result.GetSolution(
        modes_.at(idx.first).xx.at(idx.second));
    com.block(0, i, 2*kLinearDim_, 1) << xi.head(kLinearDim_),
        xi.segment(kLinearDim_ + kAngularDim_, kLinearDim_);
    orientation.block(0, i, 2*kAngularDim_, 1) <<
        xi.segment(kLinearDim_, kAngularDim_), xi.tail(kAngularDim_);
    time_knots(i) = time + dt_ * i;
  }

  CoMTraj.time_vector = time_knots;
  CoMTraj.datapoints = com;
  AngularTraj.time_vector = time_knots;
  AngularTraj.datapoints = orientation;

  double next_touchdown_time = time +
      dt_ * (modes_.front().N + 1 - x0_knot_idx_);

  Vector3d swing_ft_traj_breaks(
      time, 0.5*(time + next_touchdown_time), next_touchdown_time);
  SwingFootTraj.time_vector = swing_ft_traj_breaks;
  SwingFootTraj.datapoints = CalcSwingFootKnotPoints(
      state, result, time_since_last_touchdown);;

  LcmTrajectory lcm_traj;
  lcm_traj.AddTrajectory(CoMTraj.traj_name, CoMTraj);
  lcm_traj.AddTrajectory(AngularTraj.traj_name, AngularTraj);
  lcm_traj.AddTrajectory(SwingFootTraj.traj_name, SwingFootTraj);

  return lcm_traj.GenerateLcmObject();
}

MatrixXd SrbdCMPC::CalcSwingFootKnotPoints(const VectorXd& x,
    const MathematicalProgramResult& result,
    double time_since_last_touchdown) const {

}

void SrbdCMPC::print_constraint(
    const std::vector<drake::solvers::LinearConstraint*>& constraints) const {
  for (auto &x0_const : constraints) {
    std::cout << x0_const->get_description() << ":\n A:\n"
              << x0_const->A() << "\nub:\n"
              << x0_const->upper_bound() << "\nlb\n"
              << x0_const->lower_bound() << std::endl;
  }
}

void SrbdCMPC::print_constraint(
    const std::vector<drake::solvers::LinearEqualityConstraint*>& constraints) const {
  for (auto &x0_const : constraints) {
    std::cout << x0_const->get_description() << ":\n A:\n"
              << x0_const->A() << "\nb:\n"
              << x0_const->upper_bound() << std::endl;
  }
}

void SrbdCMPC::print_initial_state_constraints() const {
  for (auto& mode : modes_) {
    print_constraint(mode.init_state_constraint);
  }
}

void SrbdCMPC::print_dynamics_constraints() const{
  for (auto& mode : modes_) {
    print_constraint(mode.dynamics_constraints);
  }
}

void SrbdCMPC::print_state_knot_constraints()const {
  print_constraint(state_knot_constraints_);
}

void SrbdCMPC::print_current_init_state_constraint() const {
  std::cout << "x0 index:" << std::to_string(x0_mode_idx_) << " : " <<
  std::to_string(x0_knot_idx_) << std::endl;
  auto constraint = modes_.at(x0_mode_idx_)
      .init_state_constraint.at(x0_knot_idx_);
  std::cout << "A:\n" << constraint->A()           <<
            "\nlb:\n" << constraint->lower_bound() <<
            "\nub:\n" << constraint->upper_bound() << std::endl;
}

std::pair<int,int> SrbdCMPC::GetTerminalStepIdx() const {
  if (x0_knot_idx_ != 0) {
    return std::pair<int,int>(x0_mode_idx_, x0_knot_idx_ - 1);
  }
  if (x0_mode_idx_ == 0) {
    return std::pair<int,int>(nmodes_ - 1, modes_.back().N);
  }
  return std::pair<int,int>(x0_mode_idx_-1, modes_.at(x0_mode_idx_-1).N);
}

std::vector<std::pair<int, int>> SrbdCMPC::MapDecisionVariablesToKnots() const {
  std::vector<std::pair<int, int>> map;
  int j = x0_knot_idx_;
  for (int i = x0_mode_idx_; i < nmodes_; i++) {
    while (j < modes_.at(i).N) {
      map.push_back({i, j});
    }
    j = 0;
  }
  for (int i = 0; i <= x0_mode_idx_; i++) {
    int lim = (i == x0_mode_idx_) ? x0_knot_idx_ : modes_.at(i).N;
    for (j = 0; j < lim; j++) {
      map.push_back({i, j});
    }
  }
  return map;
}

void SrbdCMPC::CopyDiscreteDynamicsConstraint(
    const SrbdMode& mode, bool current_stance,
    const Vector3d& foot_pos,
    const drake::EigenPtr<MatrixXd> &A,
    const drake::EigenPtr<VectorXd> &b) const {
  DRAKE_DEMAND(A != nullptr && b != nullptr);

  if (!current_stance) {
    A->block(0, 0, nx_, nx_ + kLinearDim_) = mode.dynamics.A;
    *b = -mode.dynamics.b;
  } else {
    A->block(0, 0, nx_, nx_) = mode.dynamics.A.block(0, 0, nx_, nx_);
    A->block(0, nx_, nx_, kLinearDim_) = MatrixXd::Zero(nx_, kLinearDim_);
    *b = mode.dynamics.A.block(0, nx_, nx_, kLinearDim_)
             * foot_pos - mode.dynamics.b;
  }
  A->block(0, nx_ + kLinearDim_, nx_, nu_) = mode.dynamics.B;
  A->block(0, nx_ + nu_ + kLinearDim_, nx_, nx_) =
      -MatrixXd::Identity(nx_, nx_);
}

MatrixXd SrbdCMPC::MakeCollocationConstraintAMatrix(const MatrixXd& A,
                                                    const MatrixXd& B) {
  DRAKE_DEMAND(A.rows() == A.cols());
  DRAKE_DEMAND(A.cols() == nx_);
  DRAKE_DEMAND(B.rows() == A.rows());
  DRAKE_DEMAND(B.cols() == nu_);

  // useful constants
  int n = A.cols();
  int m = B.cols();
  double h = dt_;
  MatrixXd I = MatrixXd::Identity(n, n);

  MatrixXd A_col = MatrixXd::Zero(n, 2 * n + 2 * m);
  A_col.block(0, 0, n, n) = (-3.0 / (2.0 * h)) * I - A * ((h / 8.0) * A + (3.0 / 4.0) * I);
  A_col.block(0, n, n, n) = (3.0 / (2.0 * h)) * I - A * ((-h / 8.0) * A + (3.0 / 4.0) * I);
  A_col.block(0, 2*n, n, m) = (-h/8.0) * A * B - (3.0/4.0) * B;
  A_col.block(0, 2*n +m, n, m) = (h/8.0) * A * B - (3.0/4.0) * B;
  return A_col;
}

MatrixXd SrbdCMPC::MakeSimpsonIntegratedTrackingAndInputCost(
    const Eigen::MatrixXd &A, const Eigen::MatrixXd &B) const {

  //useful constants
  int n = nx_ + np_;
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
  VectorXd yref(2*nx_ + 2* nu_);
  VectorXd ustar = VectorXd::Zero(nu_);
  ustar(nu_-2) = 9.81 * plant_.mass();
  yref << x_des_, x_des_, ustar, ustar;
  return yref;
}

void SrbdCMPC::CheckSquareMatrixDimensions(
    const MatrixXd& M, const int n) const {
  DRAKE_DEMAND(M.rows() == n);
  DRAKE_DEMAND(M.cols() == n);
}

} // dairlib