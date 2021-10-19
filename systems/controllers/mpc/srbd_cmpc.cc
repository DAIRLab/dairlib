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

using drake::solvers::Solve;
using drake::solvers::OsqpSolver;
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

void SrbdCMPC::AddMode(const LinearSrbdDynamics&  dynamics,
    BipedStance stance, const MatrixXd& reset, int N) {
  DRAKE_DEMAND(stance == nmodes_);
  SrbdMode mode = {dynamics, reset, stance, N};
  for (int i = 0; i < N; i++) {
    xx.push_back(prog_.NewContinuousVariables(
        nx_, "x_" + std::to_string(i + stance)));
    uu.push_back(prog_.NewContinuousVariables(
        nu_, "u_" + std::to_string(i + stance)));
  }
  pp.push_back(prog_.NewContinuousVariables(
      kLinearDim_, "p_" + std::to_string(stance)));
  total_knots_ += N;
  modes_.push_back(mode);
  nmodes_ ++;
}

void SrbdCMPC::FinalizeModeSequence(){
     xx.push_back(prog_.NewContinuousVariables(nx_, "x_f"));
}

void SrbdCMPC::SetReachabilityBoundingBox(const Vector3d& bounds,
    const std::vector<Eigen::VectorXd> &nominal_relative_pos) {
  kin_bounds_ = bounds;
  for (const auto& pos : nominal_relative_pos) {
    nominal_foot_pos_.push_back(pos);
  }
}

void SrbdCMPC::CheckProblemDefinition() {
  DRAKE_DEMAND(!modes_.empty());
  DRAKE_DEMAND(!nominal_foot_pos_.empty());
  DRAKE_DEMAND(plant_.mass() > 0);
  DRAKE_DEMAND(mu_ > 0 );
  DRAKE_DEMAND(x_des_.rows() == nx_);
  CheckSquareMatrixDimensions(Q_, nx_);
  CheckSquareMatrixDimensions(R_, nu_);
}

void SrbdCMPC::Build() {
  CheckProblemDefinition();
  MakeDynamicsConstraints();
  MakeFrictionConeConstraints();
  MakeKinematicReachabilityConstraints();
  MakeInitialStateConstraint();
  MakeTerrainConstraints();
  MakeCost();

  drake::solvers::SolverOptions solver_options;
  solver_options.SetOption(OsqpSolver::id(), "verbose", 0);
  solver_options.SetOption(OsqpSolver::id(), "eps_abs", 5e-5);
  solver_options.SetOption(OsqpSolver::id(), "eps_rel", 1e-4);
//  solver_options.SetOption(OsqpSolver::id(), "eps_prim_inf", 1e-4);
//  solver_options.SetOption(OsqpSolver::id(), "eps_dual_inf", 1e-4);
  solver_options.SetOption(OsqpSolver::id(), "polish", 1);
//  solver_options.SetOption(OsqpSolver::id(), "scaled_termination", 1);
//  solver_options.SetOption(OsqpSolver::id(), "adaptive_rho_fraction", 1.0);
  solver_options.SetOption(OsqpSolver::id(), "max_iter", 15000);
  prog_.SetSolverOptions(solver_options);
}

void SrbdCMPC::MakeTerrainConstraints(
    const Vector3d& normal, const Vector3d& origin) {
  /// TODO: @Brian-Acosta add update function to adapt to terrain
  MatrixXd TerrainNormal = normal.transpose();
  VectorXd TerrainPoint = normal.dot(origin) * VectorXd::Ones(1);
  for (int i = 0; i < nmodes_; i++) {
    terrain_angle_.push_back(
        prog_.AddLinearEqualityConstraint(
            TerrainNormal, TerrainPoint, pp.at(i))
        .evaluator().get());
  }
}

void SrbdCMPC::MakeDynamicsConstraints() {
  for (int j = 0; j < nmodes_; j++) {
    auto mode = modes_.at(j);
    for (int i = 0; i < mode.N; i++) {
      MatrixXd Aeq = MatrixXd::Zero(nx_, 2*nx_ + nu_ + kLinearDim_);
      VectorXd beq = VectorXd::Zero(nx_);
      Vector3d pos = Vector3d::Zero();
      pos(1) = nominal_foot_pos_.at(j)(1);
      CopyDiscreteDynamicsConstraint(mode, false, pos, &Aeq, &beq);
      dynamics_.push_back(
          prog_.AddLinearEqualityConstraint(Aeq, beq,
              {xx.at(j * mode.N + i),
               pp.at(mode.stance),
               uu.at(j * mode.N + i),
               xx.at(j * mode.N + i + 1)}));
    }
  }
//  for (auto& binding : dynamics_) {
//    std::cout << binding.to_string() << std::endl;
//  }
}

void SrbdCMPC::UpdateKinematicConstraints(
    int n_until_stance, int fsm_state) const {
  for (auto & constraint : kinematic_constraint_){
    prog_.RemoveConstraint(constraint);
  }
  kinematic_constraint_.clear();

  MatrixXd A = MatrixXd::Zero(kLinearDim_, 2*kLinearDim_);
  A << Matrix3d::Identity(), -Matrix3d::Identity();
  auto curr_mode = modes_.at(fsm_state);
  auto next_mode = modes_.at(1-fsm_state);

  for (int i = n_until_stance; i < n_until_stance + next_mode.N; i++) {
    kinematic_constraint_.push_back(
        prog_.AddLinearConstraint(
            A,
            nominal_foot_pos_.at(next_mode.stance) - kin_bounds_,
            nominal_foot_pos_.at(next_mode.stance) + kin_bounds_,
            {pp.at(next_mode.stance), xx.at(i).head(kLinearDim_)}));
  }
  for (int i = n_until_stance + next_mode.N; i <= total_knots_; i++) {
    kinematic_constraint_.push_back(
        prog_.AddLinearConstraint(
            A,
            nominal_foot_pos_.at(curr_mode.stance) - kin_bounds_,
            nominal_foot_pos_.at(curr_mode.stance) + kin_bounds_,
            {pp.at(curr_mode.stance), xx.at(i).head(kLinearDim_)}));
  }
}

void SrbdCMPC::UpdateDynamicsConstraints(const Eigen::VectorXd& x,
    int n_until_next_stance, int fsm_state) const {

  auto& mode = modes_.at(fsm_state);
  Vector3d pos = plant_.CalcFootPosition(x, mode.stance);

//  for (auto& constraint : dynamics_) {
//    std::cout <<  constraint.to_string() << std::endl;
//  }

  if (n_until_next_stance == mode.N) {
    // make current stance dynamics and apply to mode
    MatrixXd Aeq = MatrixXd::Zero(nx_, 2*nx_ + nu_);
    VectorXd beq = VectorXd::Zero(nx_);
    CopyDiscreteDynamicsConstraint(mode, true, pos, &Aeq, &beq);

    for (int i = 0; i < (mode.N-1); i++){
      prog_.RemoveConstraint(dynamics_.at(i));
      dynamics_.at(i) = prog_.AddLinearEqualityConstraint(
          Aeq, beq, {xx.at(i), uu.at(i), xx.at(i+1)});
    }

    Aeq = MatrixXd::Zero(nx_, 2*nx_ + kLinearDim_ + nu_);
    beq = VectorXd::Zero(nx_);
    CopyDiscreteDynamicsConstraint(modes_.at(1-fsm_state),
        false, pos, &Aeq, &beq);
    prog_.RemoveConstraint(dynamics_[dynamics_.size()-1]);
    dynamics_.back() = prog_.AddLinearEqualityConstraint(
        Aeq, beq,
        {xx.at(total_knots_-1), pp.at(1-fsm_state),
         uu.at(total_knots_-1), xx.at(total_knots_)});
  } else {
    int idx = n_until_next_stance;
    MatrixXd Aeq = MatrixXd::Zero(nx_, 2*nx_ + kLinearDim_ + nu_);
    VectorXd beq = VectorXd::Zero(nx_);
    CopyDiscreteDynamicsConstraint(modes_.at(1-fsm_state), false, pos, &Aeq, &beq);
    prog_.RemoveConstraint(dynamics_.at(idx));
    dynamics_.at(idx) = prog_.AddLinearEqualityConstraint(
        Aeq, beq,
        {xx.at(idx), pp.at(1-fsm_state), uu.at(idx), xx.at(idx+1)});
    CopyDiscreteDynamicsConstraint(mode, false, pos, &Aeq, &beq);
    prog_.RemoveConstraint(dynamics_.at(idx + mode.N));
    dynamics_.at(idx+mode.N) = prog_.AddLinearEqualityConstraint(
        Aeq, beq,
        {xx.at(idx+mode.N), pp.at(fsm_state),
         uu.at(idx+mode.N), xx.at(mode.N+idx+1)});
  }
}

void SrbdCMPC::MakeInitialStateConstraint() {
  initial_state_ = prog_.AddLinearEqualityConstraint(
              MatrixXd::Identity(nx_, nx_),
              VectorXd::Zero(nx_),
              xx.at(0)).evaluator().get();
}

void SrbdCMPC::MakeKinematicReachabilityConstraints() {
  MatrixXd A = MatrixXd::Zero(kLinearDim_, 2*kLinearDim_);
  A << Matrix3d::Identity(), -Matrix3d::Identity();
  for (int j = 0; j < nmodes_ ; j ++) {
    auto mode = modes_.at(j);
    for (int i = 0; i <= mode.N; i++) {
      kinematic_constraint_.push_back(
          prog_.AddLinearConstraint(
              A,
              nominal_foot_pos_.at(mode.stance) - kin_bounds_,
              nominal_foot_pos_.at(mode.stance) + kin_bounds_,
              {pp.at(mode.stance), xx.at(i + j * mode.N).head(kLinearDim_)}));
    }
  }
}

void SrbdCMPC::MakeFrictionConeConstraints() {
  for (int i = 0; i < total_knots_; i++) {
      friction_cone_.push_back(
          prog_.AddConstraint(
          solvers::CreateLinearFrictionConstraint(mu_),
          uu.at(i).head(kLinearDim_)).evaluator().get());
  }
}

void SrbdCMPC::MakeCost(){
  VectorXd unom = VectorXd::Zero(nu_);
  unom(2) = 9.81 * plant_.mass();
  for (int i = 0; i < total_knots_; i++) {
      tracking_cost_.push_back(
          prog_.AddQuadraticErrorCost(Q_, x_des_, xx.at(i+1))
        .evaluator().get());
      input_cost_.push_back(
          prog_.AddQuadraticErrorCost(R_, unom, uu.at(i))
        .evaluator().get());
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
  int fsm_state = 0;
  if (use_fsm_) {
    fsm_state =
        (int) (discrete_state->get_vector(current_fsm_state_idx_)
            .get_value()(0));
    double last_event_time =
        discrete_state->get_vector(prev_event_time_idx_).get_value()(0);

    time_since_last_event = (last_event_time <= 0) ?
        timestamp : timestamp - last_event_time;

    UpdateConstraints(plant_.CalcSRBStateFromPlantState(x),
                                 fsm_state, time_since_last_event);
  } else {
    UpdateConstraints(plant_.CalcSRBStateFromPlantState(x), 0, 0);
  }

  result_ = solver_.Solve(prog_);
  std::cout << "result: " << result_.get_solution_result() << std::endl;

  most_recent_sol_ = MakeLcmTrajFromSol(
      result_, timestamp, time_since_last_event,  x, fsm_state);
  return EventStatus::Succeeded();
}

void SrbdCMPC::UpdateTrackingObjective(const VectorXd& xdes) const {
  x_des_ = xdes;
  for (auto & cost : tracking_cost_) {
    cost->UpdateCoefficients(2.0 * Q_, - 2.0 * Q_ * xdes);
  }
}

void SrbdCMPC::UpdateConstraints(
    const VectorXd& x0, const int fsm_state,
    const double t_since_last_switch) const {
  initial_state_->UpdateCoefficients(MatrixXd::Identity(nx_, nx_), x0);

  if (!use_fsm_) { return; }

  int n_until_next_state = (dt_ * modes_.at(fsm_state).N - t_since_last_switch) / dt_;
  UpdateDynamicsConstraints(x0, n_until_next_state, fsm_state);
  UpdateKinematicConstraints(n_until_next_state, fsm_state);
}


lcmt_saved_traj SrbdCMPC::MakeLcmTrajFromSol(
    const drake::solvers::MathematicalProgramResult& result,
    double time, double time_since_last_touchdown,
    const VectorXd& state, int fsm_state) const {

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
  MatrixXd com = MatrixXd::Zero(2*kLinearDim_ , total_knots_);
  MatrixXd orientation = MatrixXd::Zero(2*kAngularDim_ , total_knots_);
  VectorXd time_knots = VectorXd::Zero(total_knots_);



  for (int i = 0; i < total_knots_; i++) {
    VectorXd xi = result.GetSolution(xx.at(i));
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
      dt_ * (modes_.front().N) - time_since_last_touchdown;

  Vector3d swing_ft_traj_breaks(
      time, 0.5*(time + next_touchdown_time), next_touchdown_time);
  SwingFootTraj.time_vector = swing_ft_traj_breaks;
  SwingFootTraj.datapoints = CalcSwingFootKnotPoints(
      state, result,
      swing_ft_traj_breaks(1), fsm_state);;

  LcmTrajectory lcm_traj;
  lcm_traj.AddTrajectory(CoMTraj.traj_name, CoMTraj);
  lcm_traj.AddTrajectory(AngularTraj.traj_name, AngularTraj);
  lcm_traj.AddTrajectory(SwingFootTraj.traj_name, SwingFootTraj);

  return lcm_traj.GenerateLcmObject();
}

MatrixXd SrbdCMPC::CalcSwingFootKnotPoints(const VectorXd& x,
    const MathematicalProgramResult& result,
    double time_since_last_touchdown, int fsm_state) const {

  double t = time_since_last_touchdown / (modes_.at(fsm_state).N * dt_);
  Vector3d curr_pos =
      plant_.CalcFootPosition(x, modes_.at(1-fsm_state).stance);
  Vector3d next_pos =
      result.GetSolution(pp.at(1-fsm_state));

  Vector3d mid_pos = 0.5 * (curr_pos + next_pos);
  mid_pos(2) = swing_ft_ht_ * (t - pow(t, 2));

  Matrix3d knots;
  knots << curr_pos, mid_pos, next_pos;
  return knots;
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

void SrbdCMPC::print_constraint(
    const std::vector<drake::solvers::Constraint*>& constraints) const {
  for (auto &constraint : constraints) {
    auto constr = dynamic_cast<drake::solvers::LinearConstraint*> (constraint);
    std::cout << constr->get_description() << ":\n A:\n"
              << constr->A() << "\nb:\n"
              << constr->upper_bound() << std::endl;
  }
}

void SrbdCMPC::CopyDiscreteDynamicsConstraint(
    const SrbdMode& mode, bool current_stance,
    const Vector3d& foot_pos,
    const drake::EigenPtr<MatrixXd> &A,
    const drake::EigenPtr<VectorXd> &b) const {
  DRAKE_DEMAND(A != nullptr && b != nullptr);

  if (!current_stance) {
    A->block(0, 0, nx_, nx_ + kLinearDim_) = mode.dynamics.A;
    A->block(0, nx_ + kLinearDim_, nx_, nu_) = mode.dynamics.B;
    A->block(0, nx_ + nu_ + kLinearDim_, nx_, nx_) =
        -MatrixXd::Identity(nx_, nx_);
    *b = -mode.dynamics.b;
  } else {
    A->block(0, 0, nx_, nx_) = mode.dynamics.A.block(0, 0, nx_, nx_);
    A->block(0, nx_, nx_, nu_) = mode.dynamics.B;
    A->block(0, nx_ + nu_, nx_, nx_) = -MatrixXd::Identity(nx_, nx_);
    *b = mode.dynamics.A.block(0, nx_, nx_, kLinearDim_)
             * foot_pos - mode.dynamics.b;
    return;
  }
}

void SrbdCMPC::CheckSquareMatrixDimensions(
    const MatrixXd& M, const int n) const {
  DRAKE_DEMAND(M.rows() == n);
  DRAKE_DEMAND(M.cols() == n);
}

} // dairlib