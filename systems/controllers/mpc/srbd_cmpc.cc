#include "srbd_cmpc.h"
#include "common/file_utils.h"
#include <numeric>

#define A_matrix_t Eigen::Matrix<double, nx_, nx_ + kLinearDim_>
#define B_matrix_t Eigen::Matrix<double, nx_, nu_>
#define b_vector_t Eigen::Matrix<double, nx_, 1>

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
using Eigen::Matrix;

using dairlib::multibody::SingleRigidBodyPlant;
using dairlib::multibody::makeNameToPositionsMap;
using dairlib::multibody::makeNameToVelocitiesMap;
using dairlib::multibody::SetPositionsAndVelocitiesIfNew;
using dairlib::systems::OutputVector;
using dairlib::LcmTrajectory;
using dairlib::systems::residual_dynamics;

namespace dairlib{

SrbdCMPC::SrbdCMPC(const SingleRigidBodyPlant& plant,
                   double dt,
                   bool traj,
                   bool used_with_finite_state_machine,
                   bool used_with_residual_estimator):
    plant_(plant),
    use_fsm_(used_with_finite_state_machine),
    use_residuals_(used_with_residual_estimator),
    traj_tracking_(traj),
    dt_(dt){

  // Create Ports
  state_port_ = this->DeclareVectorInputPort(
      "x, u, t",
      OutputVector<double>(
          plant_.nq(), plant_.nv(), plant_.nu()))
  .get_index();

  if (!traj_tracking_) {
    x_des_port_ = this->DeclareVectorInputPort("x_des",
        BasicVector<double>(nx_)).get_index();
  }

  if (use_residuals_) {
    srbd_residual_port_ = this->DeclareAbstractInputPort(
          "residual_input_port",
        drake::Value<residual_dynamics>(
            {MatrixXd::Zero(0, 0),
             MatrixXd::Zero(0, 0),
             VectorXd::Zero(0)})).get_index();
    std::cout << "declared residual port!\n";
  }
  std::cout << "state port: " << state_port_ << "\nresidual_port: " << srbd_residual_port_ << std::endl;
//  foot_target_port_ = this->DeclareVectorInputPort("p_des" ,
//      BasicVector<double>(2*kLinearDim_)).get_index();
//
// PiecewisePolynomial<double> pp_traj;
//  srb_warmstart_port_ = this->DeclareAbstractInputPort(
//      "initial_guess",
//      drake::Value<drake::trajectories::Trajectory<double>>(pp_traj)).get_index();

  traj_out_port_ = this->DeclareAbstractOutputPort("y(t)",
      &SrbdCMPC::GetMostRecentMotionPlan,
      {this->all_state_ticket()}).get_index();

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
        nx_, "x_" + std::to_string(stance) + std::to_string(i)));
    uu.push_back(prog_.NewContinuousVariables(
        nu_, "u_" + std::to_string(stance) + std::to_string(i)));
  }
  pp.push_back(prog_.NewContinuousVariables(
      kLinearDim_, "p_" + std::to_string(stance)));
  total_knots_ += N;
  modes_.push_back(mode);
  nmodes_ ++;
}

void SrbdCMPC::FinalizeModeSequence(){
     xx.push_back(prog_.NewContinuousVariables(nx_, "x_f"));
     uu.push_back(prog_.NewContinuousVariables(nu_, "u_f"));
     if (use_residuals_) {
       residual_manager_ =
           std::make_unique<systems::MpcPeriodicResidualManager>(
               total_knots_,
               modes_.front().dynamics.A,
               modes_.front().dynamics.B,
               modes_.front().dynamics.b);
     }
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
  CheckSquareMatrixDimensions(Qf_, nx_);
  CheckSquareMatrixDimensions(R_, nu_);
  CheckSquareMatrixDimensions(Wp_, kLinearDim_);
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
  solver_options.SetOption(OsqpSolver::id(), "verbose", 1);
  solver_options.SetOption(OsqpSolver::id(), "eps_abs", 1e-5);
  solver_options.SetOption(OsqpSolver::id(), "eps_rel", 1e-5);
  solver_options.SetOption(OsqpSolver::id(), "eps_prim_inf", 1e-4);
  solver_options.SetOption(OsqpSolver::id(), "eps_dual_inf", 1e-4);
  solver_options.SetOption(OsqpSolver::id(), "polish", 1);
  solver_options.SetOption(OsqpSolver::id(), "scaled_termination", 1);
  solver_options.SetOption(OsqpSolver::id(), "adaptive_rho_fraction", 1.0);
  solver_options.SetOption(OsqpSolver::id(), "max_iter", 20000);
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
      MatrixXd Aeq = MatrixXd::Zero(nx_, 2*(nx_ + nu_) + kLinearDim_);
      VectorXd beq = VectorXd::Zero(nx_);
      Vector3d pos = Vector3d::Zero();
      pos(1) = nominal_foot_pos_.at(j)(1);
      CopyCollocationDynamicsConstraint(mode.dynamics, false, pos, &Aeq, &beq);
      dynamics_.push_back(
          prog_.AddLinearEqualityConstraint(Aeq, beq,
              {xx.at(j * mode.N + i),
               xx.at(j * mode.N + i + 1),
               uu.at(j * mode.N + i),
               uu.at(j * mode.N + i + 1),
               pp.at(mode.stance)}));
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

  std::cout << "Next stance: ";
  for (int i = n_until_stance; i <= n_until_stance + next_mode.N; i++) {
    kinematic_constraint_.push_back(
        prog_.AddLinearConstraint(
            A,
            nominal_foot_pos_.at(next_mode.stance) - kin_bounds_,
            nominal_foot_pos_.at(next_mode.stance) + kin_bounds_,
            {pp.at(next_mode.stance), xx.at(i).head(kLinearDim_)}));
    std::cout << std::to_string(i) << " ";
  }
  std::cout << std::endl;
  std::cout << "Next next stance: ";
  for (int i = n_until_stance + next_mode.N; i <= total_knots_; i++) {
    kinematic_constraint_.push_back(
        prog_.AddLinearConstraint(
            A,
            nominal_foot_pos_.at(curr_mode.stance) - kin_bounds_,
            nominal_foot_pos_.at(curr_mode.stance) + kin_bounds_,
            {pp.at(curr_mode.stance), xx.at(i).head(kLinearDim_)}));
    std::cout << std::to_string(i) << " ";
  }
  std::cout << std::endl;
}

void SrbdCMPC::UpdateDynamicsConstraints(const Eigen::VectorXd& x,
    int n_until_next_stance, int fsm_state) const {

  auto& mode = modes_.at(fsm_state);
  Vector3d pos = plant_.CalcFootPosition(x, mode.stance);

  if (n_until_next_stance == mode.N ) {
    // make current stance dynamics and apply to mode
    MatrixXd Aeq = MatrixXd::Zero(nx_, 2*(nx_ + nu_));
    VectorXd beq = VectorXd::Zero(nx_);
    CopyCollocationDynamicsConstraint(
        mode.dynamics, true, pos, &Aeq, &beq);

    for (int i = 0; i < (mode.N); i++){
      prog_.RemoveConstraint(dynamics_.at(i));
      dynamics_.at(i) = prog_.AddLinearEqualityConstraint(
          Aeq, beq,
          {xx.at(i), xx.at(i+1), uu.at(i), uu.at(i+1)});
    }

    Aeq = MatrixXd::Zero(nx_, 2*(nx_ + nu_) + kLinearDim_);
    beq = VectorXd::Zero(nx_);
    CopyCollocationDynamicsConstraint(modes_.at(1-fsm_state).dynamics,
        false, pos, &Aeq, &beq);
    prog_.RemoveConstraint(dynamics_.at(mode.N));
    dynamics_.at(mode.N) = prog_.AddLinearEqualityConstraint(
        Aeq, beq,
        {xx.at(mode.N), xx.at(mode.N+1),
          uu.at(mode.N), uu.at(mode.N+1),
          pp.at(1-fsm_state)});
  } else {
    int idx = n_until_next_stance;
    MatrixXd Aeq1 = MatrixXd::Zero(nx_, 2*(nx_ + nu_) + kLinearDim_);
    MatrixXd Aeq2 = MatrixXd::Zero(nx_, 2*(nx_ + nu_) + kLinearDim_);
    VectorXd beq1 = VectorXd::Zero(nx_);
    VectorXd beq2 = VectorXd::Zero(nx_);
    CopyCollocationDynamicsConstraint(
        modes_.at(1-fsm_state).dynamics, false, pos, &Aeq1, &beq1);

    prog_.RemoveConstraint(dynamics_.at(idx));
    dynamics_.at(idx) = prog_.AddLinearEqualityConstraint(
        Aeq1, beq1,
        {xx.at(idx), xx.at(idx+1),
         uu.at(idx), uu.at(idx+1), pp.at(1-fsm_state)});

    CopyCollocationDynamicsConstraint(mode.dynamics, false, pos, &Aeq2, &beq2);
    prog_.RemoveConstraint(dynamics_.at(idx + mode.N));
    dynamics_.at(idx+mode.N) = prog_.AddLinearEqualityConstraint(
        Aeq2, beq2,
        {xx.at(idx+mode.N), xx.at(idx+mode.N+1),
         uu.at(idx+mode.N), uu.at(idx+mode.N+1), pp.at(fsm_state)});
  }
  std::cout << "\n";
}

void SrbdCMPC::UpdateResidualDynamicsConstraints(const Eigen::VectorXd& x,
                                         int n_until_next_stance, int fsm_state) const {

  auto& mode = modes_.at(fsm_state);
  Vector3d pos = plant_.CalcFootPosition(x, mode.stance);

  if (n_until_next_stance == mode.N ) {
    // make current stance dynamics and apply to mode after
    // adding current residual estimate
    MatrixXd Aeq = MatrixXd::Zero(nx_, 2*(nx_ + nu_));
    VectorXd beq = VectorXd::Zero(nx_);

    for (int i = 0; i < (mode.N); i++){
      LinearSrbdDynamics dyn_i = {A_matrix_t::Zero(), B_matrix_t::Zero(),
                                  b_vector_t::Zero()};
      residual_manager_->AddResidualToDynamics(
          residual_dynamics{mode.dynamics.A, mode.dynamics.B, mode.dynamics.b},
          residual_manager_->GetResidualForKnotFromCurrent(i),
          &dyn_i.A, &dyn_i.B, &dyn_i.b);
      CopyCollocationDynamicsConstraint(
          dyn_i, true, pos, &Aeq, &beq);

      prog_.RemoveConstraint(dynamics_.at(i));
      dynamics_.at(i) = prog_.AddLinearEqualityConstraint(
          Aeq, beq,
          {xx.at(i), xx.at(i+1), uu.at(i), uu.at(i+1)});

    }

    Aeq = MatrixXd::Zero(nx_, 2*(nx_ + nu_) + kLinearDim_);
    beq = VectorXd::Zero(nx_);
    LinearSrbdDynamics dyn_f = {A_matrix_t::Zero(), B_matrix_t::Zero(), b_vector_t::Zero()};
    residual_manager_->AddResidualToDynamics(
        residual_dynamics{modes_.at(1-fsm_state).dynamics.A,
                          modes_.at(1-fsm_state).dynamics.B,
                          modes_.at(1-fsm_state).dynamics.b},
        residual_manager_->GetResidualForKnotFromCurrent(mode.N),
        &dyn_f.A, &dyn_f.B, &dyn_f.b);
    CopyCollocationDynamicsConstraint(
        dyn_f,false, pos, &Aeq, &beq);
    prog_.RemoveConstraint(dynamics_.at(mode.N));
    dynamics_.at(mode.N) = prog_.AddLinearEqualityConstraint(
        Aeq, beq,
        {xx.at(mode.N), xx.at(mode.N+1),
         uu.at(mode.N), uu.at(mode.N+1),
         pp.at(1-fsm_state)});
  } else {
    int idx = n_until_next_stance;

    LinearSrbdDynamics dyn_f = {A_matrix_t::Zero(), B_matrix_t::Zero(), b_vector_t::Zero()};
    residual_manager_->AddResidualToDynamics(
        residual_dynamics{modes_.at(1-fsm_state).dynamics.A,
                          modes_.at(1-fsm_state).dynamics.B,
                          modes_.at(1-fsm_state).dynamics.b},
        residual_manager_->GetResidualForKnotFromCurrent(mode.N),
        &dyn_f.A, &dyn_f.B, &dyn_f.b);

    MatrixXd Aeq = MatrixXd::Zero(nx_, 2*(nx_ + nu_) + kLinearDim_);
    VectorXd beq = VectorXd::Zero(nx_);
    CopyCollocationDynamicsConstraint(
        dyn_f, false, pos, &Aeq, &beq);

    prog_.RemoveConstraint(dynamics_.at(idx));
    dynamics_.at(idx) = prog_.AddLinearEqualityConstraint(
        Aeq, beq,
        {xx.at(idx), xx.at(idx+1),
         uu.at(idx), uu.at(idx+1), pp.at(1-fsm_state)});

    residual_manager_->AddResidualToDynamics(
        residual_dynamics{mode.dynamics.A,
                          mode.dynamics.B,
                          mode.dynamics.b},
        residual_manager_->GetResidualForKnotFromCurrent(mode.N),
        &dyn_f.A, &dyn_f.B, &dyn_f.b);
    CopyCollocationDynamicsConstraint(dyn_f, false, pos, &Aeq, &beq);

    prog_.RemoveConstraint(dynamics_.at(idx + mode.N));
    dynamics_.at(idx+mode.N) = prog_.AddLinearEqualityConstraint(
        Aeq, beq,
        {xx.at(idx+mode.N), xx.at(idx+mode.N+1),
         uu.at(idx+mode.N), uu.at(idx+mode.N+1), pp.at(fsm_state)});
  }
  std::cout << "\n";
}

void SrbdCMPC::UpdateFootPlacementCost(
    int fsm_state,
    const Eigen::VectorXd& up_next_foot_target,
    const Eigen::VectorXd& on_deck_foot_target) const  {
  foot_target_cost_.at(fsm_state)->UpdateCoefficients(
      2.0 * Wp_, - 2.0 * Wp_ * on_deck_foot_target);
  foot_target_cost_.at(1-fsm_state)->UpdateCoefficients(
      2.0 * Wp_, - 2.0 * Wp_ * up_next_foot_target);
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
  for (int i = 0; i <= total_knots_; i++) {
      friction_cone_.push_back(
          prog_.AddConstraint(
          solvers::CreateLinearFrictionConstraint(mu_),
          uu.at(i).head(kLinearDim_)).evaluator().get());
  }
}

void SrbdCMPC::MakeCost(){
  VectorXd unom = VectorXd::Zero(nu_);
  unom(2) = 9.81 * plant_.mass();
  for (int i = 0; i <= total_knots_; i++) {
      tracking_cost_.push_back(
          prog_.AddQuadraticErrorCost(Q_, x_des_, xx.at(i))
        .evaluator().get());
      input_cost_.push_back(
          prog_.AddQuadraticErrorCost(R_, unom, uu.at(i))
        .evaluator().get());
  }
  for (int i = 0; i < nmodes_; i++) {
    foot_target_cost_.push_back(
        prog_.AddQuadraticErrorCost(
            Wp_, nominal_foot_pos_.at(i), pp.at(i))
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

void SrbdCMPC::AddFootPlacementRegularization(const Eigen::MatrixXd &W) {
  Wp_ = W;
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

//  const drake::AbstractValue* traj_value =
//      this->EvalAbstractInput(context, srb_warmstart_port_);
//
//  const auto& warmstart_traj =
//      traj_value->get_value<drake::trajectories::Trajectory<double>>();

  VectorXd x = robot_output->GetState();

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
    if (use_residuals_) {
      residual_dynamics res = this->EvalAbstractInput(context, srbd_residual_port_)->
          get_value<residual_dynamics>();
      DRAKE_DEMAND(res.A.cols() == nx_+kLinearDim_);
      DRAKE_DEMAND(res.B.cols() == nu_);
      DRAKE_DEMAND(res.b.rows() == nx_);
      residual_manager_->SetResidualForCurrentKnot(res);
    }

  } else {
    UpdateConstraints(plant_.CalcSRBStateFromPlantState(x), 0, 0);
  }

//  VectorXd foot_target =
//      this->EvalVectorInput(context, foot_target_port_)->get_value();
  UpdateFootPlacementCost(
      fsm_state,
      Vector3d::Zero(),
      Vector3d::Zero());
//
//  SetInitialGuess(fsm_state, timestamp,
//                  foot_target.head(kLinearDim_),
//                  foot_target.tail(kLinearDim_),
//                  warmstart_traj);

  auto lin_con = prog_.GetAllLinearConstraints();
  for (auto& binding : lin_con) {
    std::cout << "Next Binding:\n" << binding << std::endl;
  }
//  print_constraint(lin_con);


  result_ = solver_.Solve(prog_);


  if (!result_.is_success()) {
    std::cout << "result: " << result_.get_solution_result() << std::endl;
  }

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
  std::cout << "Time since last switch: " <<
      std::to_string(t_since_last_switch) << std::endl;
  int n_until_next_state = modes_.at(fsm_state).N -
      std::floor(t_since_last_switch / dt_);
  if (use_residuals_) {
    UpdateResidualDynamicsConstraints(x0, n_until_next_state, fsm_state);
    residual_manager_->CycleCurrentKnot();
  } else {
    UpdateDynamicsConstraints(x0, n_until_next_state, fsm_state);
  }
  UpdateKinematicConstraints(n_until_next_state, fsm_state);
}

void SrbdCMPC::SetInitialGuess(
    int fsm_state, double timestamp,
    const Eigen::VectorXd& up_next_stance_target,
    const Eigen::VectorXd& on_deck_stance_target,
    const drake::trajectories::Trajectory<double>& srb_traj) const {
  prog_.SetInitialGuess(pp.at(fsm_state), on_deck_stance_target);
  prog_.SetInitialGuess(pp.at(1-fsm_state), up_next_stance_target);
  VectorXd srb_guess(nx_);
  for (int i = 0; i < total_knots_; i++) {
    double t = timestamp + i*dt_;

    Eigen::Vector4d u_guess(0, 0, plant_.mass() * 9.81, 0);
    srb_guess.head(nx_/2) = srb_traj.value(t);
    srb_guess.tail(nx_/2) = srb_traj.EvalDerivative(t, 1);
    prog_.SetInitialGuess(xx.at(i), srb_guess);
    prog_.SetInitialGuess(uu.at(i), u_guess);
  }
  double t = timestamp + total_knots_ * dt_;
  srb_guess.head(nx_/2) = srb_traj.value(t);
  srb_guess.tail(nx_/2) = srb_traj.EvalDerivative(t, 1);
  prog_.SetInitialGuess(xx.back(), srb_guess);
}

lcmt_saved_traj SrbdCMPC::MakeLcmTrajFromSol(
    const drake::solvers::MathematicalProgramResult& result,
    double time, double time_since_last_touchdown,
    const VectorXd& state, int fsm_state) const {

  DRAKE_DEMAND(result.is_success());

  LcmTrajectory::Trajectory CoMTraj;
  LcmTrajectory::Trajectory AngularTraj;
  LcmTrajectory::Trajectory SwingFootTraj;
  LcmTrajectory::Trajectory InputTraj;

  CoMTraj.traj_name = "com_traj";
  AngularTraj.traj_name = "orientation";
  SwingFootTraj.traj_name = "swing_foot_traj";
  InputTraj.traj_name = "input_traj";


  /** need to set datatypes for LcmTrajectory to save properly **/
  for (int i = 0; i < 2*kLinearDim_; i++) {
    CoMTraj.datatypes.emplace_back("double");
    AngularTraj.datatypes.emplace_back("double");
  }
  for (int i = 0; i < kLinearDim_; i++) {
    SwingFootTraj.datatypes.emplace_back("double");
    InputTraj.datatypes.emplace_back("double");
  }
  InputTraj.datatypes.emplace_back("double");

  /** Allocate Eigen matrices for trajectory blocks **/
  MatrixXd com = MatrixXd::Zero(2*kLinearDim_ , total_knots_);
  MatrixXd orientation = MatrixXd::Zero(2*kAngularDim_ , total_knots_);
  MatrixXd input = MatrixXd::Zero(nu_, total_knots_);
  VectorXd time_knots = VectorXd::Zero(total_knots_);

  for (int i = 0; i < total_knots_; i++) {
    VectorXd ui = result.GetSolution(uu.at(i));
    VectorXd xi = result.GetSolution(xx.at(i));
    input.col(i) = ui;
    com.block(0, i, 2*kLinearDim_, 1) << xi.head(kLinearDim_),
        xi.segment(kLinearDim_ + kAngularDim_, kLinearDim_);
    orientation.block(0, i, 2*kAngularDim_, 1) <<
        xi.segment(kLinearDim_, kAngularDim_), xi.tail(kAngularDim_);

    time_knots(i) = time + dt_ * i;
  }

  InputTraj.time_vector = time_knots;
  InputTraj.datapoints = input;
  CoMTraj.time_vector = time_knots;
  CoMTraj.datapoints = com;
  AngularTraj.time_vector = time_knots;
  AngularTraj.datapoints = orientation;

  VectorXd swing_ft_traj_breaks = VectorXd::Ones(1) * time;
  SwingFootTraj.time_vector = swing_ft_traj_breaks;
  SwingFootTraj.datapoints = result.GetSolution(pp.at(1-fsm_state));

  LcmTrajectory lcm_traj;
  lcm_traj.AddTrajectory(CoMTraj.traj_name, CoMTraj);
  lcm_traj.AddTrajectory(AngularTraj.traj_name, AngularTraj);
  lcm_traj.AddTrajectory(SwingFootTraj.traj_name, SwingFootTraj);
  lcm_traj.AddTrajectory(InputTraj.traj_name, InputTraj);

  return lcm_traj.GenerateLcmObject();
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
    *b = -(mode.dynamics.A.block(0, nx_, nx_, kLinearDim_)
             * foot_pos + mode.dynamics.b);
  }
}

void SrbdCMPC::CopyCollocationDynamicsConstraint(
    const LinearSrbdDynamics& dyn, bool current_stance,
    const Eigen::Vector3d &foot_pos,
    const drake::EigenPtr<Eigen::MatrixXd> &A,
    const drake::EigenPtr<Eigen::VectorXd> &b) const {
  DRAKE_DEMAND(A != nullptr && b != nullptr);
  if (current_stance) {
    DRAKE_DEMAND(A->cols() == 2*(nx_+nu_));
  } else {
    DRAKE_DEMAND(A->cols() == 2*(nx_+nu_)+3);
  }

  Matrix<double, nx_, nx_> Ax = dyn.A.block(0, 0, nx_, nx_);
  Matrix<double, nx_, 3> Ap = dyn.A.block(0, nx_, nx_, nx_+kLinearDim_);
  double h = dt_;
  A->block(0, 0, nx_, nx_) =
      (0.125*h)*Ax*Ax - 0.75*Ax - (1.5/h) * MatrixXd::Identity(nx_, nx_);
  A->block(0, nx_, nx_, nx_) =
      -(0.125*h)*Ax*Ax - 0.75*Ax + (1.5/h) * MatrixXd::Identity(nx_, nx_);
  A->block(0, 2*nx_, nx_, nu_) =
      (0.125 * h * Ax - 0.75 * MatrixXd::Identity(nx_, nx_)) * dyn.B;
  A->block(0, 2*nx_ + nu_, nx_, nu_) =
      (-0.125 * h * Ax - 0.75 * MatrixXd::Identity(nx_, nx_)) * dyn.B;
  if (current_stance) {
    *b = 1.5 * dyn.b + 1.5 * Ap * foot_pos;
  } else {
    A->block(0, 2*(nx_ + nu_), nx_, kLinearDim_) = -1.5*Ap;
    *b = 1.5 * dyn.b;
  }
}

void SrbdCMPC::CheckSquareMatrixDimensions(
    const MatrixXd& M, const int n) const {
  DRAKE_DEMAND(M.rows() == n);
  DRAKE_DEMAND(M.cols() == n);
}

} // dairlib