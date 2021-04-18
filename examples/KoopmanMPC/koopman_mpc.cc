//
// Created by brian on 3/8/21.
//

#include "koopman_mpc.h"
#include "common/file_utils.h"


using drake::multibody::JacobianWrtVariable;
using drake::multibody::MultibodyPlant;
using drake::multibody::BodyFrame;

using drake::systems::Context;
using drake::systems::BasicVector;
using drake::systems::EventStatus;
using drake::trajectories::PiecewisePolynomial;

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
using Eigen::Quaterniond;

using dairlib::multibody::WorldPointEvaluator;
using dairlib::multibody::makeNameToPositionsMap;
using dairlib::multibody::makeNameToVelocitiesMap;
using dairlib::systems::OutputVector;
using dairlib::LcmTrajectory;

namespace dairlib{

KoopmanMPC::KoopmanMPC(const MultibodyPlant<double>& plant,
                       Context<double> *plant_context, double dt,
                       bool planar, bool used_with_finite_state_machine,
                       bool use_com) :
                       plant_(plant),
                       plant_context_(plant_context),
                       world_frame_(plant_.world_frame()), dt_(dt),
                       planar_(planar), use_com_(use_com),
                       use_fsm_(used_with_finite_state_machine){

  nq_ = plant.num_positions();
  nv_ = plant.num_velocities();
  nu_p_ = plant.num_actuators();

  nx_ = planar ? kNxPlanar : kNx3d;
  nu_c_ = planar ? kNuPlanar : kNu3d;
  kLinearDim_ = planar ? 2 : 3;
  kAngularDim_ = planar ? 1 : 4;

  nxi_ = nx_ + nu_c_;
  Q_ = MatrixXd::Zero(nxi_, nxi_);

  // Create Ports
  state_port_ = this->DeclareVectorInputPort(
      OutputVector<double>(nq_, nv_, nu_p_)).get_index();

  x_des_port_ = this->DeclareVectorInputPort(
      BasicVector<double>(nxi_)).get_index();

  this->DeclareAbstractOutputPort(&KoopmanMPC::CalcOptimalMotionPlan);

  // Discrete update
  DeclarePerStepDiscreteUpdateEvent(&KoopmanMPC::DiscreteVariableUpdate);


  if ( use_fsm_ ) {
    fsm_port_ = this->DeclareVectorInputPort(
        BasicVector<double>(1)).get_index();

    current_fsm_state_idx_ =
        this->DeclareDiscreteState(VectorXd::Zero(1));
    prev_event_time_idx_ = this->DeclareDiscreteState(VectorXd::Zero(1));
  }
  x_des_idx_ = this->DeclareDiscreteState(VectorXd::Zero(nxi_));

}

void KoopmanMPC::AddMode(const KoopmanDynamics& dynamics,
    koopMpcStance stance, int N) {
  if (modes_.empty()) {
    nz_ = dynamics.x_basis_func(VectorXd::Zero(nxi_)).size();
  }

  DRAKE_DEMAND(dynamics.x_basis_func(VectorXd::Zero(nxi_)).size() == nz_);

  KoopmanMpcMode mode;
  mode.dynamics = dynamics;
  mode.stance = stance;
  mode.N = N;

  mode_knot_counts_.push_back(mode.N);
  total_knots_ += N;

  for ( int i = 0; i < N+1; i++) {
    mode.zz.push_back(prog_.NewContinuousVariables(nz_, "z_" + std::to_string(i)));
  }
  for (int i = 0; i < N; i++) {
    mode.uu.push_back(prog_.NewContinuousVariables(nu_c_, "u_" + std::to_string(i+1)));
  }
  modes_.push_back(mode);
  nmodes_++;
}

void KoopmanMPC::AddContactPoint(std::pair<const drake::multibody::BodyFrame<double>&,
                                           Eigen::Vector3d> pt, koopMpcStance stance) {
  DRAKE_ASSERT(contact_points_.size() == stance)
  contact_points_.push_back(pt);
}

void KoopmanMPC::AddJointToTrackBaseAngle(const std::string& joint_pos_name,
    const std::string& joint_vel_name) {
  DRAKE_ASSERT(planar_)
  base_angle_pos_idx_ = makeNameToPositionsMap(plant_)[joint_pos_name];
  base_angle_vel_idx_ = makeNameToVelocitiesMap(plant_)[joint_vel_name];
}

void KoopmanMPC::AddBaseFrame(const std::string &body_name,
const Eigen::Vector3d& offset, const Eigen::Isometry3d& frame_pose) {
  DRAKE_ASSERT(!planar_)
  base_ = body_name;
  frame_pose_ = frame_pose;
  com_from_base_origin_ = offset;
}

double KoopmanMPC::SetMassFromListOfBodies(std::vector<std::string> bodies) {
  double mass = CalcCentroidalMassFromListOfBodies(bodies);
  mass_ = mass;

  MatrixXd knots = MatrixXd::Zero(kLinearDim_, 2);
  MatrixXd knots_dot = MatrixXd::Zero(kLinearDim_, 2);
  Vector2d time = {0.0, 100.0};

  for (int i = 0; i < 2; i++) {
    if (planar_) {
      knots.block(0, i, kLinearDim_, 1) =
          MakePlanarVectorFrom3d(-mass_* gravity_);
    } else {
      knots.block(0, i, kLinearDim_, 1) = -mass_ * gravity_;
    }
  }

  prev_sol_base_traj_ =
      PiecewisePolynomial<double>::CubicHermite(
          time, knots, knots_dot);

  return mass;
}

void KoopmanMPC::SetReachabilityLimit(const Eigen::VectorXd& kl,
    const std::vector<Eigen::VectorXd> &kn) {
  DRAKE_DEMAND(kl.size() == kLinearDim_);

  kin_lim_ = kl;
  for (auto pos : kn) {
    kin_nominal_.push_back(pos);
  }
}

void KoopmanMPC::CheckProblemDefinition() {
  DRAKE_DEMAND(!modes_.empty());
  DRAKE_DEMAND(!tracking_cost_.empty());
  DRAKE_DEMAND(!input_cost_.empty());
  DRAKE_DEMAND(mu_ > 0 );
  DRAKE_DEMAND(!contact_points_.empty());
  DRAKE_DEMAND(!kin_nominal_.empty());
  DRAKE_DEMAND(mass_ > 0);
}

void KoopmanMPC::Build() {
  CheckProblemDefinition();

  MakeDynamicsConstraints();
  MakeFrictionConeConstraints();
  MakeStanceFootConstraints();
  MakeKinematicReachabilityConstraints();
  MakeStateKnotConstraints();
  MakeInitialStateConstraints();
  //MakeFlatGroundConstraints();
  std::cout << "Built Koopman Mpc QP: \nModes: " << std::to_string(nmodes_) <<
               "\nTotal Knots: " << std::to_string(total_knots_) << std::endl;

  prog_.SetSolverOption(OsqpSolver().id(),
      "time_limit", kMaxSolveDuration_);
}

void KoopmanMPC::MakeStanceFootConstraints() {
  MatrixXd S = MatrixXd::Identity(kLinearDim_, kLinearDim_);
  for (auto & mode : modes_) {
    // Loop over N inputs
    for (int i = 0; i < mode.N; i++) {
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

void KoopmanMPC::MakeDynamicsConstraints() {
  for (auto & mode : modes_) {
    MatrixXd dyn = MatrixXd::Zero(nz_, 2 * nz_ + nu_c_);
    dyn.block(0, 0, nz_, nz_) = mode.dynamics.A;
    dyn.block(0, nz_, nz_, nu_c_) = mode.dynamics.B;
    dyn.block(0, nz_, nz_ + nu_c_, nz_) = -MatrixXd::Identity(nz_, nz_);
    for (int i = 0; i < mode.N; i++) {
      mode.dynamics_constraints.push_back(
          prog_.AddLinearEqualityConstraint(
             dyn, -mode.dynamics.b,
             {mode.zz.at(i), mode.uu.at(i), mode.zz.at(i+1)})
              .evaluator()
              .get());

      mode.dynamics_constraints.at(i)->set_description(
          "dyn" + std::to_string(mode.stance) + "." + std::to_string(i));
    }
  }
}

void KoopmanMPC::MakeInitialStateConstraints() {
  for (auto & mode : modes_) {
    for (int i = 0; i < mode.N; i++) {
      mode.init_state_constraint_.push_back(
          prog_.AddLinearEqualityConstraint(
                  MatrixXd::Zero(1, nz_), VectorXd::Zero(1),
                  mode.zz.at(i))
              .evaluator()
              .get());
      mode.init_state_constraint_.at(i)->set_description(
          "x0" + std::to_string(mode.stance) + "." + std::to_string(i));
    }
  }
}

void KoopmanMPC::MakeKinematicReachabilityConstraints() {
  MatrixXd S = MatrixXd::Zero(kLinearDim_, kLinearDim_ * 2);

  S.block(0, 0, kLinearDim_, kLinearDim_) =
      -MatrixXd::Identity(kLinearDim_, kLinearDim_);
  S.block(0, kLinearDim_, kLinearDim_, kLinearDim_) =
      MatrixXd::Identity(kLinearDim_, kLinearDim_);

  for (auto & mode : modes_) {
    // Loop over N+1 states
    for (int i = 0; i <= mode.N; i++) {
      mode.reachability_constraints.push_back(
          prog_.AddLinearConstraint(S,
              -kin_lim_ + kin_nominal_.at(mode.stance),
              kin_lim_ + kin_nominal_.at(mode.stance),
              {mode.zz.at(i).head(kLinearDim_),
               mode.zz.at(i).segment(nx_ + kLinearDim_ * mode.stance,
                   kLinearDim_)})
               .evaluator()
               .get());

      mode.reachability_constraints.at(i)->set_description(
          "Reachability" + std::to_string(mode.stance) + "." + std::to_string(i));
    }
  }
}

void KoopmanMPC::MakeFrictionConeConstraints() {
  for (auto & mode : modes_) {
    for (int i = 0; i <= mode.N; i++) {
      if (! planar_ ) {
        mode.friction_constraints.push_back(
            prog_.AddConstraint(
                solvers::CreateLinearFrictionConstraint(mu_),
                mode.zz.at(i).segment(nxi_ - kLinearDim_, kLinearDim_))
                .evaluator().get());
      } else {
        MatrixXd cone(kLinearDim_, kLinearDim_);
        cone << -1, mu_, 1, mu_;
        mode.friction_constraints.push_back(
            prog_.AddLinearConstraint(
                cone, VectorXd::Zero(kLinearDim_),
                VectorXd::Constant(kLinearDim_, std::numeric_limits<double>::infinity()),
                mode.zz.at(i).segment(nxi_ - kLinearDim_, kLinearDim_))
                .evaluator()
                .get());
      }
      mode.friction_constraints.at(i)->set_description(
          "Friction" + std::to_string(mode.stance) + "." + std::to_string(i));
    }
  }
}

void KoopmanMPC::MakeFlatGroundConstraints() {
  for (auto & mode : modes_) {
    for (int i = 0; i < mode.N; i++) {
      mode.flat_ground_constraints.push_back(
          prog_.AddLinearEqualityConstraint(
          MatrixXd::Identity(1, 1),
          VectorXd::Zero(1),
          mode.zz.at(i).segment(nx_ + kLinearDim_ * (mode.stance + 1)-1, 1))
          .evaluator()
          .get());

      mode.flat_ground_constraints.at(i)->set_description(
          "FlatGrounConst" + std::to_string(mode.stance) + "." + std::to_string(i));
    }
  }
}

void KoopmanMPC::MakeStateKnotConstraints() {
  // Dummy constraint to be used when cycling modes
  state_knot_constraints_.push_back(
      prog_.AddLinearEqualityConstraint(
          MatrixXd::Zero(1, 2*nz_),
          VectorXd::Zero(1),
          {modes_.back().zz.back(), modes_.front().zz.front()})
          .evaluator()
          .get());

  MatrixXd Aeq = MatrixXd::Identity(nz_, 2 * nz_);
  Aeq.block(0, nz_, nz_, nz_) = -MatrixXd::Identity(nz_, nz_);
  for (int i = 0; i < modes_.size() - 1; i++) {
    state_knot_constraints_.push_back(
        prog_.AddLinearEqualityConstraint( Aeq, VectorXd::Zero(nz_),
            {modes_.at(i).zz.back(), modes_.at(i+1).zz.front()})
            .evaluator()
            .get());
    state_knot_constraints_.at(i)->set_description(
        "KnotConstraint" + std::to_string(i));
  }
}

void KoopmanMPC::AddTrackingObjective(const Eigen::VectorXd &xdes,
                                      const Eigen::MatrixXd &Q) {
  Q_ = Q;
  for (auto & mode : modes_) {
    // loop over N+1 states
    for (int i = 0; i <= mode.N; i++ ) {
      if (i != 0) {
        tracking_cost_.push_back(
            prog_.AddQuadraticErrorCost(
                Q, xdes, mode.zz.at(i).head(nxi_))
            .evaluator().get());
      }
    }
  }
}

void KoopmanMPC::AddInputRegularization(const Eigen::MatrixXd &R) {
  DRAKE_DEMAND(R.cols() == nu_c_);
  DRAKE_DEMAND(R.rows() == nu_c_);

  for(auto & mode : modes_) {
    // loop over N inputs
    for (int i = 0; i < mode.N; i++) {
      input_cost_.push_back(
          prog_.AddQuadraticCost(R, VectorXd::Zero(nu_c_), mode.uu.at(i))
          .evaluator()
          .get());
    }
  }
}

VectorXd KoopmanMPC::CalcCentroidalStateFromPlant(VectorXd x,
    double t) const {

  Vector3d com_pos;
  Vector3d com_vel;
  MatrixXd  J_CoM_v = MatrixXd::Zero(3, nv_);

  Vector3d left_pos;
  Vector3d right_pos;
  VectorXd lambda;

  plant_.SetPositionsAndVelocities(plant_context_, x);

  if (use_com_) {
    com_pos = plant_.CalcCenterOfMassPositionInWorld(*plant_context_);
    plant_.CalcJacobianCenterOfMassTranslationalVelocity(*plant_context_,
        JacobianWrtVariable::kV, world_frame_, world_frame_, &J_CoM_v);
  } else {
    com_pos = plant_.GetBodyByName(base_).EvalPoseInWorld(*plant_context_).translation();
    plant_.CalcJacobianTranslationalVelocity(*plant_context_,
        JacobianWrtVariable::kV,plant_.GetBodyByName(base_).body_frame(),
        com_from_base_origin_, world_frame_, world_frame_, &J_CoM_v);
  }

  com_vel = J_CoM_v * x.tail(nv_);

  plant_.CalcPointsPositions(*plant_context_, contact_points_.at(kLeft).first,
      contact_points_.at(kLeft).second, world_frame_, &left_pos);
  plant_.CalcPointsPositions(*plant_context_, contact_points_.at(kRight).first,
                             contact_points_.at(kRight).second, world_frame_, &right_pos);
  VectorXd base_orientation;
  VectorXd base_omega;

  if (planar_) {
    base_orientation = x.head(nq_).segment(base_angle_pos_idx_, kAngularDim_);
    base_omega = x.tail(nv_).segment(base_angle_vel_idx_, kAngularDim_);
  } else {
    auto base_transform = plant_.EvalBodyPoseInWorld(*plant_context_,
        plant_.GetBodyByName(base_));
    Quaterniond q_base = Quaterniond(base_transform.rotation() * frame_pose_.linear());
    base_orientation << q_base.w(), q_base.vec();
    MatrixXd J_spatial(6, nv_);
    plant_.CalcJacobianSpatialVelocity(*plant_context_, JacobianWrtVariable::kV,
        plant_.GetBodyByName(base_).body_frame(), frame_pose_.translation(),
        world_frame_, world_frame_, &J_spatial);

    base_omega = J_spatial.block(
        0, 0, 3, J_spatial.cols()) * x.tail(nv_);
  }

  if (planar_) {
    lambda = mass_ *
        (prev_sol_base_traj_.derivative(2).value(t) -
        MakePlanarVectorFrom3d(gravity_));
  } else {
    lambda = mass_ * (prev_sol_base_traj_.derivative(2).value(t) -
        gravity_);
  }


  VectorXd x_centroidal_inflated(nxi_);

  if (planar_) {
    x_centroidal_inflated << MakePlanarVectorFrom3d(com_pos), base_orientation,
    MakePlanarVectorFrom3d(com_vel), base_omega, MakePlanarVectorFrom3d(left_pos),
    MakePlanarVectorFrom3d(right_pos), lambda;
  } else {
    x_centroidal_inflated << com_pos, base_orientation, com_vel, base_omega,
        left_pos, right_pos, lambda;
  }
  return x_centroidal_inflated;
}

void KoopmanMPC::CalcOptimalMotionPlan(const drake::systems::Context<double> &context,
                                       dairlib::lcmt_saved_traj *traj_msg) const {
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);

  VectorXd q = robot_output->GetPositions();
  VectorXd v = robot_output->GetVelocities();
  VectorXd x(nq_+nv_);
  x << q, v;

  double timestamp = robot_output->get_timestamp();
  if (use_fsm_) {
    int fsm_state = (int) context.get_discrete_state(current_fsm_state_idx_).get_value()(0);
    double last_event_time = context.get_discrete_state(prev_event_time_idx_).get_value()(0);

    double time_since_last_event = timestamp - last_event_time;

    UpdateInitialStateConstraint(CalcCentroidalStateFromPlant(x, dt_),
        fsm_state, time_since_last_event);
  } else {
    UpdateInitialStateConstraint(
        CalcCentroidalStateFromPlant(x, timestamp), 0, 0);
  }

  const MathematicalProgramResult result = Solve(prog_);

  if (result.get_solution_result() == drake::solvers::kInfeasibleConstraints) {
    std::cout << "Infeasible problem! See infeasible constraints below:\n";
    for (auto name : result.GetInfeasibleConstraintNames(prog_)) {
      std::cout << name << std::endl;
    }
  }

  *traj_msg = MakeLcmTrajFromSol(result, timestamp, x);

}

EventStatus KoopmanMPC::DiscreteVariableUpdate(
    const drake::systems::Context<double> &context,
    drake::systems::DiscreteValues<double> *discrete_state) const {

  const BasicVector<double>* fsm_output =
      (BasicVector<double>*)this->EvalVectorInput(context, fsm_port_);
  VectorXd fsm_state = fsm_output->get_value();

  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  double timestamp = robot_output->get_timestamp();

  VectorXd xdes = this->EvalVectorInput(context, x_des_port_)->get_value();

  if (xdes != discrete_state->get_mutable_vector(x_des_idx_).get_mutable_value()) {
    UpdateTrackingObjective(xdes);
    discrete_state->get_mutable_vector(x_des_idx_).get_mutable_value() << xdes;
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

void KoopmanMPC::UpdateTrackingObjective(const VectorXd& xdes) const {
  for (auto & cost : tracking_cost_) {
    cost->UpdateCoefficients(Q_, -2*Q_*xdes, xdes.transpose() * xdes);
  }
}

void KoopmanMPC::UpdateInitialStateConstraint(const VectorXd& x0,
    const int fsm_state, const double t_since_last_switch) const {

  if (!use_fsm_) {
    modes_.front().init_state_constraint_.front()->UpdateCoefficients(
        MatrixXd::Identity(nz_, nz_), modes_.front().dynamics.x_basis_func(x0));
    return;
  }

  // Remove current initial state constraint
  modes_.at(x0_idx_[0]).init_state_constraint_.at(x0_idx_[1])->
    UpdateCoefficients(MatrixXd::Zero(1, nz_), VectorXd::Zero(1));

  // Re-create current knot constraint if necessary.
  // Otherwise re-create dynamics constraint
  if (x0_idx_[1] == 0) {
    MatrixXd Aeq = MatrixXd::Identity(nz_, 2 * nz_);
    Aeq.block(0, nz_, nz_, nz_) = -MatrixXd::Identity(nz_, nz_);
    state_knot_constraints_.at(x0_idx_[0])->UpdateCoefficients(Aeq, VectorXd::Zero(nz_));
  } else {
    MatrixXd dyn = MatrixXd::Zero(nz_, 2 * nz_ + nu_c_);
    dyn.block(0, 0, nz_, nz_) = modes_.at(x0_idx_[0]).dynamics.A;
    dyn.block(0, nz_, nz_, nu_c_) = modes_.at(x0_idx_[0]).dynamics.B;
    dyn.block(0, nz_, nz_ + nu_c_, nz_) = - MatrixXd::Identity(nz_, nz_);
    modes_.at(x0_idx_[0]).dynamics_constraints.at(x0_idx_[1]-1)->
    UpdateCoefficients(dyn, -modes_.at(x0_idx_[0]).dynamics.b);
  }


  /// TODO (@Brian-Acosta) - Add check to be able to start controller after sim
  x0_idx_[0] = fsm_state;
  x0_idx_[1] = std::floor(t_since_last_switch / dt_);


  if (x0_idx_[1] == modes_.at(x0_idx_[0]).N) {
    x0_idx_[1] = 0;
    x0_idx_[0] += 1;
    std::cout << "Shouldn't be here!" << std::endl;
    if (x0_idx_[0] == nmodes_) {
      x0_idx_[0] = 0;
    }
  }

  // Add new initial state constraint
  modes_.at(x0_idx_[0]).init_state_constraint_.at(x0_idx_[1])->
      UpdateCoefficients(MatrixXd::Identity(nz_, nz_),
          modes_.at(x0_idx_[0]).dynamics.x_basis_func(x0));

  // remove one constraint to break circular dependency
  if (x0_idx_[1] == 0) {
    state_knot_constraints_.at(x0_idx_[0])->UpdateCoefficients(
        MatrixXd::Zero(1, 2 * nz_), VectorXd::Zero(1));
  } else {
    modes_.at(x0_idx_[0]).dynamics_constraints.at(x0_idx_[1]-1)->
        UpdateCoefficients(MatrixXd::Zero(1, 2*nz_ + nu_c_), VectorXd::Zero(1));
  }

}

double KoopmanMPC::CalcCentroidalMassFromListOfBodies(std::vector<std::string> bodies) {
  double mass = 0;
  for (auto & name : bodies) {
    mass += plant_.GetBodyByName(name).get_mass(*plant_context_);
  }
  return mass;
}

/// TODO(@Brian-Acosta) Update this function to not assume planar for angular
/// traj and to not assume uniform N
lcmt_saved_traj KoopmanMPC::MakeLcmTrajFromSol(const drake::solvers::MathematicalProgramResult& result,
                                               double time,
                                               const VectorXd& state) const {
  DRAKE_ASSERT(result.is_success());

  LcmTrajectory::Trajectory CoMTraj;
  LcmTrajectory::Trajectory AngularTraj;
  LcmTrajectory::Trajectory SwingFootTraj;

  CoMTraj.traj_name = "com_traj";
  AngularTraj.traj_name = "orientation";
  SwingFootTraj.traj_name = "swing_foot_traj";


  for (int i = 0; i < 2*kLinearDim_; i++) {
    CoMTraj.datatypes.push_back("double");
    SwingFootTraj.datatypes.push_back("double");
  }

  for (int i = 0; i < 2*kAngularDim_; i++) {
    AngularTraj.datatypes.push_back("double");
  }

  MatrixXd x = MatrixXd::Zero(nx_ ,  nmodes_ * modes_.front().N);
  MatrixXd foot_traj = MatrixXd::Zero(kLinearDim_,
      modes_.at(x0_idx_[0]).N + 1 - x0_idx_[1]);


  VectorXd x_time_knots = VectorXd::Zero(nmodes_ * modes_.front().N);

  int idx_x0 = x0_idx_[0] * modes_.front().N + x0_idx_[1];

  for (int i = 0; i < nmodes_; i++) {
    auto mode = modes_.at(i);

    for (int j = 0; j < mode.N; j++) {
      int idx = i * mode.N + j;

      int col = (idx  + idx_x0 < total_knots_) ?
          idx + idx_x0 : idx + idx_x0 - total_knots_;

      x.block(0, col, nx_, 1) =
          result.GetSolution(modes_.at(i).zz.at(j).head(nx_));
      x_time_knots(col) = time + dt_ * col;
    }
  }

  MatrixXd x_com_knots(2*kLinearDim_, x.cols());
  x_com_knots << x.block(0, 0, kLinearDim_, x.cols()),
                 x.block(kLinearDim_ + kAngularDim_, 0, kLinearDim_, x.cols());
  CoMTraj.time_vector = x_time_knots;
  CoMTraj.datapoints = x_com_knots;

  MatrixXd orientation_knots(kAngularDim_ + (planar_ ? kAngularDim_ : 0), x.cols());
  if (planar_) {
    orientation_knots << x.block(kLinearDim_, 0, kAngularDim_, x.cols()),
                         x.block(kLinearDim_ * 2 + kAngularDim_, 0, kAngularDim_, x.cols());
  } else {
    /// TODO (@Brian-Acosta) add quaternion derivative
    orientation_knots << x.block(kLinearDim_, 0, kAngularDim_, x.cols());
  }

  AngularTraj.time_vector = x_time_knots;
  AngularTraj.datapoints = orientation_knots;

  Vector2d swing_ft_traj_breaks = {time,
                                   time + dt_ * (modes_.at(x0_idx_[0]).N + 1 - x0_idx_[1])};
  MatrixXd swing_ft_traj_knots = CalcSwingFootKnotPoints(state, result);

  SwingFootTraj.time_vector = swing_ft_traj_breaks;
  SwingFootTraj.datapoints = swing_ft_traj_knots;

  LcmTrajectory lcm_traj;

  lcm_traj.AddTrajectory(CoMTraj.traj_name, CoMTraj);
  lcm_traj.AddTrajectory(AngularTraj.traj_name, AngularTraj);
  lcm_traj.AddTrajectory(SwingFootTraj.traj_name, SwingFootTraj);

  return lcm_traj.GetLcmTraj();
}

MatrixXd KoopmanMPC::CalcSwingFootKnotPoints(const VectorXd& x,
    const MathematicalProgramResult& result) const {
  auto curr_mode = modes_.at(x0_idx_[0]);
  auto next_mode = modes_.at(nmodes_ % (x0_idx_[0] + 1));

  VectorXd swing_ft_curr_loc = VectorXd::Zero(kLinearDim_);
  VectorXd swing_ft_next_loc = VectorXd::Zero(kLinearDim_);
  VectorXd swing_ft_curr_vel = VectorXd::Zero(kLinearDim_);

  Vector3d swing_ft_plant;
  MatrixXd J_swing_vel = MatrixXd::Zero(3, nv_);

  plant_.SetPositionsAndVelocities(plant_context_, x);
  plant_.CalcPointsPositions(*plant_context_,
                             contact_points_.at(1 - curr_mode.stance).first,
                             contact_points_.at(1 - curr_mode.stance).second, world_frame_,
                             &swing_ft_plant);

  plant_.CalcJacobianTranslationalVelocity(*plant_context_,
                                           JacobianWrtVariable::kV, contact_points_.at(1-curr_mode.stance).first,
                                           VectorXd::Zero(3), world_frame_, world_frame_, &J_swing_vel);

  Vector3d swing_vel_plant = J_swing_vel * x.tail(nv_);

  if (planar_) {
    swing_ft_curr_loc(0) = swing_ft_plant(saggital_idx_);
    swing_ft_curr_loc(1) = swing_ft_plant(vertical_idx_);
    swing_ft_curr_vel(0) = swing_vel_plant(saggital_idx_);
    swing_ft_curr_vel(1) = swing_vel_plant(vertical_idx_);
  } else {
    swing_ft_curr_loc = swing_ft_plant;
    swing_ft_curr_vel = swing_ft_plant;
  }

  swing_ft_next_loc = result.GetSolution(
      next_mode.zz.at(0).segment(
          nx_ + kLinearDim_ * next_mode.stance, kLinearDim_));

  MatrixXd swing_ft_traj = MatrixXd::Zero(2*kLinearDim_, 2);

  swing_ft_traj.block(0, 0, kLinearDim_, 1) = swing_ft_curr_loc;
  swing_ft_traj.block(0,1, kLinearDim_, 1) = swing_ft_next_loc;
  swing_ft_traj.block(kLinearDim_, 0, kLinearDim_, 1) = swing_ft_curr_vel;
  swing_ft_traj.block(kLinearDim_, 1, kLinearDim_, 1) = VectorXd::Zero(kLinearDim_);

  return swing_ft_traj;
}

void KoopmanMPC::LoadDiscreteDynamicsFromFolder(std::string folder, double dt,
    EigenPtr<MatrixXd> Al, EigenPtr<MatrixXd> Bl, EigenPtr<MatrixXd> bl,
    EigenPtr<MatrixXd> Ar, EigenPtr<MatrixXd> Br, EigenPtr<MatrixXd> br) {

  MatrixXd Alc = readCSV(folder + "/Al.csv");
  MatrixXd Blc = readCSV(folder + "/Bl.csv");
  MatrixXd blc = readCSV(folder + "/bl.csv");

  MatrixXd Arc = readCSV(folder + "/Ar.csv");
  MatrixXd Brc = readCSV(folder + "/Br.csv");
  MatrixXd brc = readCSV(folder + "/br.csv");

  *Al << (MatrixXd::Identity(Alc.rows(), Alc.cols()) + Alc * dt);
  *Bl << dt * Blc;
  *bl << dt * bl;

  *Ar << (MatrixXd::Identity(Arc.rows(), Arc.cols()) + Arc * dt);
  *Br << dt * Brc;
  *br << dt * br;
}

Vector2d KoopmanMPC::MakePlanarVectorFrom3d(Vector3d vec) const {
  return Vector2d(vec(saggital_idx_), vec(vertical_idx_));
}

} // dairlib