//
// Created by brian on 3/8/21.
//

#include "koopman_mpc.h"

using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using dairlib::multibody::WorldPointEvaluator;

namespace dairlib::systems::controllers{

KoopmanMPC::KoopmanMPC(const MultibodyPlant<double>& plant,
                       const Context<double> *plant_context, double dt,
                       bool planar, bool used_with_finite_state_machine) :
                       plant_(plant),
                       plant_context_(plant_context), dt_(dt){

  nq_ = plant.num_positions();
  nv_ = plant.num_velocities();
  nu_p_ = plant.num_actuators();

  nx_ = planar ? kNxPlanar : kNx3d;
  nu_c_ = planar ? kNuPlanar : kNu3d;
  kLinearDim_ = planar ? 2 : 3;
  kAngularDim_ = planar ? 1 : 4;

  nxi_ = nx_ + nu_c_;

  // Create Ports
  state_port_ = this->DeclareVectorInputPort(
      OutputVector<double>(nq_, nv_, nu_p_)).get_index();

  x_des_port_ = this->DeclareVectorInputPort(
      BasicVector<double>(nx_)).get_index();

  if ( use_fsm_ ) {
    fsm_port_ = this->DeclareVectorInputPort(
        BasicVector<double>(2)).get_index();
  }

  this->DeclareAbstractOutputPort(&KoopmanMPC::CalcOptimalMotionPlan);

}

void KoopmanMPC::AddMode(const KoopmanDynamics& dynamics, koopMpcStance stance, int N) {
  if (modes_.empty()) {
    nz_ = dynamics.x_basis_func(VectorXd::Zero(nxi_)).size();
  }

  DRAKE_DEMAND(dynamics.x_basis_func(VectorXd::Zero(nxi_)).size() == nz_);

  KoopmanMpcMode mode;
  mode.dynamics = dynamics;
  mode.stance = stance;
  mode.N = N;

  for ( int i = 0; i < N+1; i++) {
    mode.zz.push_back(prog_.NewContinuousVariables(nz_, "z_" + std::to_string(i)));
  }
  for (int i = 0; i < N; i++) {
    mode.uu.push_back(prog_.NewContinuousVariables(nu_c_, "u_" + std::to_string(i+1)));
  }
  modes_.push_back(mode);
}

void KoopmanMPC::AddContactPoint(const WorldPointEvaluator<double>* evaluator,
    koopMpcStance stance) {
  DRAKE_ASSERT(contact_points_.size() == stance)
  contact_points_.push_back(evaluator);
}

void KoopmanMPC::SetReachabilityLimit(const Eigen::MatrixXd &kl,
    const std::vector<Eigen::VectorXd> &kn) {
  DRAKE_DEMAND(kl.size() == kLinearDim_);

  kin_lim_ = kl;
  for (auto pos : kn) {
    kin_nominal_.push_back(pos);
  }
}

void KoopmanMPC::BuildController() {
  DRAKE_DEMAND(!modes_.empty());
  DRAKE_DEMAND(!tracking_cost_.empty());
  DRAKE_DEMAND(!input_cost_.empty());
  DRAKE_DEMAND(mu_ > 0 );

  MakeDynamicsConstraints();
  MakeFrictionConeConstraints();
  MakeStanceFootConstraints();
  MakeKinematicReachabilityConstraints();
  MakeStateKnotConstraints();
  MakeInitialStateConstraint();
}

void KoopmanMPC::MakeStanceFootConstraints() {
  MatrixXd S = MatrixXd::Identity(kLinearDim_, kLinearDim_);
  for (auto & mode : modes_) {
    for (int i = 0; i < mode.N; i++) {
      mode.stance_foot_constraints.push_back(
          prog_.AddLinearEqualityConstraint(
              S, VectorXd::Zero(kLinearDim_),
              mode.uu.at(i).segment(mode.stance * kLinearDim_, kLinearDim_ ))
              .evaluator()
              .get());
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
    }
  }
}

/// TODO(@Brian-Acosta): Update initial state constraints to allow for
/// any xx to be x0
void KoopmanMPC::MakeInitialStateConstraint() {
  initial_state_constraint_ = prog_.AddLinearEqualityConstraint(
      MatrixXd::Identity(nz_, nz_), VectorXd::Zero(nz_),
      modes_.front().zz.front())
       .evaluator()
       .get();


}

void KoopmanMPC::MakeKinematicReachabilityConstraints() {
  MatrixXd S = MatrixXd::Zero(kLinearDim_, kLinearDim_ * 2);

  S.block(0, 0, kLinearDim_, kLinearDim_) =
      -MatrixXd::Identity(kLinearDim_, kLinearDim_);
  S.block(0, kLinearDim_, kLinearDim_, kLinearDim_) =
      MatrixXd::Identity(kLinearDim_, kLinearDim_);

  for (auto & mode : modes_) {
    for (int i = 0; i < mode.N; i++) {
      mode.reachability_constraints.push_back(
          prog_.AddLinearConstraint(S,
              -kin_lim_ + kin_nominal_.at(mode.stance),
              kin_lim_ + kin_nominal_.at(mode.stance),
              {mode.zz.at(i).head(kLinearDim_),
               mode.zz.at(i).segment(nx_ + kLinearDim_ * mode.stance,
                   kLinearDim_)})
               .evaluator()
               .get());
    }
  }
}

void KoopmanMPC::MakeFrictionConeConstraints() {
  for (auto & mode : modes_) {
    for (int i = 0; i <= mode.N; i++) {
      if (kLinearDim_ == 3) {
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
    }
  }
}

void KoopmanMPC::MakeStateKnotConstraints() {
  MatrixXd Aeq = MatrixXd::Identity(nz_, 2 * nz_);
  Aeq.block(0, nz_, nz_, nz_) = -MatrixXd::Identity(nz_, nz_);
  Aeq.block(0, nz_, nz_, nz_) = -MatrixXd::Identity(nz_, nz_);
  for (int i = 0; i < modes_.size() - 1; i++) {
    state_knot_constraints_.push_back(
        prog_.AddLinearEqualityConstraint( Aeq, VectorXd::Zero(nz_),
            {modes_.at(i).zz.back(), modes_.at(i+1).zz.front()})
            .evaluator()
            .get());
  }

  // Dummy constraint to be used when cycling modes
  state_knot_constraints_.push_back(
      prog_.AddLinearEqualityConstraint( MatrixXd::Zero(1, 2*nz_),
          VectorXd::Zero(1),
          {modes_.back().zz.back(), modes_.front().zz.front()})
          .evaluator()
          .get());
}

void KoopmanMPC::AddTrackingObjective(const Eigen::VectorXd &xdes,
                                      const Eigen::MatrixXd &Q) {
  for (auto & mode : modes_) {
    for (int i = 0; i < mode.N+1; i++ ) {
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
  for(auto & mode : modes_) {
    for (int i = 0; i < mode.N; i++) {
      input_cost_.push_back(
          prog_.AddQuadraticCost(R, VectorXd::Zero(nu_c_), mode.uu.at(i))
          .evaluator()
          .get());
    }
  }
}


VectorXd KoopmanMPC::CalcCentroidalStateFromPlant(Eigen::VectorXd x,
    double t) {

}



} // dairlib::systems::controllers