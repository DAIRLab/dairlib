//
// Created by brian on 3/8/21.
//

#include "koopman_mpc.h"

using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using Eigen::VectorXd;
using Eigen::MatrixXd;

namespace dairlib::systems::controllers{

KoopmanMPC::KoopmanMPC(const MultibodyPlant<double>& plant,
                       const Context<double> *plant_context, double dt,
                       bool planar, bool used_with_finite_state_machine) :
                       plant_(plant),
                       plant_context_(plant_context), dt_(dt){

  nx_ = planar ? kNxPlanar : kNx3d;
  nu_ = planar ? kNuPlanar : kNu3d;
  kLinearDim_ = planar ? 2 : 3;
  kAngularDim_ = planar ? 1 : 4;

  nxi_ = nx_ + nu_;

  // Create Ports
  state_port_ = this->DeclareVectorInputPort(
      BasicVector<double>(nx_)).get_index();

   x_des_port_ = this->DeclareVectorInputPort(
      BasicVector<double>(nx_)).get_index();

  if ( use_fsm_ ) {
    fsm_port_ = this->DeclareVectorInputPort(
        BasicVector<double>(2)).get_index();
  }
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
    mode.uu.push_back(prog_.NewContinuousVariables(nu_, "u_" + std::to_string(i+1)));
  }
  modes_.push_back(mode);
}

void KoopmanMPC::SetReachabilityLimit(const Eigen::MatrixXd &kl, const std::vector<Eigen::VectorXd> &kn) {
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
              mode.uu.at(i).segment(mode.stance * kLinearDim_, kLinearDim_ ) == VectorXd::Zero(kLinearDim_))
              .evaluator()
              .get());
    }
  }
}

void KoopmanMPC::MakeDynamicsConstraints() {
  for (auto & mode : modes_) {
    for (int i = 0; i < mode.N; i++) {
      mode.dynamics_constraints.push_back(
          prog_.AddLinearEqualityConstraint(
              mode.zz.at(i+1) == mode.dynamics.A * mode.zz.at(i) + mode.dynamics.B * mode.uu.at(i) + mode.dynamics.b)
              .evaluator()
              .get());
    }
  }
}

void KoopmanMPC::MakeInitialStateConstraint() {
  initial_state_constraint_ = prog_.AddLinearEqualityConstraint(
      modes_.front().zz.front() == modes_.front().dynamics.x_basis_func(VectorXd::Zero(nxi_)))
       .evaluator().get();
}

void KoopmanMPC::MakeKinematicReachabilityConstraints() {
  for (auto & mode : modes_) {
    for (int i = 0; i < mode.N; i++) {
      
    }
  }
}
}