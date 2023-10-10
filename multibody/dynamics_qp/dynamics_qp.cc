#include "dynamics_qp.h"
#include "solvers/constraint_factory.h"

namespace dairlib {
namespace multibody {

using Eigen::VectorXd;
using Eigen::MatrixXd;

using drake::multibody::JointActuatorIndex;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;

// solver
using drake::solvers::MathematicalProgram;
using drake::solvers::LinearConstraint;
using drake::solvers::LinearEqualityConstraint;

DynamicsQP::DynamicsQP(const MultibodyPlant<double> &plant) :
    nq_(plant.num_positions()), nv_(plant.num_velocities()),
    nu_(plant.num_velocities()), plant_(plant) {}

void DynamicsQP::Build() {
  prog_ = std::make_unique<MathematicalProgram>();
  u_ = prog_->NewContinuousVariables(nu_);
  dv_ = prog_->NewContinuousVariables(nv_);
  lambda_c_ = prog_->NewContinuousVariables(nc_);
  lambda_h_ = prog_->NewContinuousVariables(nh_);
  epsilon_c_ = prog_->NewContinuousVariables(nc_soft_);
  MakeDynamicsConstraint();
  MakeContactConstraints();
  MakeInputBoundsConstraint();
}

void DynamicsQP::MakeDynamicsConstraint() {
  dynamics_constraint_ = prog_->AddLinearEqualityConstraint(
      MatrixXd::Zero(nv_, nv_ + nc_ + nh_ + nu_),
      VectorXd::Zero(nv_),
      {dv_, lambda_c_, lambda_h_, u_}
  ).evaluator();
}

void DynamicsQP::MakeContactConstraints() {
  // Make Contact and Friction Cone Constraints
  for (auto& entry: contacts_) {
    // get the constraint struct
    auto& constraint = entry.second;
    DRAKE_DEMAND(constraint.evaluator_ != nullptr);

    // dimensions of this constraint
    int nc_active = constraint.nc_active_;
    int n_eps = constraint.n_eps_;

    // get the decision variables
    const auto& lambda_c = lambda_c_.segment<3>(constraint.idx_lambda_start_);
    const auto& eps_c = epsilon_c_.segment(constraint.idx_eps_start_, n_eps);

    // contact constraint
    constraint.contact_constraint_ = prog_->AddLinearEqualityConstraint(
        MatrixXd::Zero(nc_active, nv_ + n_eps),
        VectorXd::Zero(nc_active),
        {dv_, eps_c}
    ).evaluator();

    // friction cone constraint
    auto friction_cone = solvers::CreateLinearFrictionConstraint(
        constraint.mu_, 8, 2, true
    );
    constraint.friction_cone_ = prog_->AddLinearConstraint(
        friction_cone->GetDenseA(),
        friction_cone->lower_bound(),
        friction_cone->upper_bound(),
        lambda_c
    ).evaluator();

    // soft constraint cost
    if (constraint.soft_constraint_) {
      constraint.soft_constraint_cost_ = prog_->AddQuadraticCost(
          constraint.W_soft_constraint_, VectorXd::Zero(n_eps), eps_c, true
      ).evaluator();
    }
  }
}

void DynamicsQP::MakeKinematicConstraints() {
  holonomic_constraint_ = prog_->AddLinearEqualityConstraint(
      MatrixXd::Zero(nh_, nv_), VectorXd::Zero(nv_), dv_
  ).evaluator();
}

void DynamicsQP::MakeInputBoundsConstraint() {
  DRAKE_DEMAND(prog_ != nullptr);

  VectorXd u_min(nu_);
  VectorXd u_max(nu_);
  for (JointActuatorIndex i(0); i < nu_; ++i) {
    u_min[i] = -plant_.get_joint_actuator(i).effort_limit();
    u_max[i] = plant_.get_joint_actuator(i).effort_limit();
  }
  input_bounds_ = prog_->AddBoundingBoxConstraint(u_min, u_max, u_).evaluator();
}

}
}