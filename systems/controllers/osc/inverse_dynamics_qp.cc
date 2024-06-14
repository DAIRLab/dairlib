#include "inverse_dynamics_qp.h"
#include "multibody/multibody_utils.h"

namespace dairlib {
namespace systems {
namespace controllers {

using std::string;
using std::vector;
using std::unique_ptr;

using Eigen::MatrixXd;
using Eigen::VectorXd;

using multibody::KinematicEvaluator;
using multibody::WorldPointEvaluator;
using multibody::KinematicEvaluatorSet;
using multibody::SetPositionsAndVelocitiesIfNew;

using drake::systems::Context;
using drake::multibody::MultibodyPlant;
using drake::solvers::VariableRefList;
using drake::solvers::MathematicalProgram;
using drake::solvers::VectorXDecisionVariable;

InverseDynamicsQp::InverseDynamicsQp(
    const MultibodyPlant<double> &plant, Context<double> *context) :
    plant_(plant),
    context_(context),
    nv_(plant.num_velocities()),
    nu_(plant.num_actuated_dofs()) {}

void InverseDynamicsQp::AddHolonomicConstraint(
    unique_ptr<const KinematicEvaluatorSet<double>> eval) {
  DRAKE_DEMAND(&eval->plant() == &plant_);
  DRAKE_DEMAND(not built_);
  DRAKE_DEMAND(holonomic_constraints_ == nullptr);
  DRAKE_DEMAND(nh_ == 0);

  holonomic_constraints_ = std::move(eval);
  nh_ = holonomic_constraints_->count_full();
}

void InverseDynamicsQp::AddContactConstraint(
    const string &name, unique_ptr<const WorldPointEvaluator<double>> eval,
    double friction_coefficient) {

  DRAKE_DEMAND(not built_);
  DRAKE_DEMAND(friction_coefficient >= 0);
  DRAKE_DEMAND(contact_constraint_evaluators_.count(name) == 0);
  DRAKE_DEMAND(&eval->plant() == &plant_);

  mu_map_.insert({name, friction_coefficient});
  lambda_c_start_.insert({name, nc_});
  Jc_active_start_.insert({name, nc_active_});
  contact_constraint_evaluators_.insert({name, std::move(eval)});

  nc_ += contact_constraint_evaluators_.at(name)->num_full();
  nc_active_ += contact_constraint_evaluators_.at(name)->num_active();
}

void InverseDynamicsQp::AddExternalForce(
    const string &name, unique_ptr<const KinematicEvaluator<double>> eval) {
  DRAKE_DEMAND(not built_);
  DRAKE_DEMAND(&eval->plant() == &plant_);
  DRAKE_DEMAND(external_force_evaluators_.count(name) == 0);

  external_force_evaluators_.insert({name, std::move(eval)});
  lambda_e_start_and_size_.insert(
      {name, {ne_, external_force_evaluators_.at(name)->num_full()}});
  ne_ += external_force_evaluators_.at(name)->num_full();
}

void InverseDynamicsQp::Build() {
  DRAKE_DEMAND(not built_);

  dv_ = prog_.NewContinuousVariables(nv_, "dv");
  u_ = prog_.NewContinuousVariables(nu_, "u");
  lambda_h_ = prog_.NewContinuousVariables(nh_, "lambda_holonomic");
  lambda_c_ = prog_.NewContinuousVariables(nc_, "lambda_contact");
  lambda_e_ = prog_.NewContinuousVariables(ne_, "lambda_external");
  epsilon_ = prog_.NewContinuousVariables(nc_active_, "soft_constraint_slack");

  dynamics_c_ = prog_.AddLinearEqualityConstraint(
      MatrixXd::Zero(nv_, nv_ + nu_ + nh_ + nc_ + ne_),
      VectorXd::Zero(nv_), {dv_, u_, lambda_h_, lambda_c_, lambda_e_}
  ).evaluator();

  holonomic_c_ = prog_.AddLinearEqualityConstraint(
      MatrixXd::Zero(nh_, nv_), VectorXd::Zero(nh_), dv_
  ).evaluator();

  contact_c_ = prog_.AddLinearEqualityConstraint(
      MatrixXd::Zero(nc_active_, nv_ + nc_active_),
      VectorXd::Zero(nc_active_), {dv_, epsilon_}
  ).evaluator();

  for (const auto& [cname, eval] : contact_constraint_evaluators_) {
    double mu = mu_map_.at(cname);
    MatrixXd A = MatrixXd(5, 3);
    A << -1, 0, mu, 0, -1, mu, 1, 0, mu, 0, 1, mu, 0, 0, 1;
    lambda_c_friction_cone_.insert({
      cname,
      prog_.AddLinearConstraint(
          A, VectorXd::Zero(5),
          VectorXd::Constant(5, std::numeric_limits<double>::infinity()),
          lambda_c_.segment(lambda_c_start_.at(cname), 3)).evaluator()
    });
  }

  VectorXd u_min(nu_);
  VectorXd u_max(nu_);
  for (drake::multibody::JointActuatorIndex i(0); i < nu_; ++i) {
    u_min[i] = -plant_.get_joint_actuator(i).effort_limit();
    u_max[i] = plant_.get_joint_actuator(i).effort_limit();
  }

  input_limit_c_ = prog_.AddBoundingBoxConstraint(u_min, u_max, u_).evaluator();

  built_ = true;
}


void InverseDynamicsQp::AddQuadraticCost(
    const string &name, const MatrixXd &Q, const VectorXd &b,
    const VariableRefList &vars) {
  DRAKE_DEMAND(built_);
  DRAKE_DEMAND(all_costs_.count(name) == 0);
  all_costs_.insert(
      {name, prog_.AddQuadraticCost(Q, b, vars).evaluator()}
  );
}

void InverseDynamicsQp::AddQuadraticCost(
    const string &name, const MatrixXd &Q, const VectorXd &b,
    const VectorXDecisionVariable &vars) {
  DRAKE_DEMAND(built_);
  DRAKE_DEMAND(all_costs_.count(name) == 0);
  all_costs_.insert(
      {name, prog_.AddQuadraticCost(Q, b, vars).evaluator()}
  );
}

void InverseDynamicsQp::UpdateDynamics(
    const VectorXd &x, const vector<string> &active_contact_constraints,
    const vector<string> &active_external_forces) {

  SetPositionsAndVelocitiesIfNew<double>(plant_, x, context_);

  MatrixXd M(nv_, nv_);
  VectorXd bias(nv_);
  MatrixXd B = plant_.MakeActuationMatrix();
  VectorXd grav = plant_.CalcGravityGeneralizedForces(*context_);

  plant_.CalcMassMatrix(*context_, &M);
  plant_.CalcBiasTerm(*context_, &bias);

  // TODO (@Brian-Acosta) add option to turn off gravity comp
  bias = bias - grav;

  MatrixXd Jh = MatrixXd::Zero(nh_, nv_);
  VectorXd Jh_dot_v = VectorXd::Zero(nh_);
  if (holonomic_constraints_ != nullptr) {
    Jh = holonomic_constraints_->EvalFullJacobian(*context_);
    Jh_dot_v = holonomic_constraints_->EvalFullJacobianDotTimesV(*context_);
  }

  MatrixXd Jc_active = MatrixXd::Zero(nc_active_, nv_);
  VectorXd Jc_active_dot_v = VectorXd::Zero(nc_active_);
  MatrixXd Jc = MatrixXd::Zero(nc_, nv_);
  MatrixXd Je = MatrixXd::Zero(ne_, nv_);

  for (const auto &c : active_contact_constraints) {
    DRAKE_DEMAND(contact_constraint_evaluators_.count(c) > 0);
    const auto &evaluator = contact_constraint_evaluators_.at(c);
    Jc.block(lambda_c_start_.at(c), 0, 3, nv_) =
        evaluator->EvalFullJacobian(*context_);
    int start = Jc_active_start_.at(c);
    for (int i = 0; i < evaluator->num_active(); ++i) {
      Jc_active.row(start + i) =
          Jc.row(lambda_c_start_.at(c) + evaluator->active_inds().at(i));
      Jc_active_dot_v.segment(start, evaluator->num_active()) =
          evaluator->EvalActiveJacobianDotTimesV(*context_);
    }
  }
  for (const auto &e : active_external_forces) {
    const auto &[start, size] = lambda_e_start_and_size_.at(e);
    Je.block(start, 0, size, nv_) =
        external_force_evaluators_.at(e)->EvalFullJacobian(*context_);
  }

  MatrixXd A_dyn = MatrixXd::Zero(nv_, nv_ + nu_ + nh_ + nc_ + ne_);
  A_dyn.block(0, 0, nv_, nv_) = M;
  A_dyn.block(0, nv_, nv_, nu_) = -B;
  A_dyn.block(0, nv_ + nu_, nv_, nh_) = -Jh.transpose();
  A_dyn.block(0, nv_ + nu_ + nh_, nv_, nc_) = -Jc.transpose();
  A_dyn.block(0, nv_ + nu_ + nh_ + nc_, nv_, ne_) = -Je.transpose();

  MatrixXd A_c = MatrixXd::Zero(nc_active_, nv_ + nc_active_);
  A_c.block(0, 0, nc_active_, nv_) = Jc_active;
  A_c.block(0, nv_, nc_active_, nc_active_) =
      MatrixXd::Identity(nc_active_, nc_active_);

  dynamics_c_->UpdateCoefficients(A_dyn, -bias);
  holonomic_c_->UpdateCoefficients(Jh, -Jh_dot_v);
  contact_c_->UpdateCoefficients(A_c, -Jc_active_dot_v);
}

}
}
}