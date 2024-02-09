#include "inverse_dynamics_qp.h"

namespace dairlib {
namespace controllers {

using std::string;
using std::vector;
using std::unique_ptr;

using Eigen::MatrixXd;
using Eigen::VectorXd;

using multibody::KinematicEvaluator;
using multibody::KinematicEvaluatorSet;
using multibody::WorldPointEvaluator;

using drake::systems::Context;
using drake::multibody::MultibodyPlant;
using drake::solvers::MathematicalProgram;
using drake::solvers::VariableRefList;


// Some helpers for cost updating operations
namespace {
void AddIDQPCost(
    const std::string& name, const Eigen::MatrixXd& Q,
    const Eigen::VectorXd& b, const drake::solvers::VariableRefList& vars,
    CostMap& cost_dest, MathematicalProgram& prog) {
  DRAKE_DEMAND(cost_dest.count(name) == 0);
  cost_dest.insert(
      {name, prog.AddQuadraticCost(Q, b, vars).evaluator()}
  );
}
}

InverseDynamicsQp::InverseDynamicsQp(
    const MultibodyPlant<double>& plant, Context<double>* context) :
    plant_(plant),
    context_(context),
    holonomic_constraints_(plant),
    nv_(plant.num_velocities()),
    nu_(plant.num_actuated_dofs()) {}


void InverseDynamicsQp::AddHolonomicConstraint(
    const string& name, unique_ptr<const KinematicEvaluator<double>> eval) {
  DRAKE_DEMAND(holonomic_constraint_evaluators_.count(name) == 0);
  DRAKE_DEMAND(&eval->plant() == &plant_);

  holonomic_constraint_evaluators_.insert({name, std::move(eval)});
  holonomic_constraints_.add_evaluator(
      holonomic_constraint_evaluators_.at(name).get()
  );

  nh_ += holonomic_constraint_evaluators_.at(name)->num_active();
}

void InverseDynamicsQp::AddContactConstraint(
    const string& name, unique_ptr<const WorldPointEvaluator<double>> eval) {
  DRAKE_DEMAND(contact_constraint_evaluators_.count(name) == 0);
  DRAKE_DEMAND(&eval->plant() == &plant_);

  contact_constraint_evaluators_.insert({name, std::move(eval)});
  nc_ += contact_constraint_evaluators_.at(name)->num_full();
  nc_active_ += contact_constraint_evaluators_.at(name)->num_active();
}

void InverseDynamicsQp::AddExternalForce(
    const string &name, unique_ptr<const KinematicEvaluator<double>> eval) {
  DRAKE_DEMAND(external_force_evaluators_.count(name) == 0);
  DRAKE_DEMAND(&eval->plant() == &plant_);

  external_force_evaluators_.insert({name, std::move(eval)});
  ne_ += external_force_evaluators_.at(name)->num_full();
}

void InverseDynamicsQp::Build() {
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

  built_ = true;
}



void InverseDynamicsQp::AddAccelerationCost(
    const string &name, const MatrixXd &Q, const VectorXd &b,
    const VariableRefList& vars) {
  AddIDQPCost(name, Q, b, vars, dv_costs_, prog_);
}

void InverseDynamicsQp::AddInputCost(
    const string &name, const MatrixXd &Q, const VectorXd &b,
    const VariableRefList& vars) {
  AddIDQPCost(name, Q, b, vars, u_costs_, prog_);
}

void InverseDynamicsQp::AddContactForceCost(
    const string &name, const MatrixXd &Q, const VectorXd &b,
    const VariableRefList& vars) {
  AddIDQPCost(name, Q, b, vars, lambda_c_costs_, prog_);
}

void InverseDynamicsQp::AddExternalForceCost(
    const string &name, const MatrixXd &Q, const VectorXd &b,
    const VariableRefList& vars) {
  AddIDQPCost(name, Q, b, vars, lambda_e_costs_, prog_);
}

void InverseDynamicsQp::UpdateAccelerationCost(
    const string &name, const MatrixXd &Q, const VectorXd &b, double c) {
  dv_costs_.at(name)->UpdateCoefficients(Q, b, c, true);
}

void InverseDynamicsQp::UpdateInputCost(
    const string &name, const MatrixXd &Q, const VectorXd &b, double c) {
  u_costs_.at(name)->UpdateCoefficients(Q, b, c, true);
}

void InverseDynamicsQp::UpdateContactForceCost(
    const string &name, const MatrixXd &Q, const VectorXd &b, double c) {
  lambda_c_costs_.at(name)->UpdateCoefficients(Q, b, c, true);
}

void InverseDynamicsQp::UpdateExternalForceCost(
    const string &name, const MatrixXd &Q, const VectorXd &b, double c) {
  lambda_e_costs_.at(name)->UpdateCoefficients(Q, b, c, true);
}

}
}