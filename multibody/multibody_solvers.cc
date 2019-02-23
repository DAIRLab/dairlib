#include "multibody/multibody_solvers.h"

namespace dairlib {
namespace multibody {

using std::make_shared;
using std::make_unique;
using std::logic_error;
using std::shared_ptr;
using std::string;
using std::vector;

using drake::AutoDiffXd;
using drake::AutoDiffVecXd;
using drake::math::autoDiffToValueMatrix;
using drake::math::autoDiffToGradientMatrix;
using drake::math::DiscardGradient;
using drake::math::initializeAutoDiff;
using drake::solvers::Constraint;
using drake::solvers::MathematicalProgram;
using drake::solvers::SnoptSolver;
using drake::solvers::SolutionResult;
using drake::solvers::VectorXDecisionVariable;
using drake::symbolic::Variable;
using drake::symbolic::Expression;
using drake::VectorX;
using Eigen::VectorXd;
using Eigen::Matrix3Xd;
using Eigen::MatrixXd;

PositionConstraint::PositionConstraint(const RigidBodyTree<double>& tree,
                                       const string& description)
    : Constraint(tree.getNumPositionConstraints(), tree.get_num_positions(),
                 VectorXd::Zero(tree.getNumPositionConstraints()),
                 VectorXd::Zero(tree.getNumPositionConstraints()), description),
      tree_(tree) {}

void PositionConstraint::DoEval(const Eigen::Ref<const Eigen::VectorXd>& q,
                                Eigen::VectorXd* y) const {
  AutoDiffVecXd y_t;
  Eval(drake::math::initializeAutoDiff(q), &y_t);
  *y = autoDiffToValueMatrix(y_t);
}

void PositionConstraint::DoEval(const Eigen::Ref<const drake::AutoDiffVecXd>& q,
                                drake::AutoDiffVecXd* y) const {
  const AutoDiffVecXd q_t = q.head(tree_.get_num_positions());
  KinematicsCache<AutoDiffXd> k_cache = tree_.doKinematics(q_t);
  *y = tree_.positionConstraints(k_cache);
}

void PositionConstraint::DoEval(
    const Eigen::Ref<const drake::VectorX<drake::symbolic::Variable>>& x,
    drake::VectorX<drake::symbolic::Expression>* y) const {
  throw logic_error("PositionConstraint does not support symbolic evaluation.");
}

ContactConstraint::ContactConstraint(const RigidBodyTree<double>& tree,
                                     ContactInfo contact_info,
                                     const string& description)
    : Constraint(contact_info.num_contacts, tree.get_num_positions(),
                 VectorXd::Zero(contact_info.num_contacts),
                 VectorXd::Zero(contact_info.num_contacts), description),
      tree_(tree),
      contact_info_(contact_info),
      num_positions_(tree.get_num_positions()),
      num_velocities_(tree.get_num_velocities()),
      num_efforts_(tree.get_num_actuators()),
      num_position_forces_(tree.getNumPositionConstraints()),
      num_contacts_(contact_info.num_contacts),
      num_forces_(tree.getNumPositionConstraints() +
                  contact_info.num_contacts * 3) {
  contact_toolkit_ =
      make_unique<ContactToolkit<AutoDiffXd>>(tree, contact_info);
}

void ContactConstraint::DoEval(const Eigen::Ref<const Eigen::VectorXd>& q,
                               Eigen::VectorXd* y) const {
  AutoDiffVecXd y_t;
  Eval(initializeAutoDiff(q), &y_t);
  *y = autoDiffToValueMatrix(y_t);
}

void ContactConstraint::DoEval(const Eigen::Ref<const drake::AutoDiffVecXd>& q,
                               drake::AutoDiffVecXd* y) const {
  // Verifying the size of the input vector
  DRAKE_DEMAND(q.size() == num_positions_);

  const AutoDiffVecXd q_t = q.head(tree_.get_num_positions());

  // Kinematics Cache
  KinematicsCache<AutoDiffXd> k_cache_autodiff = tree_.doKinematics(q_t);
  KinematicsCache<double> k_cache_double =
      tree_.doKinematics(DiscardGradient(q_t));

  AutoDiffVecXd y_t = initializeAutoDiff(VectorXd::Zero(num_contacts_));

  for (int i = 0; i < num_contacts_; ++i) {
    // Transforming the point on the body to the world frame.
    AutoDiffVecXd contact_pt_A = tree_.transformPoints(
        k_cache_autodiff, contact_info_.xA.col(i), contact_info_.idxA.at(i), 0);
    // The constraint is that the point must lie on the z-plane.
    y_t(i) = contact_pt_A(2);
  }

  *y = y_t;
}

void ContactConstraint::DoEval(
    const Eigen::Ref<const drake::VectorX<drake::symbolic::Variable>>& q,
    drake::VectorX<drake::symbolic::Expression>* y) const {
  throw logic_error(
      "FixedPointConstraint does not support symbolic evaluation.");
}

FixedPointConstraint::FixedPointConstraint(const RigidBodyTree<double>& tree,
                                           ContactInfo contact_info,
                                           const string& description)
    : Constraint(tree.get_num_velocities(),
                 tree.get_num_positions() + tree.get_num_actuators() +
                     tree.getNumPositionConstraints() +
                     contact_info.num_contacts * 3,
                 VectorXd::Zero(tree.get_num_velocities()),
                 VectorXd::Zero(tree.get_num_velocities()), description),
      tree_(tree),
      contact_info_(contact_info),
      num_positions_(tree.get_num_positions()),
      num_velocities_(tree.get_num_velocities()),
      num_efforts_(tree.get_num_actuators()),
      num_position_forces_(tree.getNumPositionConstraints()),
      num_forces_(tree.getNumPositionConstraints() +
                  contact_info.num_contacts * 3) {
  contact_toolkit_ =
      make_unique<ContactToolkit<AutoDiffXd>>(tree, contact_info);
}

void FixedPointConstraint::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& q_u_l, Eigen::VectorXd* y) const {
  AutoDiffVecXd y_t;
  Eval(initializeAutoDiff(q_u_l), &y_t);
  *y = autoDiffToValueMatrix(y_t);
}

void FixedPointConstraint::DoEval(
    const Eigen::Ref<const drake::AutoDiffVecXd>& q_u_l,
    drake::AutoDiffVecXd* y) const {
  // Verifying the size of the input vector
  DRAKE_DEMAND(q_u_l.size() == num_positions_ + num_efforts_ + num_forces_);

  // Extracting the components
  const AutoDiffVecXd q = q_u_l.head(num_positions_);
  const AutoDiffVecXd u = q_u_l.segment(num_positions_, num_efforts_);
  const AutoDiffVecXd lambda = q_u_l.tail(num_forces_);

  const AutoDiffVecXd v =
      VectorXd::Zero(num_velocities_).template cast<AutoDiffXd>();
  AutoDiffVecXd x(num_positions_ + num_velocities_);
  x << q, v;

  *y = contact_toolkit_->CalcMVDot(x, u, lambda);
  std::cout << y->transpose() << std::endl << "------------------" << std::endl;
}

void FixedPointConstraint::DoEval(
    const Eigen::Ref<const drake::VectorX<drake::symbolic::Variable>>& q_u_l,
    drake::VectorX<drake::symbolic::Expression>* y) const {
  throw logic_error(
      "FixedPointConstraint does not support symbolic evaluation.");
}

PositionSolver::PositionSolver(const RigidBodyTree<double>& tree)
    : tree_(tree), prog_(make_shared<MathematicalProgram>()) {
  // Initializing the variable
  q_ = prog_->NewContinuousVariables(tree_.get_num_positions(), "q");
}

void PositionSolver::SetInitialGuessQ(VectorXd q) {
  prog_->SetInitialGuess(q_, q);
}

SolutionResult PositionSolver::Solve(VectorXd q, vector<int> fixed_joints) {
  // Setting the solver options
  prog_->SetSolverOption(SnoptSolver::id(), "Log file", filename_);
  prog_->SetSolverOption(SnoptSolver::id(), "Major feasibility tolerance",
                         major_tolerance_);
  prog_->SetSolverOption(SnoptSolver::id(), "Minor feasibility tolerance",
                         minor_tolerance_);

  auto position_constraint = make_shared<PositionConstraint>(tree_);

  prog_->AddConstraint(position_constraint, q_);

  // Adding the fixed joint constraints
  for (uint i = 0; i < fixed_joints.size(); i++) {
    int ind = fixed_joints[i];
    prog_->AddConstraint(q_(ind) == q(ind));
  }

  prog_->AddQuadraticCost((q_ - q).dot(q_ - q));

  // The initial guess for q needs to be set up separately before calling Solve
  solution_result_ = prog_->Solve();

  return solution_result_;
}

bool PositionSolver::CheckConstraint(VectorXd q, double tolerance) const {
  auto position_constraint = make_shared<PositionConstraint>(tree_);
  return position_constraint->CheckSatisfied(q, tolerance);
}

shared_ptr<MathematicalProgram> PositionSolver::get_program() { return prog_; }

SolutionResult PositionSolver::get_solution_result() {
  return solution_result_;
}

VectorXd PositionSolver::GetSolutionQ() { return prog_->GetSolution(q_); }

void PositionSolver::set_filename(string filename) { filename_ = filename; }

void PositionSolver::set_major_tolerance(double major_tolerance) {
  major_tolerance_ = major_tolerance;
}
void PositionSolver::set_minor_tolerance(double minor_tolerance) {
  minor_tolerance_ = minor_tolerance;
}

string PositionSolver::get_filename() { return filename_; }

double PositionSolver::get_major_tolerance() { return major_tolerance_; }

double PositionSolver::get_minor_tolerance() { return minor_tolerance_; }

ContactSolver::ContactSolver(const RigidBodyTree<double>& tree,
                             ContactInfo contact_info)
    : tree_(tree),
      contact_info_(contact_info),
      prog_(make_shared<MathematicalProgram>()) {
  // Initializing the variable
  q_ = prog_->NewContinuousVariables(tree_.get_num_positions(), "q");
}

void ContactSolver::SetInitialGuessQ(VectorXd q) {
  prog_->SetInitialGuess(q_, q);
}

SolutionResult ContactSolver::Solve(VectorXd q, vector<int> fixed_joints) {
  // Setting the solver options
  prog_->SetSolverOption(SnoptSolver::id(), "Log file", filename_);
  prog_->SetSolverOption(SnoptSolver::id(), "Major feasibility tolerance",
                         major_tolerance_);
  prog_->SetSolverOption(SnoptSolver::id(), "Minor feasibility tolerance",
                         minor_tolerance_);

  auto position_constraint = make_shared<PositionConstraint>(tree_);
  auto contact_constraint =
      make_shared<ContactConstraint>(tree_, contact_info_);

  prog_->AddConstraint(position_constraint, q_);
  prog_->AddConstraint(contact_constraint, q_);

  // Adding the fixed joint constraints
  for (uint i = 0; i < fixed_joints.size(); i++) {
    int ind = fixed_joints[i];
    prog_->AddConstraint(q_(ind) == q(ind));
  }

  prog_->AddQuadraticCost((q_ - q).dot(q_ - q));

  // The initial guess for q needs to be set up separately before calling Solve
  solution_result_ = prog_->Solve();

  return solution_result_;
}

bool ContactSolver::CheckConstraint(VectorXd q, double tolerance) const {
  auto position_constraint = make_shared<PositionConstraint>(tree_);
  auto contact_constraint =
      make_shared<ContactConstraint>(tree_, contact_info_);
  return position_constraint->CheckSatisfied(q, tolerance) &&
         contact_constraint->CheckSatisfied(q, tolerance);
}

shared_ptr<MathematicalProgram> ContactSolver::get_program() { return prog_; }

SolutionResult ContactSolver::get_solution_result() { return solution_result_; }

VectorXd ContactSolver::GetSolutionQ() { return prog_->GetSolution(q_); }

void ContactSolver::set_filename(string filename) { filename_ = filename; }

void ContactSolver::set_major_tolerance(double major_tolerance) {
  major_tolerance_ = major_tolerance;
}
void ContactSolver::set_minor_tolerance(double minor_tolerance) {
  minor_tolerance_ = minor_tolerance;
}

string ContactSolver::get_filename() { return filename_; }

double ContactSolver::get_major_tolerance() { return major_tolerance_; }

double ContactSolver::get_minor_tolerance() { return minor_tolerance_; }

FixedPointSolver::FixedPointSolver(const RigidBodyTree<double>& tree)
    : tree_(tree),
      contact_info_(ContactInfo()),
      prog_(make_shared<MathematicalProgram>()) {
  // Initializing the variable
  q_ = prog_->NewContinuousVariables(tree_.get_num_positions(), "q");
  u_ = prog_->NewContinuousVariables(tree_.get_num_actuators(), "u");
  lambda_ = prog_->NewContinuousVariables(tree_.getNumPositionConstraints(),
                                          "lambda");
}

FixedPointSolver::FixedPointSolver(const RigidBodyTree<double>& tree,
                                   ContactInfo contact_info)
    : tree_(tree),
      contact_info_(contact_info),
      prog_(make_shared<MathematicalProgram>()) {
  // Initializing the variable
  q_ = prog_->NewContinuousVariables(tree_.get_num_positions(), "q");
  u_ = prog_->NewContinuousVariables(tree_.get_num_actuators(), "u");
  lambda_ = prog_->NewContinuousVariables(
      tree_.getNumPositionConstraints() + contact_info.num_contacts * 3,
      "lambda");
}

void FixedPointSolver::SetInitialGuess(VectorXd q, VectorXd u,
                                       VectorXd lambda) {
  prog_->SetInitialGuess(q_, q);
  prog_->SetInitialGuess(u_, u);
  prog_->SetInitialGuess(lambda_, lambda);
}

void FixedPointSolver::SetInitialGuessQ(VectorXd q) {
  prog_->SetInitialGuess(q_, q);
}

void FixedPointSolver::SetInitialGuessU(VectorXd u) {
  prog_->SetInitialGuess(u_, u);
}

void FixedPointSolver::SetInitialGuessLambda(VectorXd lambda) {
  prog_->SetInitialGuess(lambda_, lambda);
}

void FixedPointSolver::AddSpreadNormalForcesCost() {
  // Adding cost on the normal forces to spread them out
  int num_position_constraints = tree_.getNumPositionConstraints();
  int num_contacts = contact_info_.num_contacts;
  int num_forces = num_position_constraints + num_contacts * 3;
  Expression normal_cost_expression(0);
  for (int i = num_position_constraints; i < num_forces; i = i + 3) {
    normal_cost_expression += lambda_(i) * lambda_(i);
  }
}

void FixedPointSolver::AddFrictionConeConstraint(const double mu) {
  // Adding the friction cone constraint at all the contact points
  int num_position_constraints = tree_.getNumPositionConstraints();
  int num_contacts = contact_info_.num_contacts;
  for (int i = 0; i < num_contacts; ++i) {
    prog_->AddConstraint(lambda_(i * 3 + 1 + num_position_constraints) <=
                         mu * lambda_(i * 3 + num_position_constraints));
    prog_->AddConstraint(-lambda_(i * 3 + 1 + num_position_constraints) <=
                         mu * lambda_(i * 3 + num_position_constraints));
    prog_->AddConstraint(lambda_(i * 3 + 2 + num_position_constraints) <=
                         mu * lambda_(i * 3 + num_position_constraints));
    prog_->AddConstraint(-lambda_(i * 3 + 2 + num_position_constraints) <=
                         mu * lambda_(i * 3 + num_position_constraints));
  }
}

SolutionResult FixedPointSolver::Solve(VectorXd q, VectorXd u,
                                       vector<int> fixed_joints) {
  // Setting the solver options
  prog_->SetSolverOption(SnoptSolver::id(), "Log file", filename_);
  prog_->SetSolverOption(SnoptSolver::id(), "Major feasibility tolerance",
                         major_tolerance_);
  prog_->SetSolverOption(SnoptSolver::id(), "Minor feasibility tolerance",
                         minor_tolerance_);

  auto position_constraint = make_shared<PositionConstraint>(tree_);
  auto contact_constraint =
      make_shared<ContactConstraint>(tree_, contact_info_);
  auto fixed_point_constraint =
      make_shared<FixedPointConstraint>(tree_, contact_info_);

  prog_->AddConstraint(position_constraint, q_);

  if (contact_info_.num_contacts) {
    prog_->AddConstraint(contact_constraint, q_);
  }

  prog_->AddConstraint(fixed_point_constraint, {q_, u_, lambda_});

  // Adding the fixed joint constraints
  for (uint i = 0; i < fixed_joints.size(); ++i) {
    int ind = fixed_joints[i];
    prog_->AddConstraint(q_(ind) == q(ind));
  }

  // Cost on q and u
  prog_->AddQuadraticCost((q_ - q).dot(q_ - q) + (u_ - u).dot(u_ - u));

  // The initial guess for q needs to be set up separately before calling
  // Solve
  solution_result_ = prog_->Solve();

  return solution_result_;
}

bool FixedPointSolver::CheckConstraint(VectorXd q, VectorXd u, VectorXd lambda,
                                       double tolerance) const {
  auto position_constraint = make_shared<PositionConstraint>(tree_);
  auto fixed_point_constraint =
      make_shared<FixedPointConstraint>(tree_, contact_info_);
  VectorXd q_u_l = VectorXd(q.size() + u.size() + lambda.size());
  q_u_l << q, u, lambda;
  std::cout << "Position constraint: "
            << position_constraint->CheckSatisfied(q, tolerance) << std::endl;
  std::cout << "FP constraint: "
            << fixed_point_constraint->CheckSatisfied(q_u_l, tolerance)
            << std::endl;
  return position_constraint->CheckSatisfied(q, tolerance) &&
         fixed_point_constraint->CheckSatisfied(q_u_l, tolerance);
}

shared_ptr<MathematicalProgram> FixedPointSolver::get_program() {
  return prog_;
}

SolutionResult FixedPointSolver::get_solution_result() {
  return solution_result_;
}

VectorXd FixedPointSolver::GetSolutionQ() { return prog_->GetSolution(q_); }

VectorXd FixedPointSolver::GetSolutionU() { return prog_->GetSolution(u_); }

VectorXd FixedPointSolver::GetSolutionLambda() {
  return prog_->GetSolution(lambda_);
}

void FixedPointSolver::set_filename(string filename) { filename_ = filename; }

void FixedPointSolver::set_major_tolerance(double major_tolerance) {
  major_tolerance_ = major_tolerance;
}
void FixedPointSolver::set_minor_tolerance(double minor_tolerance) {
  minor_tolerance_ = minor_tolerance;
}

string FixedPointSolver::get_filename() { return filename_; }

double FixedPointSolver::get_major_tolerance() { return major_tolerance_; }

double FixedPointSolver::get_minor_tolerance() { return minor_tolerance_; }

}  // namespace multibody
}  // namespace dairlib
