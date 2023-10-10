#pragma once
#include <limits>
#include "multibody/kinematic/kinematic_evaluator_set.h"
#include "multibody/kinematic/world_point_evaluator.h"
#include "solvers/fast_osqp_solver.h"

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/osqp_solver.h"
#include "drake/solvers/solve.h"

// Dynamics QP is a general purpose interface for solving the manipulator
// equations subject to constraints like contact forces, inputs, etc. It can be
// used for things like operational space control, force control, etc.

namespace dairlib {
namespace multibody {

using std::map;
using std::vector;
using std::unique_ptr;
using std::shared_ptr;

using drake::systems::Context;
using drake::solvers::QuadraticCost;
using drake::solvers::LinearConstraint;
using drake::solvers::MathematicalProgram;
using drake::solvers::BoundingBoxConstraint;
using drake::solvers::VectorXDecisionVariable;
using drake::solvers::LinearEqualityConstraint;

class DynamicsQP {
 public:
  DynamicsQP(const drake::multibody::MultibodyPlant<double> &plant);

  /*!
   * Build the QP by adding all of the costs and constraints to the
   * MathematicalProgram
   */
  void Build();

  /*!
   * @brief Add the permanent kinematic constraints defined by evaluators
   */
  void AddKinematicConstraint(
      std::unique_ptr<const KinematicEvaluatorSet<double>> evaluators) {
    DRAKE_DEMAND(&evaluators->plant() == &plant_);
    kinematic_evals_ = std::move(evaluators);
    nh_ = kinematic_evals_->count_full();
  }

  /*!
   * @brief Update the lower and upper bounds of the input constraint.
   *        Set from the URDF by default.
   * @param lb: input lower bound
   * @param ub: input upper bound
   */
  void UpdateInputLimits(const Eigen::VectorXd &lb, const Eigen::VectorXd ub) {
    DRAKE_DEMAND(lb.rows() == nu_);
    DRAKE_DEMAND(ub.rows() == nu_);
    input_bounds_->set_bounds(lb, ub);
  }

  /*!
   * @brief Update all constraints which depend on q and v (kinematic, contact,
   * and dynamics)
   */
  void UpdateStateDependentConstraints(const Context<double>& context,
                                       vector<std::string>& active_contacts);

  /*!
   * @brief Update the manipulator equations given the current robot state
   * (inn context) and active contacts
   * @param context robot context
   * @param active_contacts names of the active contexts
   */
  void UpdateDynamicsConstraint(const Context<double>& context,
                                vector<std::string>& active_contacts);

  /*!
   * @brief Add a contact point to the QP
   * @param name name of the contact
   * @param W_soft_constraint wieght of the soft contraint version of this
   * contact constraint. To enforce a hard constraint, make this a 0x0 matrix
   */
  void AddContactPoint(
      unique_ptr<const WorldPointEvaluator<double>>,
      const std::string& name,
      Eigen::MatrixXd W_soft_constraint, double mu);

 private:

  // Specialized struct to make it easier to deal with contact constraints
  struct ContactConstraint {
    // Initialized in AddContactPoint
    unique_ptr<const WorldPointEvaluator<double>> evaluator_ = nullptr;
    Eigen::MatrixXd W_soft_constraint_;
    const int idx_lambda_start_;
    const int idx_eps_start_;
    double mu_;
    bool soft_constraint_;
    int nc_active_;
    int n_eps_;

    // Initialized in MakeContactConstraints
    shared_ptr<LinearEqualityConstraint> contact_constraint_ = nullptr;
    shared_ptr<LinearConstraint> friction_cone_ = nullptr;
    shared_ptr<QuadraticCost> soft_constraint_cost_ = nullptr;

    void UpdateActive(const Context<double>& context) {
      int nv = evaluator_->plant().num_velocities();
      friction_cone_->UpdateLowerBound(
          Eigen::VectorXd::Zero(friction_cone_->num_constraints())
      );
      Eigen::MatrixXd A_c = Eigen::MatrixXd::Zero(nc_active_, nv + n_eps_);
      A_c.leftCols(nv) = evaluator_->EvalActiveJacobian(context);
      A_c.rightCols(n_eps_) = Eigen::MatrixXd::Identity(n_eps_, n_eps_);
      contact_constraint_->UpdateCoefficients(
          A_c, -evaluator_->EvalActiveJacobianDotTimesV(context)
      );
    }
    void UpdateInactive(const Context<double>& context) {
      friction_cone_->UpdateLowerBound(Eigen::VectorXd::Constant(
          friction_cone_->num_constraints(),
          -std::numeric_limits<double>::infinity()
      ));
      contact_constraint_->UpdateCoefficients(
          Eigen::MatrixXd::Zero(
              contact_constraint_->num_constraints(),
              contact_constraint_->num_vars()
          ),
          Eigen::VectorXd::Zero(contact_constraint_->num_constraints())
      );
    }
  };

  // Helper functions for Build
  void MakeDynamicsConstraint();
  void MakeContactConstraints();
  void MakeKinematicConstraints();
  void MakeInputBoundsConstraint();

  // variable counts
  const int nq_;
  const int nv_;
  const int nu_;
  int nh_ = 0;
  int nc_ = 0;
  int nc_active_ = 0;
  int nc_soft_ = 0;

  // multibody
  const drake::multibody::MultibodyPlant<double> &plant_;
  unique_ptr<const KinematicEvaluatorSet<double>> kinematic_evals_ = nullptr;
  map<std::string, ContactConstraint> contacts_;

  // MathematicalProgram
  unique_ptr<MathematicalProgram> prog_ = nullptr;

  // Decision variables
  VectorXDecisionVariable u_;
  VectorXDecisionVariable dv_;
  VectorXDecisionVariable lambda_h_;
  VectorXDecisionVariable lambda_c_;
  VectorXDecisionVariable epsilon_c_;

  // Constraints
  shared_ptr<LinearEqualityConstraint> dynamics_constraint_ = nullptr;
  shared_ptr<LinearEqualityConstraint> holonomic_constraint_ = nullptr;
  shared_ptr<BoundingBoxConstraint> input_bounds_ = nullptr;

};
}
}