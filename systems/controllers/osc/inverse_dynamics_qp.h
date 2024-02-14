#pragma once

#include "dairlib/lcmt_id_qp.hpp"
#include "multibody/kinematic/kinematic_evaluator_set.h"
#include "multibody/kinematic/world_point_evaluator.h"

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/mathematical_program.h"


namespace dairlib {
namespace systems {
namespace controllers {

using CostMap = std::unordered_map<
    std::string, std::shared_ptr<drake::solvers::QuadraticCost>>;

using FrictionConeMap = std::unordered_map<
    std::string, std::shared_ptr<drake::solvers::LinearConstraint>>;

using ContactMap = std::unordered_map<
    std::string,
    std::unique_ptr<const multibody::WorldPointEvaluator<double>>>;


/*!
 * Wrapper class for handling kinematics and dynamics for a quadratic program
 * which computes dynamically consistent accelerations, inputs,
 * and constraint forces (including contacts) which minimize some combined
 * cost on these variables.
 *
 * Designed to be used as a back-end for operational space control, but
 * applicable to other model-based instantaneous QP controllers.
 *
 * Constructs a QP of the form
 *
 * minimize C₁(v̇) + C₂(u), C₃(λₕ) + C₄(λ_c) + C₄(λₑ)
 * subject to Jₕv̇ + J̇ₕ = 0
 *            J_cv̇ + J̇_c = 0
 *            Mv̇ + c = Bu + Jₕᵀλₕ + J_cᵀλ_c + Jₑᵀλₑ
 *            λ_c ∈ FrictionCone
 *
 * Where
 *
 * Cᵢ are arbitrary user-defined convex quadratic costs,
 * v̇ are generalized accelerations,
 * u are actuation efforts,
 * λₕ are constraint forces for general holonomic constraints such linkages,
 * λ_c are constraint forces for contact constraints
 * λₑ are external forces, such as contact forces, which do not constrain the
 * robot's motion (like force on the end effector of a robot arm)
 *
 */
class InverseDynamicsQp {

 public:
  InverseDynamicsQp(
      const drake::multibody::MultibodyPlant<double> &plant,
      drake::systems::Context<double> *context);

  /*!
   * @brief Adds the set of holonomic constraints given by eval to the QP.
   */
  void AddHolonomicConstraint(
      std::unique_ptr<const multibody::KinematicEvaluatorSet<double>> eval);

  /*!
   * Adds a contact constraint
   * @param name name of the contact constraint
   * @param eval WorldPointEvaluator describing the contact point
   * @param friction_coefficient friction coefficient of the friction cone for
   * the associated contact force
   */
  void AddContactConstraint(
      const std::string &name,
      std::unique_ptr<const multibody::WorldPointEvaluator<double>> eval,
      double friction_coefficient=1);

  /*!
   * Adds an external force to the dynamics
   * @param name name of the external force
   * @param eval kinematic evaluator for the associated jacobian
   */
  // TODO (@Brian-Acosta) should this be explicitly point force for now?
  void AddExternalForce(
      const std::string &name,
      std::unique_ptr<const multibody::KinematicEvaluator<double>> eval);

  /*!
   * Adds the quadratic cost 1/2 xᵀQx + bᵀx to the underlying QP
   *
   * Must be called AFTER build
   *
   * @param name name of the cost (must be unique)
   * @param Q PSD cost hessian
   * @param b linear term
   * @param vars decision variables representing x
   */
  void AddQuadraticCost(
      const std::string &name, const Eigen::MatrixXd &Q,
      const Eigen::VectorXd &b,
      const drake::solvers::VectorXDecisionVariable& vars);

  /*!
   * See above
   */
  void AddQuadraticCost(
      const std::string &name, const Eigen::MatrixXd &Q,
      const Eigen::VectorXd &b, const drake::solvers::VariableRefList &vars);

  /*!
   * Builds the underlying QP
   */
  void Build();

  /*!
   * @return the total dimension of the (stacked) contact forces.
   * This is greater than or equal to the number of active rows in the
   * contact constraint.
   */
  [[nodiscard]] int nc() const {return nc_;}

  /*!
   * @return the total dimension of the holonomic constraint forces.
   * Equal to the number of rows in the holonomic constraint.
   */
  [[nodiscard]] int nh() const {return nh_;}


  /*!
   * @return the total number of rows in the contact constraint for all contacts
   */
  [[nodiscard]] int nc_active() const {return nc_active_;}

  /*
   * N.B. To avoid overhead in the OSC loop, we don't check the if the QP is
   * built before these are called, but the decision variables are empty
   * until then!
   */
  [[nodiscard]] const drake::solvers::VectorXDecisionVariable &dv() const {
    return dv_;
  }
  [[nodiscard]] const drake::solvers::VectorXDecisionVariable &u() const {
    return u_;
  }
  [[nodiscard]] const drake::solvers::VectorXDecisionVariable &lambda_h()
  const {
    return lambda_h_;
  }
  [[nodiscard]] const drake::solvers::VectorXDecisionVariable &lambda_c()
  const {
    return lambda_c_;
  }
  [[nodiscard]] const drake::solvers::VectorXDecisionVariable &lambda_e()
  const {
    return lambda_e_;
  }
  [[nodiscard]] const drake::solvers::VectorXDecisionVariable &epsilon() const {
    return epsilon_;
  }

  /*!
   * @return a const-reference to the underlying MathematicalProgram
   */
  [[nodiscard]] const drake::solvers::MathematicalProgram &get_prog() const {
    return prog_;
  }

  /*!
   * @return a mutable reference to the underlying MathematicalProgram
   */
  [[nodiscard]] drake::solvers::MathematicalProgram &get_mutable_prog() {
    return prog_;
  }

  /*!
   * @brief gets a const reference WorldPointEvaluator for the contact with
   * the given name
   */
  [[nodiscard]] const multibody::WorldPointEvaluator<double>&
  get_contact_evaluator(const std::string& name) const {
    return *contact_constraint_evaluators_.at(name);
  }

  /*!
   * @brief gets a const reference to the KinematicEvaluatorSet associated
   * with the holonomic constraints
   */
  [[nodiscard]] const multibody::KinematicEvaluatorSet<double>&
  get_holonomic_evaluators() const {
    return *holonomic_constraints_;
  }

  /*!
   * @brief updates the coefficients of the cost with the given name to
   * 1/2 xᵀQx + bᵀx + c
   */
  void UpdateCost(
      const std::string &name, const Eigen::MatrixXd &Q,
      const Eigen::VectorXd &b, double c=0) {
    all_costs_.at(name)->UpdateCoefficients(Q, b, c, true);
  };

  /*!
   * updates the dynamics and contact constraints to reflect a given
   * state and contact configuration.
   * @param x the robot state [q, v]
   * @param active_contact_constraints the names of all contact constraints
   * currently in contact
   * @param active_external_forces the names of all the external forces
   * currently being applied
   */
  void UpdateDynamics(
      const Eigen::VectorXd &x,
      const std::vector<std::string> &active_contact_constraints,
      const std::vector<std::string> &active_external_forces);

  lcmt_id_qp SerializeToLcm() const;

  /*!
   * @brief gets the drake QuadraticCost evaluator associated with a given cost
   */
  [[nodiscard]] const drake::solvers::QuadraticCost&
  get_cost_evaluator(const std::string& name) const {
    return *all_costs_.at(name);
  }

  /*!
   * @brief checks if this QP has a cost with the given name
   */
  bool has_cost_named(const std::string& name) {
    return all_costs_.count(name) > 0;
  }

 private:

  // Multibody Dynamics
  const drake::multibody::MultibodyPlant<double> &plant_;
  drake::systems::Context<double> *context_;

  // Holonomic constraints are bilateral constraints that are always active
  std::unique_ptr<const multibody::KinematicEvaluatorSet<double>>
  holonomic_constraints_ = nullptr;

  // Contact constraints are unilateral constraints with an associated
  // contact force which obeys the friction cone
  ContactMap contact_constraint_evaluators_{};
  std::unordered_map<std::string, int> lambda_c_start_;
  std::unordered_map<std::string, int> Jc_active_start_;
  std::unordered_map<std::string, double> mu_map_;

  // External forces are reaction forces we want to "track," but which do not
  // constrain the robot's motion
  std::unordered_map<
      std::string, std::unique_ptr<const multibody::KinematicEvaluator<double>>>
      external_force_evaluators_{};

  std::unordered_map<std::string, std::pair<int, int>> lambda_e_start_and_size_;

  // Size of velocity and input of the plant
  int nv_;
  int nu_;

  // Size of constraints and external forces
  int nh_ = 0;
  int ne_ = 0;
  int nc_ = 0;
  int nc_active_ = 0;

  // Mathematical Program
  drake::solvers::MathematicalProgram prog_;

  // Decision Variables
  drake::solvers::VectorXDecisionVariable u_{};
  drake::solvers::VectorXDecisionVariable dv_{};
  drake::solvers::VectorXDecisionVariable lambda_h_{};
  drake::solvers::VectorXDecisionVariable lambda_c_{};
  drake::solvers::VectorXDecisionVariable lambda_e_{};
  drake::solvers::VectorXDecisionVariable epsilon_{};

  // Costs
  CostMap all_costs_;
  std::shared_ptr<drake::solvers::QuadraticCost> soft_constraint_cost_;

  // Friction Cone Constraints
  FrictionConeMap lambda_c_friction_cone_;
  FrictionConeMap lambda_e_friction_cone_;

  // Dynamics
  std::shared_ptr<drake::solvers::LinearEqualityConstraint> dynamics_c_;
  std::shared_ptr<drake::solvers::LinearEqualityConstraint> holonomic_c_;
  std::shared_ptr<drake::solvers::LinearEqualityConstraint> contact_c_;

  // Bookkeeping
  bool built_ = false;
};

}
}
}