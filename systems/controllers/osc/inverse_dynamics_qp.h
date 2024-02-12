#pragma once
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

class InverseDynamicsQp {

 public:
  InverseDynamicsQp(
      const drake::multibody::MultibodyPlant<double> &plant,
      drake::systems::Context<double> *context);

  void AddHolonomicConstraint(
      std::unique_ptr<const multibody::KinematicEvaluatorSet<double>> eval);

  void AddContactConstraint(
      const std::string &name,
      std::unique_ptr<const multibody::WorldPointEvaluator<double>> eval);

  void AddExternalForce(
      const std::string &name,
      std::unique_ptr<const multibody::KinematicEvaluator<double>> eval);

  void Build();

  int nc() const {return nc_;}
  int nh() const {return nh_;}
  int nc_active() const {return nc_active_;}

  const drake::solvers::VectorXDecisionVariable &dv() const { return dv_; }
  const drake::solvers::VectorXDecisionVariable &u() const { return u_; }
  const drake::solvers::VectorXDecisionVariable &lambda_h() const {
    return lambda_h_;
  }
  const drake::solvers::VectorXDecisionVariable &lambda_c() const {
    return lambda_c_;
  }
  const drake::solvers::VectorXDecisionVariable &lambda_e() const {
    return lambda_e_;
  }
  const drake::solvers::VectorXDecisionVariable &epsilon() const {
    return epsilon_;
  }

  const drake::solvers::MathematicalProgram &get_prog() const {
    return prog_;
  }

  drake::solvers::MathematicalProgram &get_mutable_prog() {
    return prog_;
  }

  const std::unordered_map<std::string, int> get_contact_start_indices() const {
    return lambda_c_start_;
  }

  const ContactMap& get_contact_evaluators() const {
    return contact_constraint_evaluators_;
  }

  const multibody::KinematicEvaluatorSet<double>& get_holonomic_evaluators()
  const {
    return *holonomic_constraints_;
  }

  void AddQuadraticCost(
      const std::string &name, const Eigen::MatrixXd &Q,
      const Eigen::VectorXd &b,
      const drake::solvers::VectorXDecisionVariable& vars);

  void AddQuadraticCost(
      const std::string &name, const Eigen::MatrixXd &Q,
      const Eigen::VectorXd &b, const drake::solvers::VariableRefList &vars);

  void UpdateCost(
      const std::string &name, const Eigen::MatrixXd &Q,
      const Eigen::VectorXd &b, double c=0) {
    all_costs_.at(name)->UpdateCoefficients(Q, b, c, true);
  };

  void UpdateDynamics(
      const Eigen::VectorXd &x,
      const std::vector<std::string> &active_contact_constraints,
      const std::vector<std::string> &active_external_forces);

  void MakeAllInactiveForceCostsZero(
      const std::vector<std::string> &active_contacts,
      const std::vector<std::string> &active_external_forces);

  const drake::solvers::QuadraticCost&
  get_cost_evaluator(const std::string& name) const {
    return *all_costs_.at(name);
  }

  bool has_cost(const std::string& name) {
    return all_costs_.count(name) > 0;
  }

 private:

  // Multibody Dynamics
  const drake::multibody::MultibodyPlant<double> &plant_;
  drake::systems::Context<double> *context_;

  // Holonomic constraints are bilateral constraints that are always active
  std::unique_ptr<const multibody::KinematicEvaluatorSet<double>>
  holonomic_constraints_;

  // Contact constraints are unilateral constraints with an associated
  // contact force which obeys the friction cone
  ContactMap contact_constraint_evaluators_{};

  std::unordered_map<std::string, int> lambda_c_start_;
  std::unordered_map<std::string, int> Jc_active_start_;

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
  drake::solvers::VectorXDecisionVariable dv_{};
  drake::solvers::VectorXDecisionVariable u_{};
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