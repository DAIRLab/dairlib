#pragma once

#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/snopt_solver.h"
#include "multibody/contact_toolkit.h"

namespace dairlib {
namespace multibody {

class PositionConstraint : public drake::solvers::Constraint {
 public:
  PositionConstraint(const RigidBodyTree<double>& tree,
                     const std::string& description = "");
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& q,
              Eigen::VectorXd* y) const override;
  void DoEval(const Eigen::Ref<const drake::AutoDiffVecXd>& x,
              drake::AutoDiffVecXd* y) const override;
  void DoEval(
      const Eigen::Ref<const drake::VectorX<drake::symbolic::Variable>>& x,
      drake::VectorX<drake::symbolic::Expression>* y) const override;

 private:
  const RigidBodyTree<double>& tree_;
};

class FixedPointConstraint : public drake::solvers::Constraint {
 public:
  FixedPointConstraint(const RigidBodyTree<double>& tree,
                       ContactInfo contact_info,
                       const std::string& description = "");
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& q_u_l,
              Eigen::VectorXd* y) const override;
  void DoEval(const Eigen::Ref<const drake::AutoDiffVecXd>& q_u_l,
              drake::AutoDiffVecXd* y) const override;
  void DoEval(
      const Eigen::Ref<const drake::VectorX<drake::symbolic::Variable>>& q_u_l,
      drake::VectorX<drake::symbolic::Expression>* y) const override;

 private:
  std::unique_ptr<ContactToolkit<drake::AutoDiffXd>> contact_toolkit_;
  const RigidBodyTree<double>& tree_;
  ContactInfo contact_info_;
  const int num_positions_;
  const int num_velocities_;
  const int num_efforts_;
  const int num_position_forces_;
  const int num_contact_forces_;
  const int num_forces_;
};

class ContactConstraint : public drake::solvers::Constraint {
 public:
  ContactConstraint(const RigidBodyTree<double>& tree, ContactInfo contact_info,
                    const std::string& description = "");
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& q_u_l,
              Eigen::VectorXd* y) const override;
  void DoEval(const Eigen::Ref<const drake::AutoDiffVecXd>& q_u_l,
              drake::AutoDiffVecXd* y) const override;
  void DoEval(
      const Eigen::Ref<const drake::VectorX<drake::symbolic::Variable>>& q_u_l,
      drake::VectorX<drake::symbolic::Expression>* y) const override;

 private:
  std::unique_ptr<ContactToolkit<drake::AutoDiffXd>> contact_toolkit_;
  const RigidBodyTree<double>& tree_;
  ContactInfo contact_info_;
  const int num_positions_;
  const int num_velocities_;
  const int num_efforts_;
  const int num_position_forces_;
  const int num_contact_forces_;
  const int num_forces_;
};

class PositionSolver {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PositionSolver)

  PositionSolver(const RigidBodyTree<double>& tree);

  void SetInitialGuess(Eigen::VectorXd q);
  void Solve(Eigen::VectorXd q, std::vector<int> fixed_joints);
  bool CheckConstraint(Eigen::VectorXd q) const;

  std::shared_ptr<drake::solvers::MathematicalProgram> get_program();
  drake::solvers::SolutionResult get_solution_result();
  Eigen::VectorXd GetSolutionQ();

  void set_filename(std::string filename);
  void set_major_tolerance(double major_tolerance);
  void set_minor_tolerance(double minor_tolerance);

 private:
  const RigidBodyTree<double>& tree_;
  std::shared_ptr<drake::solvers::MathematicalProgram> prog_;
  drake::solvers::VectorXDecisionVariable q_;
  drake::solvers::SolutionResult solution_result_;
  std::string filename_ = "multibody/solver_log/position_solver";
  double major_tolerance_ = 1.0e-10;
  double minor_tolerance_ = 1.0e-10;
};

}  // namespace multibody
}  // namespace dairlib
