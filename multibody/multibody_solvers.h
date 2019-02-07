#pragma once

#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/mathematical_program.h"

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

//class FixedPointConstraint : public drake::solvers::Constraint {
// public:
//  FixedPointConstraint(const RigidBodyTree<double>& tree, const int num_forces,
//                       const std::string& description = "");
//  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& q_u_l,
//              Eigen::VectorXd* y) const override;
//  void DoEval(const Eigen::Ref<const drake::AutoDiffVecXd>& q_u_l,
//              drake::AutoDiffVecXd* y) const override;
//  void DoEval(
//      const Eigen::Ref<const drake::VectorX<drake::symbolic::Variable>>& q_u_l,
//      drake::VectorX<drake::symbolic::Expression>* y) const override;
//
// private:
//  const RigidBodyTree<double>& tree_;
//  const int num_forces_;
//};
//
//class FixedPointSolver {};
//
//class PositionSolver {};

}  // namespace multibody
}  // namespace dairlib
