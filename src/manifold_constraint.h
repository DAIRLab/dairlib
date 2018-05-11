#pragma once

#include "drake/solvers/constraint.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace goldilocks_walking {

//Manifold constraint class
//Currently uses a predefined basis set
class ManifoldConstraint : public solvers::Constraint {
 public:
  ManifoldConstraint(const RigidBodyTree<double>& tree,
    const Eigen::MatrixXd& weights);

  int n_features() {return n_features_;};

 public:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd& y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd& y) const override;

  template <typename T>
  void EvaluateConstraint(const Eigen::Ref<const VectorX<T>>& x,
              VectorX<T>& y) const;

  template <typename T>
  VectorX<T> CalcFeatures(const Eigen::Ref<const VectorX<T>>& x) const;

  int n_features_;
  const Eigen::MatrixXd weights_;
  const RigidBodyTree<double>* tree_;
};
}
}