#include "manifold_constraint.h"
#include "drake/common/drake_assert.h"

namespace drake {
namespace goldilocks_walking {

using solvers::Constraint;
using Eigen::VectorXd;
using Eigen::MatrixXd;

//
ManifoldConstraint::ManifoldConstraint(const RigidBodyTree<double>& tree,
  const MatrixXd& weights)
  : Constraint(weights.rows(), tree.get_num_positions() + tree.get_num_velocities(),
   VectorXd::Zero(1), VectorXd::Zero(1), "manifold"), weights_{weights} {
  tree_ = &tree;
  n_features_ = 3*tree.get_num_positions() + 3*tree.get_num_velocities() + 1;

  DRAKE_ASSERT(n_features_ == weights.cols());
}

void ManifoldConstraint::DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd& y) const {
  EvaluateConstraint<double>(x,y);
}

void ManifoldConstraint::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd& y) const {
  EvaluateConstraint<AutoDiffXd>(x,y);
}

template <typename T>
VectorX<T> ManifoldConstraint::CalcFeatures(const Eigen::Ref<const VectorX<T>>& x) const {
  VectorX<T> features(n_features_);
  features(0) = 1; //constant feature
  for (int i = 0; i < x.size(); i++) {
    features(i+1) = x(i);
    features(x.size()+i+1) = cos(x(i));
    features(2*x.size()+i+1) = sin(x(i));
  }
  return features;
}

template <typename T>
void ManifoldConstraint::EvaluateConstraint(const Eigen::Ref<const VectorX<T>>& x,
              VectorX<T>& y) const {
  VectorX<T> features = CalcFeatures(x);
  y = weights_ * features;
}

template void ManifoldConstraint::EvaluateConstraint<double>(const Eigen::Ref<const VectorXd>&,VectorXd& y) const;
template void ManifoldConstraint::EvaluateConstraint<AutoDiffXd>(const Eigen::Ref<const AutoDiffVecXd>&,AutoDiffVecXd& y) const;

}
}