#pragma once

#include "drake/multibody/rigid_body_tree.h"
#include "drake/common/symbolic.h"

namespace dairlib {
namespace goldilocks_models {


/// Manifold constraint class
/// Defined by providing it a basis set
/// The constraints, in vector form, are 0 = weight * features
class SymbolicManifold {
 public:
  /// @param tree the RigidBodyTree (probably not needed?)
  /// @param features A vector of symbolic features
  /// @param weights the weights. Each row of the weights matrix corresponds
  ///    to a different constraint
  SymbolicManifold(const RigidBodyTree<double>& tree,
    const drake::VectorX<drake::symbolic::Expression>& features,
    const Eigen::MatrixXd& weights);

  int n_features() {return n_features_;}

  /// Return the expressions for the constraints, weights * features
  drake::VectorX<drake::symbolic::Expression> getConstraintExpressions();

  drake::symbolic::Expression getFeature(int index) {return features_(index);}

  int n_features_;
  const Eigen::MatrixXd weights_;
  const RigidBodyTree<double>* tree_;
  const drake::VectorX<drake::symbolic::Expression> features_;
};

}  // namespace goldilocks_models
}  // namespace dairlib
