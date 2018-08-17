#include "symbolic_manifold.h"
#include "drake/common/drake_assert.h"

namespace drake {
namespace goldilocks_models {

using Eigen::VectorXd;

using Eigen::MatrixXd;

//
SymbolicManifold::SymbolicManifold(const RigidBodyTree<double>& tree,
  const VectorX<symbolic::Expression>& features,
  const MatrixXd& weights)
  : weights_{weights}, tree_{&tree}, features_{features} {
  n_features_ = features.size();

  DRAKE_ASSERT(n_features_ == weights.cols());
}

VectorX<symbolic::Expression> SymbolicManifold::getConstraintExpressions() {
  return weights_*features_;
}

}  // namespace goldilocks_models
}  // namespace drake
