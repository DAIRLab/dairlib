#include "symbolic_manifold.h"
#include "drake/common/drake_assert.h"

namespace dairlib {
namespace goldilocks_models {

using Eigen::VectorXd;
using Eigen::MatrixXd;
using drake::VectorX;
using drake::symbolic::Expression;

//
SymbolicManifold::SymbolicManifold(const RigidBodyTree<double>& tree,
  const VectorX<Expression>& features,
  const MatrixXd& weights)
  : weights_{weights}, tree_{&tree}, features_{features} {
  n_features_ = features.size();

  DRAKE_ASSERT(n_features_ == weights.cols());
}

VectorX<Expression> SymbolicManifold::getConstraintExpressions() {
  return weights_*features_;
}

}  // namespace goldilocks_models
}  // namespace dairlib
