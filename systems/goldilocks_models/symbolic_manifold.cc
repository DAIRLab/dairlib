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
  // n_features_ = 3*tree.get_num_positions() + 3*tree.get_num_velocities() + 1;
  n_features_ = features.size();

  // std::cout << n_features_ << weights.cols() << std::endl;
  DRAKE_ASSERT(n_features_ == weights.cols());
}

// symbolic::Formula SymbolicManifold::getConstraintFormula(int index) {
//   return weights_.row(index).dot(features_) == 0;
// }

VectorX<symbolic::Expression> SymbolicManifold::getConstraintExpressions() {
  // std::set<symbolic::Formula> ret;
  // for (int i = 0; i < weights_.rows(); i++) {
  //   ret.insert(getConstraintFormula(i));
  // }
  // return ret;
  return weights_*features_;
}

}
}