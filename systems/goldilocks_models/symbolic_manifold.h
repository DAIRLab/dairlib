#pragma once

#include "drake/multibody/rigid_body_tree.h"
#include "drake/common/symbolic.h"

namespace drake {
namespace goldilocks_models {


//Manifold constraint class
//Currently uses a predefined basis set
class SymbolicManifold {
 public:
  SymbolicManifold(const RigidBodyTree<double>& tree,
    const VectorX<symbolic::Expression>& features,
    const Eigen::MatrixXd& weights);

  int n_features() {return n_features_;};

  VectorX<symbolic::Expression> getConstraintExpressions();
  VectorX<symbolic::Formula> getConstraintFormulas();

  // symbolic::Formula getConstraintFormula(int index);

  symbolic::Expression getFeature(int index) {return features_(index);};

  int n_features_;
  const Eigen::MatrixXd weights_;
  const RigidBodyTree<double>* tree_;
  const VectorX<symbolic::Expression> features_;
};
}
}