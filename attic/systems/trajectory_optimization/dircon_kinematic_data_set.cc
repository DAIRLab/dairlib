#include <chrono>

#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "attic/systems/trajectory_optimization/dircon_kinematic_data_set.h"

namespace dairlib {

using std::vector;
using drake::VectorX;
using drake::MatrixX;
using drake::AutoDiffXd;

template <typename T>
DirconKinematicDataSet<T>::DirconKinematicDataSet(
    const RigidBodyTree<double>& tree,
    vector<DirconKinematicData<T>*>* constraints) :
    DirconKinematicDataSet(tree, constraints, tree.get_num_positions(),
                           tree.get_num_velocities()) {}

template <typename T>
DirconKinematicDataSet<T>::DirconKinematicDataSet(
    const RigidBodyTree<double>& tree,
    vector<DirconKinematicData<T>*>* constraints,
    int num_positions, int num_velocities) :
    cache_(tree.CreateKinematicsCacheWithType<T>()) {
  tree_ = &tree;

  constraints_ = constraints;
  num_positions_ = num_positions;
  num_velocities_ = num_velocities;
  // Initialize matrices
  constraint_count_ = 0;
  for (uint i=0; i < constraints_->size(); i++) {
    constraint_count_ += (*constraints_)[i]->getLength();
  }
  c_ = VectorX<T>(constraint_count_);
  cdot_ = VectorX<T>(constraint_count_);
  J_ = MatrixX<T>(constraint_count_, num_positions);
  Jdotv_ = VectorX<T>(constraint_count_);
  cddot_ = VectorX<T>(constraint_count_);
  vdot_ = VectorX<T>(num_velocities_);
  xdot_ = VectorX<T>(num_positions_ + num_velocities_);
}


template <typename T>
void DirconKinematicDataSet<T>::updateData(const VectorX<T>& state,
                                           const VectorX<T>& input,
                                           const VectorX<T>& forces) {
  const VectorX<T> q = state.head(num_positions_);
  const VectorX<T> v = state.tail(num_velocities_);
  cache_ = tree_->doKinematics(q, v, true);

  int index = 0;
  int n;
  for (uint i=0; i < constraints_->size(); i++) {
    (*constraints_)[i]->updateConstraint(cache_);

    n = (*constraints_)[i]->getLength();
    c_.segment(index, n) = (*constraints_)[i]->getC();
    cdot_.segment(index, n) = (*constraints_)[i]->getCDot();
    J_.block(index, 0, n, num_positions_) = (*constraints_)[i]->getJ();
    Jdotv_.segment(index, n) = (*constraints_)[i]->getJdotv();

    index += n;
  }

  const MatrixX<T> M = tree_->massMatrix(cache_);
  const typename RigidBodyTree<T>::BodyToWrenchMap no_external_wrenches;
  const MatrixX<T> J_transpose = getJ().transpose();

  // right_hand_side is the right hand side of the system's equations:
  // M*vdot -J^T*f = right_hand_side.
  VectorX<T> right_hand_side =
      -tree_->dynamicsBiasTerm(cache_, no_external_wrenches) +
      tree_->B*input + J_transpose*forces;
  vdot_ = M.llt().solve(right_hand_side);

  cddot_ = Jdotv_ + J_*vdot_;

  // assumes v = qdot
  xdot_ << tree_->GetVelocityToQDotMapping(cache_)*v, vdot_;
}

template <typename T>
int DirconKinematicDataSet<T>::countConstraints() {
  return constraint_count_;
}

template <typename T>
int DirconKinematicDataSet<T>::getNumConstraintObjects() {
  return constraints_->size();
}

template <typename T>
VectorX<T> DirconKinematicDataSet<T>::getC() {
  return c_;
}

template <typename T>
VectorX<T> DirconKinematicDataSet<T>::getCDot() {
  return cdot_;
}

template <typename T>
MatrixX<T> DirconKinematicDataSet<T>::getJ() {
  return J_;
}

template <typename T>
VectorX<T> DirconKinematicDataSet<T>::getJdotv() {
  return Jdotv_;
}

template <typename T>
VectorX<T> DirconKinematicDataSet<T>::getCDDot() {
  return cddot_;
}

template <typename T>
VectorX<T> DirconKinematicDataSet<T>::getVDot() {
  return vdot_;
}

template <typename T>
VectorX<T> DirconKinematicDataSet<T>::getXDot() {
  return xdot_;
}

template <typename T>
DirconKinematicData<T>* DirconKinematicDataSet<T>::getConstraint(int index) {
  return (*constraints_)[index];
}


// Explicitly instantiates on the most common scalar types.
template class DirconKinematicDataSet<double>;
template class DirconKinematicDataSet<AutoDiffXd>;

}  // namespace dairlib
