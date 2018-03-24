#include "dircon_kinematic_data.h"

namespace drake{

template <typename T>
DirconKinematicData<T>::DirconKinematicData(const RigidBodyTree<double>& tree, int length) {
  tree_ = &tree;
  length_ = length;
  //force_constraints_ = std::vector<solvers::Constraint>(0);
}

template <typename T>
DirconKinematicData<T>::~DirconKinematicData() {}

template <typename T>
VectorX<T> DirconKinematicData<T>::getC() {
  return c_;
}

template <typename T>
VectorX<T> DirconKinematicData<T>::getCDot() {
  return cdot_;
}

template <typename T>
MatrixX<T> DirconKinematicData<T>::getJ() {
  return J_;
}

template <typename T>
VectorX<T> DirconKinematicData<T>::getJdotv() {
  return Jdotv_;
}

template <typename T>
int DirconKinematicData<T>::getLength() {
  return length_;
}

// Explicitly instantiates on the most common scalar types.
template class DirconKinematicData<double>;
template class DirconKinematicData<AutoDiffXd>;

template <typename T>
DirconKinematicDataSet<T>::DirconKinematicDataSet(RigidBodyTree<double>& tree, std::vector<DirconKinematicData<T>> constraints, int num_positions, int num_velocities) {
  tree_ = &tree;

  constraints_ = constraints;
  num_positions_ = num_positions;
  num_velocities_ = num_velocities;
  // Initialize matrices
  num_constraints_ = 0;
  for (int i=0; i < constraints_.size(); i++) {
    num_constraints_ += constraints_[i].getLength();
  }
  c_ = VectorX<T>(num_constraints_);
  cdot_ = VectorX<T>(num_constraints_);
  J_ = MatrixX<T>(num_constraints_,num_positions);
  Jdotv_ = VectorX<T>(num_constraints_);
}

template <typename T>
DirconKinematicDataSet<T>::~DirconKinematicDataSet() {
}

template <typename T>
void DirconKinematicDataSet<T>::updateData(const VectorX<T>& state, const VectorX<T>& input, const VectorX<T>& forces) {
  AutoDiffVecXd q = state.head(num_positions_);
  AutoDiffVecXd v = state.tail(num_velocities_);
  KinematicsCache<T> cache = tree_->doKinematics(q, v, true);

  int index = 0;
  int n;
  for (int i=0; i < constraints_.size(); i++) {
    constraints_[i].updateConstraint(cache);

    n = constraints_[i].getLength();
    c_.segment(index, n) = constraints_[i].getC();
    cdot_.segment(index, n) = constraints_[i].getCDot();
    J_.block(index, 0, n, num_positions_) = constraints_[i].getJ();
    Jdotv_.segment(index, n) = constraints_[i].getJdotv();

    index += n;
  }

  updateVdot(state, input, forces);

  cddot_ = Jdotv_ + J_*vdot_;
}

template <typename T>
void DirconKinematicDataSet<T>::updateVdot(const VectorX<T>& state, const VectorX<T>& input, const VectorX<T>& forces) {
  VectorX<T> q = state.head(num_positions_);
  VectorX<T> v = state.tail(num_velocities_);

  KinematicsCache<T> kinsol = tree_->doKinematics(q,v,true);

  const MatrixX<T> M = tree_->massMatrix(kinsol);
  const drake::eigen_aligned_std_unordered_map<RigidBody<double> const*, drake::WrenchVector<T>> no_external_wrenches;
  const MatrixX<T> J_transpose = constraints_->getJ().transpose();

  // right_hand_side is the right hand side of the system's equations:
  // M*vdot -J^T*f = right_hand_side.
  VectorX<T> right_hand_side = -tree_->dynamicsBiasTerm(kinsol, no_external_wrenches) + tree_->B * input + J_transpose*forces;

  vdot_ = M.llt().solve(right_hand_side);
}


template <typename T>
int DirconKinematicDataSet<T>::getNumConstraints() {
  return num_constraints_;
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
}

