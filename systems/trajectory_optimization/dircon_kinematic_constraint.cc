#include "dircon_kinematic_constraint.h"

namespace drake{

template <typename T>
DirconKinematicConstraint<T>::DirconKinematicConstraint(const RigidBodyTree<double>& tree, int length) {
  tree_ = &tree;
  length_ = length;
  //force_constraints_ = std::vector<solvers::Constraint>(0);
}

template <typename T>
DirconKinematicConstraint<T>::~DirconKinematicConstraint() {}

template <typename T>
VectorX<T> DirconKinematicConstraint<T>::getC() {
  return c_;
}

template <typename T>
VectorX<T> DirconKinematicConstraint<T>::getCDot() {
  return cdot_;
}

template <typename T>
MatrixX<T> DirconKinematicConstraint<T>::getJ() {
  return J_;
}

template <typename T>
VectorX<T> DirconKinematicConstraint<T>::getJdotv() {
  return Jdotv_;
}

template <typename T>
int DirconKinematicConstraint<T>::getLength() {
  return length_;
}



// Explicitly instantiates on the most common scalar types.
template class DirconKinematicConstraint<double>;
template class DirconKinematicConstraint<AutoDiffXd>;

template <typename T>
DirconKinematicConstraintSet<T>::DirconKinematicConstraintSet(std::vector<DirconKinematicConstraint<T>> constraints, int num_positions) {
  constraints_ = constraints;
  num_positions_ = num_positions;
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
DirconKinematicConstraintSet<T>::~DirconKinematicConstraintSet() {
}


template <typename T>
void DirconKinematicConstraintSet<T>::updateConstraints(KinematicsCache<T>& cache) {
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
}

template <typename T>
int DirconKinematicConstraintSet<T>::getNumConstraints() {
  return num_constraints_;
}

template <typename T>
VectorX<T> DirconKinematicConstraintSet<T>::getC() {
  return c_;
}

template <typename T>
VectorX<T> DirconKinematicConstraintSet<T>::getCDot() {
  return cdot_;
}

template <typename T>
MatrixX<T> DirconKinematicConstraintSet<T>::getJ() {
  return J_;
}

template <typename T>
VectorX<T> DirconKinematicConstraintSet<T>::getJdotv() {
  return Jdotv_;
}
}

