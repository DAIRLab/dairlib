#include "dircon_kinematic_constraint.h"

namespace drake{

template <typename T>
DirconKinematicConstraint<T>::DirconKinematicConstraint(RigidBodyTree<double>* tree, int length) {
  int nq = 1;
  tree_ = tree;
  c_ = VectorX<T>(length);
  cdot_ = VectorX<T>(length);
  J_ = MatrixX<T>(length,nq);
  Jdotv_ = VectorX<T>(length);
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

// Explicitly instantiates on the most common scalar types.
template class DirconKinematicConstraint<double>;
template class DirconKinematicConstraint<AutoDiffXd>;
}

