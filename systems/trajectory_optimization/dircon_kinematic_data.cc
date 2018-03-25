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

}

