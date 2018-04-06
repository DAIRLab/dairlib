#include "dircon_kinematic_data.h"

namespace drake{

template <typename T>
DirconKinematicData<T>::DirconKinematicData(RigidBodyTree<double>& tree, int length) {
  tree_ = &tree;
  length_ = length;
  force_constraints_ = std::vector<std::shared_ptr<solvers::Constraint>>(0);
  c_ = VectorX<T>::Zero(length);
  cdot_ = VectorX<T>::Zero(length);
  J_ = MatrixX<T>::Zero(length, tree.get_num_positions());
  Jdotv_ = VectorX<T>::Zero(length);
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

template <typename T>
int DirconKinematicData<T>::numForceConstraints() {
  return force_constraints_.size();
}

template <typename T>
std::shared_ptr<solvers::Constraint> DirconKinematicData<T>::getForceConstraint(int index) {
  return force_constraints_[index];
}

// Explicitly instantiates on the most common scalar types.
template class DirconKinematicData<double>;
template class DirconKinematicData<AutoDiffXd>;

}

