#include "dircon_kinematic_data.h"

namespace dairlib {

using std::shared_ptr;
using std::vector;
using drake::VectorX;
using drake::MatrixX;
using drake::solvers::Constraint;
using drake::multibody::MultibodyPlant;

template <typename T>
DirconKinematicData<T>::DirconKinematicData(const MultibodyPlant<T>& plant,
                                            int length) :
    plant_(plant),
    length_(length) {
  force_constraints_ = vector<shared_ptr<Constraint>>(0);
  c_ = VectorX<T>::Zero(length);
  cdot_ = VectorX<T>::Zero(length);
  J_ = MatrixX<T>::Zero(length, plant.num_velocities());
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
shared_ptr<Constraint> DirconKinematicData<T>::getForceConstraint(int index) {
  return force_constraints_[index];
}

// Explicitly instantiates on the most common scalar types.
template class DirconKinematicData<double>;
template class DirconKinematicData<drake::AutoDiffXd>;

}  // namespace dairlib

