// Instantiates FrankaKinematicsVector for default scalar types.

#include "examples/plate-balancing/systems/franka_kinematics_vector.h"

#include "drake/common/default_scalars.h"

namespace dairlib {
namespace examples {
namespace plate_balancing {
namespace systems {

// Default constructor
template <typename T>
FrankaKinematicsVector<T>::FrankaKinematicsVector(
    int num_end_effector_positions, int num_object_positions,
    int num_end_effector_velocities, int num_object_velocities)
    : TimestampedVector<T>(num_end_effector_positions + num_object_positions +
                           num_end_effector_velocities + num_object_velocities),
      num_end_effector_positions_(num_end_effector_positions),
      num_object_positions_(num_object_positions),
      num_end_effector_velocities_(num_end_effector_velocities),
      num_object_velocities_(num_object_velocities),
      end_effector_positions_start_(0),
      object_positions_start_(num_end_effector_positions),
      end_effector_velocities_start_(num_end_effector_positions +
                                     num_object_positions),
      object_velocities_start_(num_end_effector_positions +
                               num_object_positions +
                               num_end_effector_velocities),
      num_positions_(num_end_effector_positions + num_object_positions),
      num_velocities_(num_end_effector_velocities + num_object_velocities) {}

// Construct from vectors
template <typename T>
FrankaKinematicsVector<T>::FrankaKinematicsVector(
    const drake::VectorX<T>& end_effector_positions,
    const drake::VectorX<T>& object_positions,
    const drake::VectorX<T>& end_effector_velocities,
    const drake::VectorX<T>& object_velocities)
    : FrankaKinematicsVector<T>(
          end_effector_positions.size(), object_positions.size(),
          end_effector_velocities.size(), object_velocities.size()) {
  SetEndEffectorPositions(end_effector_positions);
  SetObjectPositions(object_positions);
  SetEndEffectorVelocities(end_effector_velocities);
  SetObjectVelocities(object_velocities);
}

// Setters
template <typename T>
void FrankaKinematicsVector<T>::SetEndEffectorPositions(
    drake::VectorX<T> positions) {
  this->get_mutable_data().segment(end_effector_positions_start_,
                                   num_end_effector_positions_) = positions;
}

template <typename T>
void FrankaKinematicsVector<T>::SetObjectPositions(
    drake::VectorX<T> positions) {
  this->get_mutable_data().segment(object_positions_start_,
                                   num_object_positions_) = positions;
}

template <typename T>
void FrankaKinematicsVector<T>::SetEndEffectorVelocities(
    drake::VectorX<T> velocities) {
  this->get_mutable_data().segment(end_effector_velocities_start_,
                                   num_end_effector_velocities_) = velocities;
}

template <typename T>
void FrankaKinematicsVector<T>::SetObjectVelocities(
    drake::VectorX<T> velocities) {
  this->get_mutable_data().segment(object_velocities_start_,
                                   num_object_velocities_) = velocities;
}

template <typename T>
void FrankaKinematicsVector<T>::SetState(drake::VectorX<T> state) {
  DRAKE_DEMAND(state.size() == this->data_size());
  this->get_mutable_data() = state;
}

// Getters
template <typename T>
const drake::VectorX<T> FrankaKinematicsVector<T>::GetState() const {
  return this->get_data();
}

template <typename T>
const drake::VectorX<T> FrankaKinematicsVector<T>::GetEndEffectorPositions()
    const {
  return this->get_data().segment(end_effector_positions_start_,
                                  num_end_effector_positions_);
}

template <typename T>
const drake::VectorX<T> FrankaKinematicsVector<T>::GetObjectPositions() const {
  return this->get_data().segment(object_positions_start_,
                                  num_object_positions_);
}

template <typename T>
const drake::VectorX<T> FrankaKinematicsVector<T>::GetEndEffectorVelocities()
    const {
  return this->get_data().segment(end_effector_velocities_start_,
                                  num_end_effector_velocities_);
}

template <typename T>
const drake::VectorX<T> FrankaKinematicsVector<T>::GetObjectVelocities() const {
  return this->get_data().segment(object_velocities_start_,
                                  num_object_velocities_);
}

template <typename T>
const drake::VectorX<T> FrankaKinematicsVector<T>::GetVelocities() const {
  return this->get_data().segment(end_effector_velocities_start_,
                                  num_velocities_);
}

template <typename T>
const drake::VectorX<T> FrankaKinematicsVector<T>::GetPositions() const {
  return this->get_data().segment(end_effector_positions_start_,
                                  num_positions_);
}

// Mutable accessors
template <typename T>
Eigen::Map<drake::VectorX<T>> FrankaKinematicsVector<T>::GetMutablePositions() {
  auto data = this->get_mutable_data().segment(end_effector_positions_start_,
                                               num_positions_);
  return Eigen::Map<drake::VectorX<T>>(&data(0), data.size());
}

template <typename T>
Eigen::Map<drake::VectorX<T>>
FrankaKinematicsVector<T>::GetMutableVelocities() {
  auto data = this->get_mutable_data().segment(end_effector_velocities_start_,
                                               num_velocities_);
  return Eigen::Map<drake::VectorX<T>>(&data(0), data.size());
}

template <typename T>
Eigen::Map<drake::VectorX<T>> FrankaKinematicsVector<T>::GetMutableState() {
  auto data = this->get_mutable_data().segment(end_effector_positions_start_,
                                               this->data_size());
  return Eigen::Map<drake::VectorX<T>>(&data(0), data.size());
}

// Clone
template <typename T>
FrankaKinematicsVector<T>* FrankaKinematicsVector<T>::DoClone() const {
  return new FrankaKinematicsVector<T>(
      num_end_effector_positions_, num_object_positions_,
      num_end_effector_velocities_, num_object_velocities_);
}

}  // namespace systems
}  // namespace plate_balancing
}  // namespace examples
}  // namespace dairlib

// Explicit template instantiation for default scalars
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::dairlib::examples::plate_balancing::systems::FrankaKinematicsVector)
