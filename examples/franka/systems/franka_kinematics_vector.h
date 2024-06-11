#pragma once

#include <string>
#include <vector>

#include "systems/framework/timestamped_vector.h"

namespace dairlib {
namespace systems {

/// FrankaKinematicsVector stores the robot output as a TimestampedVector
///    * positions
///    * velocities
template <typename T>
class FrankaKinematicsVector : public TimestampedVector<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FrankaKinematicsVector)

  FrankaKinematicsVector() = default;

  explicit FrankaKinematicsVector(int num_end_effector_positions,
                                  int num_object_positions,
                                  int num_end_effector_velocities,
                                  int num_object_velocities)
      : TimestampedVector<T>(num_end_effector_positions + num_object_positions +
                             num_end_effector_velocities +
                             num_object_velocities),
        num_end_effector_positions_(num_end_effector_positions),
        num_object_positions_(num_object_positions),
        num_end_effector_velocities_(num_end_effector_velocities),
        num_object_velocities_(num_object_velocities),
        end_effector_positions_start_(0),
        object_positions_start_(num_end_effector_positions_),
        end_effector_velocities_start_(num_end_effector_positions_ +
                                       num_object_positions_),
        object_velocities_start_(num_end_effector_positions_ +
                                 num_object_positions_ +
                                 num_end_effector_velocities_),
        num_positions_(num_end_effector_positions_ + num_object_positions_),
        num_velocities_(num_end_effector_velocities_ + num_object_velocities_) {
  }

  /// Constructs a OutputVector with the specified positions and velocities.
  explicit FrankaKinematicsVector(
      const drake::VectorX<T>& end_effector_positions,
      const drake::VectorX<T>& object_positions,
      const drake::VectorX<T>& end_effector_velocities,
      const drake::VectorX<T>& object_velocities)
      : FrankaKinematicsVector(
            end_effector_positions.size(), object_positions.size(),
            end_effector_velocities.size(), object_velocities.size()) {
    this->SetEndEffectorPositions(end_effector_positions);
    this->SetObjectPositions(object_positions);
    this->SetEndEffectorVelocities(end_effector_velocities);
    this->SetObjectVelocities(object_velocities);
  }

  void SetEndEffectorPositions(drake::VectorX<T> positions) {
    DRAKE_DEMAND(positions.size() == num_end_effector_positions_);
    this->get_mutable_data().segment(end_effector_positions_start_,
                                     num_end_effector_positions_) = positions;
  }

  void SetObjectPositions(drake::VectorX<T> positions) {
    DRAKE_DEMAND(positions.size() == num_object_positions_);
    this->get_mutable_data().segment(object_positions_start_,
                                     num_object_positions_) = positions;
  }

  void SetEndEffectorVelocities(drake::VectorX<T> velocities) {
    DRAKE_DEMAND(velocities.size() == num_end_effector_velocities_);
    this->get_mutable_data().segment(end_effector_velocities_start_,
                                     num_end_effector_velocities_) = velocities;
  }

  void SetObjectVelocities(drake::VectorX<T> velocities) {
    DRAKE_DEMAND(velocities.size() == num_object_velocities_);
    this->get_mutable_data().segment(object_velocities_start_,
                                     num_object_velocities_) = velocities;
  }

  void SetState(drake::VectorX<T> state) {
    DRAKE_DEMAND(state.size() == this->data_size());
    this->get_mutable_data().segment(end_effector_positions_start_,
                                     this->data_size()) = state;
  }

  /// Returns a const state vector
  const drake::VectorX<T> GetState() const {
    return this->get_data().segment(end_effector_positions_start_,
                                    this->data_size());
  }

  /// Returns a const positions vector for the end effector
  const drake::VectorX<T> GetEndEffectorPositions() const {
    return this->get_data().segment(end_effector_positions_start_,
                                    num_end_effector_positions_);
  }

  /// Returns a const positions vector for the object
  const drake::VectorX<T> GetObjectPositions() const {
    return this->get_data().segment(object_positions_start_,
                                    num_object_positions_);
  }

  /// Returns a const positions vector for the end effector
  const drake::VectorX<T> GetEndEffectorVelocities() const {
    return this->get_data().segment(end_effector_velocities_start_,
                                    num_end_effector_velocities_);
  }

  /// Returns a const positions vector for the object
  const drake::VectorX<T> GetObjectVelocities() const {
    return this->get_data().segment(object_velocities_start_,
                                    num_object_velocities_);
  }

  /// Returns a const velocities vector
  const drake::VectorX<T> GetVelocities() const {
    return this->get_data().segment(
        end_effector_velocities_start_,
        num_end_effector_velocities_ + num_object_velocities_);
  }

  /// Returns a const positions vector
  const drake::VectorX<T> GetPositions() const {
    return this->get_data().segment(
        end_effector_positions_start_,
        num_end_effector_positions_ + num_object_positions_);
  }

  /// Returns a mutable positions vector
  Eigen::Map<drake::VectorX<T>> GetMutablePositions() {
    auto data = this->get_mutable_data().segment(
        end_effector_positions_start_,
        num_end_effector_positions_ + num_object_positions_);
    return Eigen::Map<drake::VectorX<T>>(&data(0), data.size());
  }

  /// Returns a mutable velocities vector
  Eigen::Map<drake::VectorX<T>> GetMutableVelocities() {
    auto data = this->get_mutable_data().segment(
        end_effector_velocities_start_,
        num_end_effector_velocities_ + num_object_velocities_);
    return Eigen::Map<drake::VectorX<T>>(&data(0), data.size());
  }

  /// Returns a mutable state vector
  Eigen::Map<drake::VectorX<T>> GetMutableState() {
    auto data = this->get_mutable_data().segment(end_effector_positions_start_,
                                                 this->data_size());
    return Eigen::Map<drake::VectorX<T>>(&data(0), data.size());
  }

 protected:
  virtual FrankaKinematicsVector<T>* DoClone() const {
    return new FrankaKinematicsVector<T>(
        num_end_effector_positions_, num_object_positions_,
        num_end_effector_velocities_, num_object_velocities_);
  }

 private:
  const int num_end_effector_positions_;
  const int num_object_positions_;
  const int num_end_effector_velocities_;
  const int num_object_velocities_;
  const int end_effector_positions_start_;
  const int object_positions_start_;
  const int end_effector_velocities_start_;
  const int object_velocities_start_;

  const int num_positions_;
  const int num_velocities_;
};

}  // namespace systems
}  // namespace dairlib
