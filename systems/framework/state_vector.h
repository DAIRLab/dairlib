#pragma once

#include "systems/framework/timestamped_vector.h"
#include <string>
#include <vector>


namespace dairlib {
namespace systems {

using drake::VectorX;
using std::string;
using std::vector;

/// StateVector stores the object state as a TimestampedVector
///    * positions
///    * velocities
/// Similar to OutputVector but only the state variables
template <typename T>
class StateVector : public TimestampedVector<T>  {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(StateVector)

  StateVector() = default;

  /// Initializes with the given @p size using the drake::dummy_value<T>, which
  /// is NaN when T = double.
  explicit StateVector(int num_positions, int num_velocities)
      : TimestampedVector<T>(num_positions + num_velocities),
        num_positions_(num_positions),
        num_velocities_(num_velocities),
        position_start_(0){}

  /// Constructs a StateVector with the specified positions and velocities.
  explicit StateVector(const VectorX<T>& positions,
                        const VectorX<T>& velocities)
      : StateVector(positions.size(), velocities.size()) {
    this->SetPositions(positions);
    this->SetVelocities(velocities);
  }

  void SetPositions(VectorX<T> positions) {
    this->get_mutable_data().segment(position_start_,
                                     num_positions_) = positions;
  }

  void SetVelocities(VectorX<T> velocities) {
    this->get_mutable_data().segment(position_start_ + num_positions_,
                                     num_velocities_) = velocities;
  }

  void SetPositionAtIndex(int index, T value) {
    this->SetAtIndex(position_start_ + index, value);
  }

  void SetVelocityAtIndex(int index, T value) {
    this->SetAtIndex(position_start_ + num_positions_ + index, value);
  }

  void SetState(VectorX<T> state) {
    this->get_mutable_data().segment(position_start_,
                                     num_positions_ + num_velocities_) = state;
  }

  /// Returns a const state vector
  const VectorX<T> GetState() const {
    return this->get_data().segment(position_start_,
                                    num_positions_ + num_velocities_);
  }

  /// Returns a const positions vector
  const VectorX<T> GetPositions() const {
    return this->get_data().segment(position_start_, num_positions_);
  }

  /// Returns a const velocities vector
  const VectorX<T> GetVelocities() const {
    return this->get_data().segment(position_start_ + num_positions_,
                                    num_velocities_);
  }

  /// Returns a mutable state vector
  Eigen::Map<VectorX<T>> GetMutableState() {
    auto data = this->get_mutable_data().segment(position_start_,
                                                 num_positions_ + num_velocities_);
    return Eigen::Map<VectorX<T>>(&data(0), data.size());
  }

  /// Returns a mutable positions vector
  Eigen::Map<VectorX<T>> GetMutablePositions() {
    auto data = this->get_mutable_data().segment(position_start_, num_positions_);
    return Eigen::Map<VectorX<T>>(&data(0), data.size());
  }

  /// Returns a mutable velocities vector
  Eigen::Map<VectorX<T>> GetMutableVelocities() {
    auto data = this->get_mutable_data().segment(
        position_start_ + num_positions_, num_velocities_);
    return Eigen::Map<VectorX<T>>(&data(0), data.size());
  }

  T GetPositionAtIndex(int index) const {
    return this->GetAtIndex(position_start_ + index);
  }

  T GetVelocityAtIndex(int index) const {
    return this->GetAtIndex(position_start_ + num_positions_ + index);
  }

  void SetName(int index, string name) {
    position_names_[index] = name;
  }

  string GetName(int index) {
    return position_names_[index];
  }

 protected:
  virtual StateVector<T>* DoClone() const {
    return new StateVector<T>(num_positions_, num_velocities_);
  }

 private:
  const int num_positions_;
  const int num_velocities_;
  const int position_start_;
  vector<string> position_names_;
};

}  // namespace systems
}  // namespace dairlib
