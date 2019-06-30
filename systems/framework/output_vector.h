#pragma once

#include "systems/framework/timestamped_vector.h"
#include <string>
#include <vector>


namespace dairlib {
namespace systems {

using drake::VectorX;
using std::string;
using std::vector;

/// OutputVector stores the robot output as a TimestampedVector
///    * positions
///    * velocities
///    * efforts
///    * imu accelerations
/// Can be later extended if more information is desired in here
template <typename T>
class OutputVector : public TimestampedVector<T>  {
public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(OutputVector)

  OutputVector() = default;

  /// Initializes with the given @p size using the drake::dummy_value<T>, which
  /// is NaN when T = double.
  explicit OutputVector(int num_positions, int num_velocities, int num_efforts)
      : TimestampedVector<T>(num_positions + num_velocities + num_efforts + 3),
      num_positions_(num_positions),
      num_velocities_(num_velocities),
      num_efforts_(num_efforts),
      position_start_(0){}

  /// Constructs a OutputVector with the specified positions and velocities.
  explicit OutputVector(const VectorX<T>& positions,
                        const VectorX<T>& velocities,
                        const VectorX<T>& efforts)
      : OutputVector(positions.size(), velocities.size(), efforts.size()) {
    this->SetPositions(positions);
    this->SetVelocities(velocities);
    this->SetEfforts(efforts);
  }

  void SetPositions(VectorX<T> positions) {
    this->get_mutable_data().segment(position_start_,
                                     num_positions_) = positions;
  }

  void SetVelocities(VectorX<T> velocities) {
    this->get_mutable_data().segment(position_start_ + num_positions_,
                                     num_velocities_) = velocities;
  }

  void SetEfforts(VectorX<T> efforts) {
    this->get_mutable_data().segment(position_start_ + num_positions_ +
                                     num_velocities_, num_efforts_) = efforts;
  }

  void SetIMUAccelerations(VectorX<T> imu_accelerations) {
    this->get_mutable_data().segment(position_start_ + num_positions_  +
                                     num_velocities_ + num_efforts_,
                                     3) = imu_accelerations;
  }

  void SetEffortAtIndex(int index, T value) {
    this->SetAtIndex(position_start_ + num_positions_ + num_velocities_ +
          index, value);
  }

  void SetPositionAtIndex(int index, T value) {
    this->SetAtIndex(position_start_ + index, value);
  }

  void SetVelocityAtIndex(int index, T value) {
    this->SetAtIndex(position_start_ + num_positions_ + index, value);
  }

  void SetIMUAccelerationAtIndex(int index, T value) {
    this->SetAtIndex(position_start_ + num_positions_ + num_velocities_ + 
                     num_efforts_ + index, value);
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

  /// Returns a const velocities vector
  const VectorX<T> GetEfforts() const {
    return this->get_data().segment(position_start_ + num_positions_ +
                                    num_velocities_, num_efforts_);
  }

  /// Returns a const imu accelerations vectors
  const VectorX<T> GetIMUAccelerations() const {
    return this->get_data().segment(position_start_ + num_positions_ + 
                                    num_velocities_ + num_efforts_, 3);
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

  /// Returns a mutable efforts vector
  Eigen::Map<VectorX<T>> GetMutableEfforts() {
    auto data = this->get_mutable_data().segment(
        position_start_ + num_positions_ + num_velocities_, num_efforts_);
    return Eigen::Map<VectorX<T>>(&data(0), data.size());
  }

  /// Returns a mutable imu acceleration vector
  Eigen::Map<VectorX<T>> GetMutableIMUAccelerations() {
    auto data = this->get_mutable_data().segment(
        position_start_ + num_positions_ + num_velocities_ + num_efforts_, 3);
    return Eigen::Map<VectorX<T>>(&data(0), data.size());
  }

  T GetPositionAtIndex(int index) const {
    return this->GetAtIndex(position_start_ + index);
  }

  T GetVelocityAtIndex(int index) const {
    return this->GetAtIndex(position_start_ + num_positions_ + index);
  }

  T GetIMUAccelerationAtIndex(int index) const {
    return this->GetAtIndex(position_start_ + num_positions_ + 
                            num_velocities_ + num_efforts_ + index);
  }


  void SetName(int index, string name) {
    position_names_[index] = name;
  }

  string GetName(int index) {
    return position_names_[index];
  }

protected:
  virtual OutputVector<T>* DoClone() const {
    return new OutputVector<T>(num_positions_, num_velocities_, num_efforts_);
  }

private:
  const int num_positions_;
  const int num_velocities_;
  const int num_efforts_;
  const int position_start_;
  vector<string> position_names_;
};

}  // namespace systems
}  // namespace dairlib
