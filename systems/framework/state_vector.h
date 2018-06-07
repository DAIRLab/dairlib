#pragma once

#include "systems/framework/timestamped_vector.h"

namespace dairlib {
namespace systems {

using drake::VectorX;

/// TimestampedVector wraps a BasicVector along with a timestamp field
/// The primary purpose of this is to pass-through a message (e.g. LCM) timestamp
/// Uses a length N+1 BasicVector to store a vector of length N and a timestamp
/// The timestamp is stored as the final element (Nth)
template <typename T>
class StateVector : public TimestampedVector<T>  {
public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TimestampedVector)

  StateVector() = default;

  /// Initializes with the given @p size using the drake::dummy_value<T>, which
  /// is NaN when T = double.
  explicit StateVector(int num_positions, int num_velocities)
      : TimestampedVector<T>(num_positions + num_velocities),
      num_positions_(num_positions),
      num_velocities_(num_velocities),
      position_start_(0),
      velocity_start_(num_positions_) {}

  /// Constructs a StateVector with the specified positions and velocities.
  explicit StateVector(const VectorX<T>& positions,
                       const VectorX<T>& velocities)
      : StateVector(positions.size(), velocities.size()) {
    this->SetPositions(positions);
    this->SetVelocities(positions);
  }

  void SetPositions(VectorX<T> positions) {
    this->get_mutable_data().segment(position_start_, num_positions_) = positions;
  }

  void SetVelocities(VectorX<T> positions) {
    this->get_mutable_data().segment(velocity_start_, num_velocities_) = positions;
  }

  void SetPositionAtIndex(int index, T value) {
    this->SetAtIndex(position_start_ + index, value);
  }

  void SetVelocityAtIndex(int index, T value) {
    this->SetAtIndex(velocity_start_ + index, value);
  }

  void SetState(VectorX<T> state) {
    this->get_mutable_data().segment(position_start_, num_positions_ + num_velocities_) = positions;
  }

private:
  const int num_positions_;
  const int num_velocities_;
  const int position_start_;
  const int velocity_start_;
};