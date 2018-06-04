#pragma once

#include "drake/systems/framework/basic_vector.h"

namespace dairlib {
namespace systems {

using drake::systems::BasicVector;
using drake::VectorX;

/// TimestampedVector wraps a BasicVector along with a timestamp field
/// The primary purpose of this is to pass-through a message (e.g. LCM) timestamp
template <typename T>
class TimestampedVector : public drake::systems::BasicVector<T>  {
public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TimestampedVector)

  /// Constructs an empty TimestampedVector.
  TimestampedVector() = default;


  /// Initializes with the given @p size using the drake::dummy_value<T>, which
  /// is NaN when T = double.
  explicit TimestampedVector(int size)
      : BasicVector<T>(size), timestamp_(0) {}

  /// Constructs a TimestampedVector with the specified @p data.
  explicit TimestampedVector(const VectorX<T>& data) 
      : BasicVector<T>(data), timestamp_(0) {}

  /// Constructs a TimestampedVector whose elements are the elements of @p data.
  TimestampedVector(const std::initializer_list<T>& data)
      : BasicVector<T>(data), timestamp_(0) {}

  void set_timestamp(double timestamp) {
    timestamp_ = timestamp;
  }

  double get_timestamp() const {return timestamp_;}

  /// Copies the entire vector to a new TimestampedVector, with the same concrete
  /// implementation type.
  ///
  /// Uses the Non-Virtual Interface idiom because smart pointers do not have
  /// type covariance.
  std::unique_ptr<TimestampedVector<T>> Clone() const {
    auto clone = std::unique_ptr<TimestampedVector<T>>(DoClone());
    clone->set_value(this->get_value());
    clone->set_timestamp(this->get_timestamp());
    return clone;
  }

 protected:
  /// Returns a new TimestampedVector containing a copy of the entire vector.
  /// Caller must take ownership, and may rely on the NVI wrapper to initialize
  /// the clone elementwise.
  ///
  /// Subclasses of TimestampedVector must override DoClone to return their covariant
  /// type.
  ///
  /// mposa: This doesn't appear to actually clone the object
  virtual TimestampedVector<T>* DoClone() const {
    return new TimestampedVector<T>(this->size());
  }

  private:
    double timestamp_;
  };

}  // namespace systems
}  // namespace dairlib
