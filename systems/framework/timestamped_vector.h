#pragma once

#include "drake/systems/framework/basic_vector.h"

namespace dairlib {
namespace systems {

using drake::systems::BasicVector;
using drake::VectorX;

/// TimestampedVector wraps a BasicVector along with a timestamp field
/// The primary purpose of this is to pass-through a message (e.g. LCM) timestamp
/// Uses a length N+1 BasicVector to store a vector of length N and a timestamp
/// The timestamp is stored as the final element (Nth)
template <typename T>
class TimestampedVector : public drake::systems::BasicVector<T>  {
public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TimestampedVector)

  /// Constructs an empty TimestampedVector.
  TimestampedVector() = default;


  /// Initializes with the given @p size using the drake::dummy_value<T>, which
  /// is NaN when T = double.
  explicit TimestampedVector(int size)
      : BasicVector<T>(size + 1) {}

  /// Constructs a TimestampedVector with the specified @p data.
  explicit TimestampedVector(const VectorX<T>& data) 
      : TimestampedVector(data.size()) {
    VectorX<T> data_timestamped = VectorX<T>(data.size() + 1);
    data_timestamped.head(data.size()) = data;
    data_timestamped(data.size()) = 0;
    this->SetFromVector(data_timestamped);
  }

  /// Constructs a TimestampedVector whose elements are the elements of @p data.
  TimestampedVector(const std::initializer_list<T>& data)
      : TimestampedVector<T>(data.size()) {
    int i = 0;
    for (const T& datum : data) {
      this->SetAtIndex(i++, datum);
    }
  }

  void set_timestamp(T timestamp) {
    this->SetAtIndex(this->size()-1, timestamp); 
  }

  T get_timestamp() const {return this->GetAtIndex(this->size()-1);}

  /// Copies the entire vector to a new TimestampedVector, with the same concrete
  /// implementation type.
  ///
  /// Uses the Non-Virtual Interface idiom because smart pointers do not have
  /// type covariance.
  std::unique_ptr<TimestampedVector<T>> Clone() const {
    auto clone = std::unique_ptr<TimestampedVector<T>>(DoClone());
    clone->set_value(this->get_value());
    return clone;
  }

  /// Returns the vector without the timestamp
  VectorX<T> CopyVectorNoTimestamp() const {
    return this->CopyToVector().head(this->size()-1);
  }

  /// Returns a mutable vector of the data values (without timestamp)
  Eigen::VectorBlock<VectorX<T>> get_mutable_data() {
    return this->get_mutable_value().head(this->size() - 1);
  }

  void SetDataVector(const Eigen::Ref<const VectorX<T>>& value) {
    this->get_mutable_data() = value;
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
  };

}  // namespace systems
}  // namespace dairlib
