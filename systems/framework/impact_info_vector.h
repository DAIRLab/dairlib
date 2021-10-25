#pragma once

#include <string>
#include <vector>

#include "systems/framework/timestamped_vector.h"

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
class ImpactInfoVector : public TimestampedVector<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ImpactInfoVector)

  ImpactInfoVector() = default;

  /// Initializes with the given @p size using the drake::dummy_value<T>, which
  /// is NaN when T = double.
  explicit ImpactInfoVector(int num_contact, int num_holonomic, int space_dim)
      : TimestampedVector<T>(3 + num_contact + num_holonomic),
        num_contact_impulses_(num_contact),
        num_holonomic_impulses_(num_holonomic),
        space_dim_(space_dim),
        contact_mode_index_(0),
        alpha_pre_index_(1),
        alpha_post_index_(2),
        impulse_start_(3) {}

  void SetCurrentContactMode(int contact_mode) {
    this->get_mutable_data()(contact_mode_index_) = contact_mode;
  }

  void SetAlphaPre(double alpha_pre) {
    this->get_mutable_data()(alpha_pre_index_) = alpha_pre;
  }

  void SetAlphaPost(double alpha_post) {
    this->get_mutable_data()(alpha_post_index_) = alpha_post;
  }

  void SetImpulseVector(VectorX<T> impulse) {
    this->get_mutable_data().segment(
        impulse_start_, num_contact_impulses_ + num_holonomic_impulses_) =
        impulse;
  }

  const T GetAlphaPre() const {
    return this->GetAtIndex(alpha_pre_index_);
  }

  const T GetAlphaPost() const {
    return this->GetAtIndex(alpha_post_index_);
  }

  const T GetCurrentContactMode() const {
    return this->GetAtIndex(contact_mode_index_);
  }

  /// Returns a const impulse vector
  const VectorX<T> GetImpulseVector() const {
    return this->get_data().segment(
        impulse_start_, num_contact_impulses_ + num_holonomic_impulses_);
  }

  const VectorX<T> GetContactImpulseAtContactIndex(int contact_index) const {
    return this->get_data().segment(impulse_start_ + contact_index * space_dim_, space_dim_);
  }

  const T GetContactImpulseAtIndex(int index) const {
    return this->GetAtIndex(impulse_start_ + index);
  }

  void SetName(int index, string name) { impulse_names_[index] = name; }

  string GetName(int index) { return impulse_names_[index]; }

 protected:
  virtual ImpactInfoVector<T>* DoClone() const {
    return new ImpactInfoVector<T>(num_contact_impulses_, num_holonomic_impulses_, space_dim_);
  }

 private:
  const int num_contact_impulses_;
  const int num_holonomic_impulses_;
  const int space_dim_;
  const int alpha_pre_index_;
  const int alpha_post_index_;
  const int contact_mode_index_;
  const int impulse_start_;
  vector<string> impulse_names_;
};

}  // namespace systems
}  // namespace dairlib
