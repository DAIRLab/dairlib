#pragma once

#include <string>
#include <vector>

#include "systems/framework/timestamped_vector.h"

namespace dairlib {
namespace systems {

using drake::VectorX;
using std::string;
using std::vector;

/*!
 * @brief
 * TimestampedVector consumed by the OSC that contains relevant information
 * about the anticipated impact event.
 * This includes:
 * contact_mode: contact mode as defined by the OSC
 * alpha: blending coefficient for the impact-invariant projection
 * impulse: anticipated contact impulse (unused currently)
 *
 */
template <typename T>
class ImpactInfoVector : public TimestampedVector<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ImpactInfoVector)

  ImpactInfoVector() = default;

  explicit ImpactInfoVector(int num_contact, int num_holonomic, int space_dim)
      : TimestampedVector<T>(3 + num_contact + num_holonomic),
        has_impulse_info_(num_contact != 0),
        num_contact_impulses_(num_contact),
        num_holonomic_impulses_(num_holonomic),
        space_dim_(space_dim) {}

  void SetCurrentContactMode(int contact_mode) {
    this->get_mutable_data()(contact_mode_index_) = contact_mode;
  }

  void SetAlpha(double alpha) {
    this->get_mutable_data()(alpha_index_) = alpha;
  }

  void SetImpulseVector(VectorX<T> impulse) {
    this->get_mutable_data().segment(
        impulse_start_, num_contact_impulses_ + num_holonomic_impulses_) =
        impulse;
  }

  const T GetAlpha() const { return this->GetAtIndex(alpha_index_); }

  const T GetCurrentContactMode() const {
    return this->GetAtIndex(contact_mode_index_);
  }

  /// Returns a const impulse vector
  const VectorX<T> GetImpulseVector() const {
    return this->get_data().segment(
        impulse_start_, num_contact_impulses_ + num_holonomic_impulses_);
  }

  const VectorX<T> GetContactImpulseAtContactIndex(int contact_index) const {
    return this->get_data().segment(impulse_start_ + contact_index * space_dim_,
                                    space_dim_);
  }

  const T GetContactImpulseAtIndex(int index) const {
    return this->GetAtIndex(impulse_start_ + index);
  }

  void SetName(int index, string name) { impulse_names_[index] = name; }

  string GetName(int index) { return impulse_names_[index]; }

 protected:
  virtual ImpactInfoVector<T>* DoClone() const {
    return new ImpactInfoVector<T>(num_contact_impulses_,
                                   num_holonomic_impulses_, space_dim_);
  }

 private:
  bool has_impulse_info_;
  const int num_contact_impulses_;
  const int num_holonomic_impulses_;
  const int space_dim_;
  const int contact_mode_index_ = 0;
  const int alpha_index_ = 1;
  const int impulse_start_ = 2;
  vector<string> impulse_names_;
};

}  // namespace systems
}  // namespace dairlib
