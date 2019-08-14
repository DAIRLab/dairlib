#pragma once

/// @file
/// Template method implementations for pass_through.h.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

/* clang-format off to disable clang-format-includes */
#include "systems/primitives/timestamped_subvector_pass_through.h"
/* clang-format on */

#include <memory>
#include <utility>

namespace dairlib {
namespace systems {

template <typename T>
TSSubvectorPassThrough<T>::TSSubvectorPassThrough(int vector_size, int start,
      int length)
    : drake::systems::LeafSystem<T>(
        drake::systems::SystemTypeTag<TSSubvectorPassThrough>()),
      start_(start), length_(length) {
  DRAKE_DEMAND(vector_size != -1);
  systems::TimestampedVector<T> input(vector_size);
  systems::TimestampedVector<T> output(length);
  this->DeclareVectorInputPort(input);
  this->DeclareVectorOutputPort(
      output, &TSSubvectorPassThrough::DoCalcVectorOutput);
}

template <typename T>
template <typename U>
TSSubvectorPassThrough<T>::TSSubvectorPassThrough(
    const TSSubvectorPassThrough<U>& other)
    : TSSubvectorPassThrough(other.get_input_port().size(), other.start_,
                           other.length_) {}

template <typename T>
void TSSubvectorPassThrough<T>::DoCalcVectorOutput(
      const drake::systems::Context<T>& context,
      systems::TimestampedVector<T>* output) const {
  const systems::TimestampedVector<T>& input =
      *this->template EvalVectorInput<TimestampedVector>(context, 0);
  output->SetDataVector(input.get_data().segment(start_, length_));
  output->set_timestamp(input.get_timestamp());
}

}  // namespace systems
}  // namespace dairlib
