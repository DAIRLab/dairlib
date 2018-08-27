#pragma once

/// @file
/// Template method implementations for pass_through.h.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

/* clang-format off to disable clang-format-includes */
#include "systems/primitives/subvector_pass_through.h"
/* clang-format on */

#include <memory>
#include <utility>

namespace dairlib {
namespace systems {

template <typename T>
SubvectorPassThrough<T>::SubvectorPassThrough(int vector_size, int start,
      int length)
    : drake::systems::LeafSystem<T>(
        drake::systems::SystemTypeTag<SubvectorPassThrough>()),
      start_(start), length_(length) {
  DRAKE_DEMAND(vector_size != -1);
  drake::systems::BasicVector<T> input(vector_size);
  drake::systems::BasicVector<T> output(length);
  this->DeclareVectorInputPort(input);
  this->DeclareVectorOutputPort(
      output, &SubvectorPassThrough::DoCalcVectorOutput);
}

template <typename T>
template <typename U>
SubvectorPassThrough<T>::SubvectorPassThrough(
    const SubvectorPassThrough<U>& other)
    : SubvectorPassThrough(other.get_input_port().size(), other.start_,
                           other.length_) {}

template <typename T>
void SubvectorPassThrough<T>::DoCalcVectorOutput(
      const drake::systems::Context<T>& context,
      drake::systems::BasicVector<T>* output) const {
  const drake::systems::BasicVector<T>& input =
      *this->EvalVectorInput(context, 0);
  output->SetFromVector(input.get_value().segment(start_, length_));
}

template <typename T>
drake::optional<bool> SubvectorPassThrough<T>::DoHasDirectFeedthrough(
    int input_port, int output_port) const {
  DRAKE_DEMAND(input_port == 0);
  DRAKE_DEMAND(output_port == 0);
  // By definition, a pass-through will have direct feedthrough, as the
  // output depends directly on the input.
  return true;
}

}  // namespace systems
}  // namespace dairlib
