#pragma once

/// @file
/// Template method implementations for output_subvector_pass_through.h.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

/* clang-format off to disable clang-format-includes */
#include "systems/primitives/output_subvector_pass_through.h"
/* clang-format on */

#include <memory>
#include <utility>

namespace dairlib {
namespace systems {

template <typename T>
OutputSubvectorPassThrough<T>::OutputSubvectorPassThrough(
    int num_positions, int num_velocities, int num_inputs, int pos_start,
    int pos_length, int vel_start, int vel_length, int input_start,
    int input_length)
    : drake::systems::LeafSystem<T>(
          drake::systems::SystemTypeTag<OutputSubvectorPassThrough>()),
      num_positions_(num_positions),
      num_velocities_(num_velocities),
      num_inputs_(num_inputs),
      pos_start_(pos_start),
      pos_length_(pos_length),
      vel_start_(vel_start),
      vel_length_(vel_length),
      input_start_(input_start),
      input_length_(input_length) {
  systems::OutputVector<T> input(num_positions, num_velocities, num_inputs);
  systems::OutputVector<T> output(pos_length, vel_length, input_length);
  this->DeclareVectorInputPort(input);
  this->DeclareVectorOutputPort(
      output, &OutputSubvectorPassThrough::DoCalcVectorOutput);
}

template <typename T>
template <typename U>
OutputSubvectorPassThrough<T>::OutputSubvectorPassThrough(
    const OutputSubvectorPassThrough<U>& other)
    : OutputSubvectorPassThrough(
          other.num_positions_, other.num_velocities_, other.num_inputs_,
          other.pos_start_, other.pos_length_, other.vel_start_,
          other.vel_length_, other.input_start_, other.input_length_) {}

template <typename T>
void OutputSubvectorPassThrough<T>::DoCalcVectorOutput(
    const drake::systems::Context<T>& context,
    systems::OutputVector<T>* output) const {
  const systems::OutputVector<T>& input =
      *this->template EvalVectorInput<OutputVector>(context, 0);
  output->SetPositions(input.get_data().segment(pos_start_, pos_length_));
  output->SetVelocities(input.get_data().segment(vel_start_, vel_length_));
  output->SetEfforts(input.get_data().segment(input_start_, input_length_));
  output->SetIMUAccelerations(input.GetIMUAccelerations());
  output->set_timestamp(input.get_timestamp());
}

}  // namespace systems
}  // namespace dairlib
