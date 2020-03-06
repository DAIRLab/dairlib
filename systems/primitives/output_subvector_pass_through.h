#pragma once

#include <memory>

#include "systems/framework/output_vector.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace systems {

/// This class is copied from drake/systems/primitives/PassThrough
/// with the modification that it only passes through a subset of the vector
/// Unliked SubvectorPassThrough, this is designed for use with
/// TimeStampedVectors. It will pass through the subvector AND the timestamp.
template <typename T>
class OutputSubvectorPassThrough final : public drake::systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(OutputSubvectorPassThrough)

  /// Constructs a pass through system (`y = u.segment(start,length)`).
  /// @param vector_size the length of the input vector
  /// @param start the initial index of the subvector
  /// @param length number of elements in the subvector
  explicit OutputSubvectorPassThrough(int num_positions, int num_velocities,
                                      int num_inputs, int pos_start,
                                      int pos_length, int vel_start,
                                      int vel_length, int input_start,
                                      int input_length);

  /// Scalar-type converting copy constructor.
  /// See @ref system_scalar_conversion.
  template <typename U>
  explicit OutputSubvectorPassThrough(const OutputSubvectorPassThrough<U>&);

  virtual ~OutputSubvectorPassThrough() {}

  /// Returns the sole input port.
  const drake::systems::InputPort<T>& get_input_port() const {
    return drake::systems::LeafSystem<T>::get_input_port(0);
  }

  // Don't use the indexed get_input_port when calling this system directly.
  void get_input_port(int) = delete;

  /// Returns the sole output port.
  const drake::systems::OutputPort<T>& get_output_port() const {
    return drake::systems::LeafSystem<T>::get_output_port(0);
  }

  // Don't use the indexed get_output_port when calling this system directly.
  void get_output_port(int) = delete;

 protected:
  void DoCalcVectorOutput(const drake::systems::Context<T>& context,
                          systems::OutputVector<T>* output) const;

 private:
  bool is_abstract() const { return false; }

  // Allow different specializations to access each other's private data.
  template <typename U>
  friend class OutputSubvectorPassThrough;


  int num_positions_;
  int num_velocities_;
  int num_inputs_;
  int pos_start_;
  int pos_length_;
  int vel_start_;
  int vel_length_;
  int input_start_;
  int input_length_;
};

}  // namespace systems
}  // namespace dairlib
