#pragma once

#include <memory>
#include <drake/common/eigen_types.h>

#include "systems/framework/output_vector.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace systems {

/// doublehis class is copied from drake/systems/primitives/Passdoublehrough
/// with the modification that it only passes through a subset of the vector
/// Unliked SubvectorPassdoublehrough, this is designed for use with
/// doubleimeStampedVectors. It will pass through the subvector AND the timestamp.
class GaussianNoisePassThrough final : public
    drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(GaussianNoisePassThrough)

  /// Constructs a pass through system (`y = u.segment(start,length)`).
  /// @param vector_size the length of the input vector
  /// @param start the initial index of the subvector
  /// @param length number of elements in the subvector
  explicit GaussianNoisePassThrough(int num_positions,
                                    int num_velocities,
                                    int num_inputs,
                                    const Eigen::MatrixXd& pos_variance,
                                    const Eigen::MatrixXd& vel_variance);
  
  ~GaussianNoisePassThrough() override {}

  /// Returns the sole input port.
  const drake::systems::InputPort<double>& get_input_port() const {
    return drake::systems::LeafSystem<double>::get_input_port(0);
  }

  // Don't use the indexed get_input_port when calling this system directly.
  void get_input_port(int) = delete;

  /// Returns the sole output port.
  const drake::systems::OutputPort<double>& get_output_port() const {
    return drake::systems::LeafSystem<double>::get_output_port(0);
  }

  // Don't use the indexed get_output_port when calling this system directly.
  void get_output_port(int) = delete;

 protected:
  void DoCalcVectorOutput(const drake::systems::Context<double>& context,
                          systems::OutputVector<double>* output) const;

 private:
  bool is_abstract() const { return false; }

  int num_positions_;
  int num_velocities_;
  int num_inputs_;
  Eigen::MatrixXd pos_variance_;
  Eigen::MatrixXd vel_variance_;
//  int pos_start_;
//  int pos_length_;
//  int vel_start_;
//  int vel_length_;
//  int input_start_;
//  int input_length_;
};

}  // namespace systems
}  // namespace dairlib
