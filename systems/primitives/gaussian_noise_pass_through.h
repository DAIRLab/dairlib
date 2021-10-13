#pragma once

#include <memory>
#include <drake/common/eigen_types.h>

#include "systems/framework/output_vector.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace systems {

/// this class is copied from drake/systems/primitives/Passthrough
/// with the modification that it adds random noise to the values it is
/// passing through
class GaussianNoisePassThrough final : public
    drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(GaussianNoisePassThrough)

  /// Constructs a pass through system (`y = u.segment(start,length)`).
  /// @param num_positions the number of generalized positions
  /// @param num_velocities the number of generalized velocities
  /// @param num_inputs the number of inputs
  /// @param pos_variance covariance matrix for the generalized position
  /// @param vel_variance covariance matrix for the generalized velocities
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
  Eigen::MatrixXd pos_variance_;
  Eigen::MatrixXd vel_variance_;
};

}  // namespace systems
}  // namespace dairlib
