#pragma once

#include <random>

#include <Eigen/Dense>

#include "examples/sampling_c3/parameter_headers/object_state_error_params.h"

#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace systems {

/// A Drake system in the simulator diagram that corrupts a clean object state
/// with the kinds of errors the hardware pose estimator produces, so that a
/// simulated controller sees a hardware-like object pose estimate.
///
/// The input and output are both the plain (untimestamped) BasicVector
/// [q, v] of a single object's model instance, i.e. exactly what
/// plant.get_state_output_port(object_index) produces and what
/// ObjectStateSender consumes.  The first four positions are taken to be a
/// quaternion (w, x, y, z) and the next three the object's position; the
/// velocities are passed through untouched.
///
/// The error is the sum of up to three terms, each independently enabled by
/// ObjectStateErrorParams:
///   - a constant bias, drawn once at construction and held for the run,
///   - white noise, redrawn at every update,
///   - an Ornstein-Uhlenbeck drift, which is what reproduces the rare,
///     sustained, multi-second excursions that dominate the hardware error.
///
/// The error state is only recomputed on a periodic discrete update of period
/// `update_period` (set this to the object state publish period), so the error
/// is held constant between publishes rather than re-rolling on every
/// evaluation of the output port.
class ObjectStateErrorInjector : public drake::systems::LeafSystem<double> {
 public:
  /// @param num_positions Number of positions of the object model instance.
  ///                      Must be at least 7 (quaternion plus position).
  /// @param num_velocities Number of velocities of the object model instance.
  /// @param params The error terms to apply.
  /// @param update_period Period, in seconds, at which the error is redrawn.
  ObjectStateErrorInjector(int num_positions, int num_velocities,
                           const ObjectStateErrorParams& params,
                           double update_period);

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_input_port_);
  }

  const drake::systems::OutputPort<double>& get_output_port_noisy_state()
      const {
    return this->get_output_port(noisy_state_output_port_);
  }

 private:
  drake::systems::EventStatus UpdateErrors(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  void CalcNoisyState(const drake::systems::Context<double>& context,
                      drake::systems::BasicVector<double>* output) const;

  /// Draws a vector of independent zero-mean Gaussians with the given per-axis
  /// standard deviations.
  Eigen::Vector3d DrawGaussian(const Eigen::VectorXd& std_devs) const;

  const int num_positions_;
  const int num_velocities_;
  const ObjectStateErrorParams params_;
  const double update_period_;
  /// Per-step retention factor of the Ornstein-Uhlenbeck drift,
  /// exp(-update_period / drift_time_constant).
  const double drift_decay_;

  drake::systems::InputPortIndex state_input_port_;
  drake::systems::OutputPortIndex noisy_state_output_port_;
  /// [drift_position(3), drift_rpy(3), white_position(3), white_rpy(3)].
  drake::systems::DiscreteStateIndex error_state_index_;

  Eigen::Vector3d bias_position_;
  Eigen::Vector3d bias_rpy_;

  /// Mutable because the discrete update handler is const.  This is only
  /// touched from that single handler, which the simulator calls serially, so
  /// there is no race here.
  mutable std::mt19937 generator_;
};

}  // namespace systems
}  // namespace dairlib
