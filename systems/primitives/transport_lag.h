#pragma once

#include <Eigen/Dense>

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/diagram.h"

namespace dairlib {
namespace systems {

/// A pure transport delay followed by a first-order lag, applied elementwise to
/// a vector-valued signal.  In the Laplace domain,
///
///   y(s) = exp(-delay * s) / (time_constant * s + 1) * u(s).
///
/// This is the usual way to describe an actuator that does not respond for a
/// while after it is commanded (queueing, transport, communication) and then
/// eases into the command rather than snapping to it (acceleration limits,
/// compliance).  Both effects are measured together as the "dead time" and
/// "time constant" of a step response, which is how the arguments here are
/// meant to be supplied.
///
/// @system
/// name: TransportLag
/// input_ports:
/// - u
/// output_ports:
/// - y
/// @endsystem
///
/// Both stages are discrete, so adding this to an otherwise discrete diagram
/// does not introduce continuous state.  They are also both linear and
/// time-invariant, and therefore commute with differentiation: if the input
/// vector stacks a signal and its own derivative, the output does too.
///
/// Drake's own drake::systems::DiscreteTimeDelay and
/// drake::systems::FirstOrderLowPassFilter cover one stage each, but the latter
/// has continuous state and neither can start from a nonzero value.  This class
/// exists to compose the two and to provide SetInitialValue().
class TransportLag final : public drake::systems::Diagram<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TransportLag)

  /// @param size the number of elements in the signal.
  /// @param update_period the discrete update period, in seconds.
  /// @param delay the dead time, in seconds.  Must be a non-negative integer
  ///        multiple of `update_period`; zero omits the delay stage.
  /// @param time_constant the first-order time constant, in seconds.  Must be
  ///        non-negative; zero omits the lag stage.
  ///
  /// With `delay` and `time_constant` both zero this is a pass-through.
  ///
  /// @throws std::exception if `size` or `update_period` is not positive, if
  ///         either of `delay`/`time_constant` is negative, or if `delay` is
  ///         not an integer multiple of `update_period`.
  TransportLag(int size, double update_period, double delay,
               double time_constant);

  ~TransportLag() final = default;

  const drake::systems::InputPort<double>& get_input_port() const {
    return drake::systems::Diagram<double>::get_input_port(0);
  }

  const drake::systems::OutputPort<double>& get_output_port() const {
    return drake::systems::Diagram<double>::get_output_port(0);
  }

  /// Sets the state of both stages so that the system outputs `value` until the
  /// input has had time to propagate through it.  Without this the delay's
  /// buffer starts at zero, which would drag the output to the origin for the
  /// first `delay` seconds.
  ///
  /// @param context this system's context, e.g. from
  ///        `diagram.GetMutableSubsystemContext(lag, &root_context)`.
  /// @throws std::exception if `value.size() != size()`.
  void SetInitialValue(drake::systems::Context<double>* context,
                       const Eigen::Ref<const Eigen::VectorXd>& value) const;

  int size() const { return size_; }
  double delay() const { return delay_; }
  double time_constant() const { return time_constant_; }

 private:
  const int size_;
  const double delay_;
  const double time_constant_;

  // Null when the corresponding stage is omitted.
  const drake::systems::System<double>* delay_system_{nullptr};
  const drake::systems::System<double>* lag_system_{nullptr};
};

}  // namespace systems
}  // namespace dairlib
