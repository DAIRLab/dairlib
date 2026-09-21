#include "examples/sampling_c3/object_state_error_injector.h"

#include <cmath>
#include <stdexcept>

#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"

namespace dairlib {

using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;
using Eigen::Vector3d;
using Eigen::VectorXd;

namespace systems {

namespace {
// Indices into the error discrete state vector.
constexpr int kDriftPositionStart = 0;
constexpr int kDriftRpyStart = 3;
constexpr int kWhitePositionStart = 6;
constexpr int kWhiteRpyStart = 9;
constexpr int kNumErrorStates = 12;
}  // namespace

ObjectStateErrorInjector::ObjectStateErrorInjector(
    int num_positions, int num_velocities,
    const ObjectStateErrorParams& params, double update_period)
    : num_positions_(num_positions),
      num_velocities_(num_velocities),
      params_(params),
      update_period_(update_period),
      drift_decay_(params.enable_drift
                       ? std::exp(-update_period / params.drift_time_constant)
                       : 0.0),
      bias_position_(Vector3d::Zero()),
      bias_rpy_(Vector3d::Zero()) {
  this->set_name("ObjectStateErrorInjector");

  if (num_positions < 7) {
    throw std::runtime_error(
        "ObjectStateErrorInjector expects a floating object with at least 7 "
        "positions (quaternion plus position), but got " +
        std::to_string(num_positions) + ".");
  }
  if (update_period <= 0.0) {
    throw std::runtime_error(
        "ObjectStateErrorInjector requires a positive update period.");
  }

  generator_.seed(params_.seed.has_value()
                      ? static_cast<std::mt19937::result_type>(*params_.seed)
                      : std::random_device{}());

  // The bias is constant for the whole run, so draw it once here rather than
  // keeping it in state.
  if (params_.enable_bias) {
    bias_position_ = DrawGaussian(params_.bias_position_std);
    bias_rpy_ = DrawGaussian(params_.bias_orientation_std_deg) * M_PI / 180.0;
  }

  state_input_port_ =
      this->DeclareVectorInputPort(
              "x", BasicVector<double>(num_positions_ + num_velocities_))
          .get_index();

  noisy_state_output_port_ =
      this->DeclareVectorOutputPort(
              "x_noisy", BasicVector<double>(num_positions_ + num_velocities_),
              &ObjectStateErrorInjector::CalcNoisyState)
          .get_index();

  error_state_index_ = this->DeclareDiscreteState(VectorXd::Zero(
      kNumErrorStates));

  this->DeclarePeriodicDiscreteUpdateEvent(
      update_period_, 0.0, &ObjectStateErrorInjector::UpdateErrors);
}

Vector3d ObjectStateErrorInjector::DrawGaussian(
    const VectorXd& std_devs) const {
  std::normal_distribution<double> standard_normal(0.0, 1.0);
  Vector3d draw;
  for (int i = 0; i < 3; i++) {
    draw(i) = std_devs(i) * standard_normal(generator_);
  }
  return draw;
}

EventStatus ObjectStateErrorInjector::UpdateErrors(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  VectorXd errors = context.get_discrete_state(error_state_index_).value();

  if (params_.enable_drift) {
    // Exact discretization of an Ornstein-Uhlenbeck process, so that the
    // configured standard deviations are the stationary standard deviations of
    // the drift rather than per-step increments.
    const double innovation_scale =
        std::sqrt(1.0 - drift_decay_ * drift_decay_);
    errors.segment<3>(kDriftPositionStart) =
        drift_decay_ * errors.segment<3>(kDriftPositionStart) +
        innovation_scale * DrawGaussian(params_.drift_position_std);
    errors.segment<3>(kDriftRpyStart) =
        drift_decay_ * errors.segment<3>(kDriftRpyStart) +
        innovation_scale *
            DrawGaussian(params_.drift_orientation_std_deg) * M_PI / 180.0;
  }

  if (params_.enable_white_noise) {
    errors.segment<3>(kWhitePositionStart) =
        DrawGaussian(params_.white_noise_position_std);
    errors.segment<3>(kWhiteRpyStart) =
        DrawGaussian(params_.white_noise_orientation_std_deg) * M_PI / 180.0;
  }

  discrete_state->get_mutable_vector(error_state_index_).SetFromVector(errors);

  return EventStatus::Succeeded();
}

void ObjectStateErrorInjector::CalcNoisyState(
    const Context<double>& context, BasicVector<double>* output) const {
  const VectorXd clean_state =
      this->EvalVectorInput(context, state_input_port_)->value();
  const VectorXd errors =
      context.get_discrete_state(error_state_index_).value();

  const Vector3d position_error = bias_position_ +
                                  errors.segment<3>(kDriftPositionStart) +
                                  errors.segment<3>(kWhitePositionStart);
  const Vector3d rpy_error = bias_rpy_ + errors.segment<3>(kDriftRpyStart) +
                             errors.segment<3>(kWhiteRpyStart);

  VectorXd noisy_state = clean_state;

  // Perturb the orientation in the object's own frame, which is the shape a
  // pose estimator's error takes.  Going through a rotation matrix keeps the
  // result a unit quaternion without any explicit renormalization.
  const Eigen::Quaterniond clean_quaternion(clean_state(0), clean_state(1),
                                            clean_state(2), clean_state(3));
  const drake::math::RotationMatrix<double> noisy_rotation =
      drake::math::RotationMatrix<double>(clean_quaternion.normalized()) *
      drake::math::RotationMatrix<double>(
          drake::math::RollPitchYaw<double>(rpy_error));
  const Eigen::Quaterniond noisy_quaternion = noisy_rotation.ToQuaternion();

  noisy_state(0) = noisy_quaternion.w();
  noisy_state(1) = noisy_quaternion.x();
  noisy_state(2) = noisy_quaternion.y();
  noisy_state(3) = noisy_quaternion.z();
  noisy_state.segment<3>(4) += position_error;

  output->SetFromVector(noisy_state);
}

}  // namespace systems
}  // namespace dairlib
