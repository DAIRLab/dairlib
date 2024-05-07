#include "state_estimator.h"

using drake::math::RotationMatrix;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteUpdateEvent;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;
using drake::systems::State;
using Eigen::Quaternion;

namespace dairlib {
namespace systems {

StateEstimator::StateEstimator(const std::vector<double>& p_FIR_values,
                               const std::vector<double>& v_FIR_values)
    : p_FIR_values_(p_FIR_values),
      v_FIR_values_(v_FIR_values),
      p_filter_length_(p_FIR_values.size()),
      v_filter_length_(v_FIR_values.size()) {
  state_estimate_param_ = drake::yaml::LoadYamlFile<StateEstimatorParams>(
      "examples/franka_ball_rolling/parameters/state_estimator_params.yaml");

  /// declare discrete states
  Vector3d initial_position;
  initial_position(0) = state_estimate_param_.x_c +
                        state_estimate_param_.traj_radius *
                            sin(state_estimate_param_.phase * M_PI / 180);
  initial_position(1) = state_estimate_param_.y_c +
                        state_estimate_param_.traj_radius *
                            cos(state_estimate_param_.phase * M_PI / 180);
  initial_position(2) = state_estimate_param_.ball_radius -
                        state_estimate_param_.ground_offset_frame(2);
  VectorXd initial_orientation = state_estimate_param_.q_init_ball;

  p_idx_ = this->DeclareDiscreteState(initial_position);
  orientation_idx_ = this->DeclareDiscreteState(initial_orientation);
  v_idx_ =
      this->DeclareDiscreteState(3);  // automatically initialized to all 0s
  w_idx_ =
      this->DeclareDiscreteState(3);  // automatically initialized to all 0s

  p_history_idx_ =
      this->DeclareAbstractState(drake::Value<std::deque<Vector3d>>(
          (size_t)p_filter_length_, initial_position));

  v_history_idx_ =
      this->DeclareAbstractState(drake::Value<std::deque<Vector3d>>(
          (size_t)v_filter_length_, VectorXd::Zero(3)));

  prev_time_idx_ = this->DeclareAbstractState(drake::Value<double>(0));

  prev_id_idx_ = this->DeclareAbstractState(drake::Value<int>(-1));

  //  this->DeclarePerStepUnrestrictedUpdateEvent(
  //    &C3StateEstimator::UpdateHistory);

  this->DeclareForcedUnrestrictedUpdateEvent(&StateEstimator::UpdateHistory);

  /// declare I/O ports
  //  franka_input_port_ = this->DeclareAbstractInputPort("franka_port",
  //                                 drake::Value<dairlib::lcmt_robot_output>{}).get_index();
  franka_input_port_ =
      this->DeclareVectorInputPort(
              "franka_port",
              OutputVector<double>(num_franka_positions_,
                                   num_franka_velocities_, num_franka_efforts_))
          .get_index();
  ball_input_port_ =
      this->DeclareAbstractInputPort(
              "ball_port", drake::Value<dairlib::lcmt_ball_position>{})
          .get_index();
  //  this->DeclareVectorOutputPort(
  //      "x",
  //      BasicVector<double>(num_franka_positions_ + num_ball_positions_ +
  //                          num_franka_velocities_ + num_ball_velocities_),
  //      &C3StateEstimator::EstimateState);
  franka_state_output_port_ =
      this->DeclareVectorOutputPort("x_franka",
                                    BasicVector<double>(num_franka_positions_ +
                                                        num_franka_velocities_),
                                    &StateEstimator::EstimateFrankaState)
          .get_index();
  this->DeclareVectorOutputPort(
      "u_franka", BasicVector<double>(num_franka_efforts_ + num_ball_efforts_),
      &StateEstimator::OutputEfforts);
  ball_state_output_port_ =
      this->DeclareVectorOutputPort(
              "x_object",
              BasicVector<double>(num_ball_positions_ + num_ball_velocities_),
              &StateEstimator::EstimateObjectState)
          .get_index();
}

EventStatus StateEstimator::UpdateHistory(const Context<double>& context,
                                          State<double>* state) const {
  /// Extract mutable abstract states
  // newest entries go to back of deque, oldest entries popped off the front
  auto& p_history =
      state->get_mutable_abstract_state<std::deque<Vector3d>>(p_history_idx_);
  auto& v_history =
      state->get_mutable_abstract_state<std::deque<Vector3d>>(v_history_idx_);
  auto& prev_time = state->get_mutable_abstract_state<double>(prev_time_idx_);
  auto& prev_id = state->get_mutable_abstract_state<int>(prev_id_idx_);

  /// check if received new input from camera
  const drake::AbstractValue* input =
      this->EvalAbstractInput(context, ball_input_port_);
  DRAKE_ASSERT(input != nullptr);
  const auto& ball_position = input->get_value<dairlib::lcmt_ball_position>();
  int id = ball_position.id;
  double timestamp = ball_position.utime * 1.0e-6;

  if (id != prev_id) {
    double dt = timestamp - prev_time;
    if (!std::isnan(ball_position.xyz[0]) && dt > 1e-9) {
      /// Becuase we mix ball and franka in one system and this system is driven
      /// by Franka state so the update would be at much higher rate, but ball
      /// is low rate, need to judge whether this ball information is a new one.

      /// estimate position and apply FIR filter
      Vector3d prev_position = state->get_discrete_state(p_idx_).value();
      p_history.push_back(Vector3d(ball_position.xyz[0], ball_position.xyz[1],
                                   ball_position.xyz[2]));
      p_history.pop_front();

      Vector3d estimated_position = VectorXd::Zero(3);
      for (int i = 0; i < p_filter_length_; i++) {
        estimated_position += p_FIR_values_[i] * p_history[i];
      }
      state->get_mutable_discrete_state(p_idx_).get_mutable_value()
          << estimated_position;

      /// estimate velocity through finite difference and apply FIR filter
      v_history.push_back((estimated_position - prev_position) / dt);
      v_history.pop_front();

      Vector3d estimated_velocity = VectorXd::Zero(3);
      for (int i = 0; i < v_filter_length_; i++) {
        estimated_velocity += v_FIR_values_[i] * v_history[i];
      }
      state->get_mutable_discrete_state(v_idx_).get_mutable_value()
          << estimated_velocity;

      ///  estimate angular velocity
      double ball_radius = state_estimate_param_.ball_radius;
      Vector3d r_ball(0, 0, ball_radius);
      Vector3d w =
          r_ball.cross(estimated_velocity) / (ball_radius * ball_radius);
      state->get_mutable_discrete_state(w_idx_).get_mutable_value() << w;

      /// numerically integrate angular velocity to get orientation
      VectorXd orientation =
          state->get_discrete_state(orientation_idx_).value();
      Quaternion<double> quaternion(orientation(0), orientation(1),
                                    orientation(2), orientation(3));
      RotationMatrix<double> curr_orientation(quaternion);
      RotationMatrix<double> rotation =
          this->RodriguesFormula(w / w.norm(), w.norm() * dt);
      VectorXd new_orientation =
          (rotation * curr_orientation).ToQuaternionAsVector4();

      // comment the following two lines to update the new orientation, not
      // important for this work
      // state->get_mutable_discrete_state(orientation_idx_).get_mutable_value()
      //   << new_orientation;

      state->get_mutable_discrete_state(orientation_idx_).get_mutable_value()
          << 1,
          0, 0, 0;
    }
    /// update prev_time
    prev_time = timestamp;
    prev_id = id;
  }

  return EventStatus::Succeeded();
}

void StateEstimator::EstimateState(
    const drake::systems::Context<double>& context,
    BasicVector<double>* output) const {
  /// parse inputs
  const OutputVector<double>* franka_output =
      (OutputVector<double>*)this->EvalVectorInput(context, franka_input_port_);
  DRAKE_ASSERT(franka_output != nullptr);

  /// read in estimates froms states
  Vector3d ball_position = context.get_discrete_state(p_idx_).value();
  VectorXd ball_orientation =
      context.get_discrete_state(orientation_idx_).value();
  Vector3d ball_velocity = context.get_discrete_state(v_idx_).value();
  Vector3d angular_velocity = context.get_discrete_state(w_idx_).value();

  VectorXd q_franka = franka_output->GetPositions();
  VectorXd v_franka = franka_output->GetVelocities();

  /// generate output
  // NOTE: vector sizes are hard coded for C3 experiments
  VectorXd positions =
      VectorXd::Zero(num_franka_positions_ + num_ball_positions_);
  for (int i = 0; i < num_franka_positions_; i++) {
    positions(i) = q_franka(i);
  }
  positions.tail(num_ball_positions_) << ball_orientation, ball_position;

  VectorXd velocities =
      VectorXd::Zero(num_franka_velocities_ + num_ball_velocities_);
  for (int i = 0; i < num_franka_velocities_; i++) {
    velocities(i) = v_franka(i);
  }
  velocities.tail(num_ball_velocities_) << angular_velocity, ball_velocity;

  VectorXd value =
      VectorXd::Zero(num_franka_positions_ + num_franka_velocities_ +
                     num_ball_positions_ + num_ball_velocities_);
  value << positions, velocities;
  output->SetFromVector(value);
}

void StateEstimator::EstimateFrankaState(
    const drake::systems::Context<double>& context,
    BasicVector<double>* output) const {
  /// parse inputs
  const OutputVector<double>* franka_output =
      (OutputVector<double>*)this->EvalVectorInput(context, franka_input_port_);
  DRAKE_ASSERT(franka_output != nullptr);

  VectorXd q_franka = franka_output->GetPositions();
  VectorXd v_franka = franka_output->GetVelocities();

  /// generate output
  VectorXd positions = VectorXd::Zero(num_franka_positions_);
  for (int i = 0; i < num_franka_positions_; i++) {
    positions(i) = q_franka(i);
  }

  VectorXd velocities = VectorXd::Zero(num_franka_velocities_);
  for (int i = 0; i < num_franka_velocities_; i++) {
    velocities(i) = v_franka(i);
  }
  VectorXd franka_state =
      VectorXd::Zero(num_franka_positions_ + num_franka_velocities_);
  franka_state << positions, velocities;
  output->SetFromVector(franka_state);
}

void StateEstimator::EstimateObjectState(
    const drake::systems::Context<double>& context,
    BasicVector<double>* output) const {
  /// read in estimates froms states
  Vector3d ball_position = context.get_discrete_state(p_idx_).value();
  VectorXd ball_orientation =
      context.get_discrete_state(orientation_idx_).value();
  Vector3d ball_velocity = context.get_discrete_state(v_idx_).value();
  Vector3d angular_velocity = context.get_discrete_state(w_idx_).value();

  /// generate output
  // NOTE: vector sizes are hard coded for C3 experiments
  VectorXd ball_state =
      VectorXd::Zero(num_ball_positions_ + num_ball_velocities_);
  ball_state << ball_orientation, ball_position, angular_velocity,
      ball_velocity;

  output->SetFromVector(ball_state);
}

void StateEstimator::OutputEfforts(
    const drake::systems::Context<double>& context,
    BasicVector<double>* output) const {
  /// parse inputs
  const OutputVector<double>* franka_output =
      (OutputVector<double>*)this->EvalVectorInput(context, franka_input_port_);
  DRAKE_ASSERT(franka_output != nullptr);

  VectorXd u_franka = franka_output->GetEfforts();

  VectorXd efforts = VectorXd::Zero(num_franka_efforts_ + num_ball_efforts_);
  for (int i = 0; i < num_franka_efforts_; i++) {
    efforts(i) = u_franka(i);
  }

  output->SetFromVector(efforts);
}

RotationMatrix<double> StateEstimator::RodriguesFormula(const Vector3d& axis,
                                                        double theta) const {
  double w1 = axis(0);
  double w2 = axis(1);
  double w3 = axis(2);

  Eigen::Matrix3d Sw;
  Sw << 0, -w3, w2, w3, 0, -w1, -w1, w1, 0;
  Eigen::Matrix3d R =
      MatrixXd::Identity(3, 3) + sin(theta) * Sw + (1 - cos(theta)) * Sw * Sw;
  return RotationMatrix<double>(R);
}

/* ------------------------------------------------------------------------------
 */
/// Method implementation of TrueBallToEstimatedBall class
TrueBallToEstimatedBall::TrueBallToEstimatedBall(double stddev, double period)
    : stddev_(stddev), period_(period) {
  state_estimate_param_ = drake::yaml::LoadYamlFile<StateEstimatorParams>(
      "examples/franka_ball_rolling/parameters/state_estimator_params.yaml");

  /// declare discrete states
  Vector3d initial_position;
  initial_position(0) = state_estimate_param_.x_c +
                        state_estimate_param_.traj_radius *
                            sin(state_estimate_param_.phase * M_PI / 180);
  initial_position(1) = state_estimate_param_.y_c +
                        state_estimate_param_.traj_radius *
                            cos(state_estimate_param_.phase * M_PI / 180);
  initial_position(2) = state_estimate_param_.ball_radius -
                        state_estimate_param_.ground_offset_frame(2);

  p_idx_ = this->DeclareDiscreteState(initial_position);
  id_idx_ = this->DeclareDiscreteState(1);  // automatically initialized to 0;
  utime_idx_ = this->DeclareDiscreteState(1);

  true_ball_input_port_ =
      this->DeclareVectorInputPort(
              "x_object",
              StateVector<double>(num_ball_positions_, num_ball_velocities))
          .get_index();
  this->DeclareAbstractOutputPort("lcmt_ball_position",
                                  &TrueBallToEstimatedBall::ConvertOutput);

  //  this->DeclareAbstractInputPort("lcmt_robot_output",
  //                                 drake::Value<dairlib::lcmt_robot_output>{});
  //  this->DeclarePeriodicDiscreteUpdateEvent(period_, 0,
  //                                                     &TrueBallToEstimatedBall::UpdateBallPosition);
  this->DeclareForcedDiscreteUpdateEvent(
      &TrueBallToEstimatedBall::UpdateBallPosition);
}

EventStatus TrueBallToEstimatedBall::UpdateBallPosition(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  const StateVector<double>* true_ball_output =
      (StateVector<double>*)this->EvalVectorInput(context,
                                                  true_ball_input_port_);
  DRAKE_ASSERT(true_ball_output != nullptr);

  VectorXd position = true_ball_output->GetPositions().tail(3);

  if (stddev_ > 1e-12) {
    std::random_device rd{};
    std::mt19937 gen{rd()};
    std::normal_distribution<> d{0, stddev_};

    /// changes
    double x_noise = d(gen);
    double y_noise = d(gen);

    /// apply 3 sigma rule to clamp the noise, or use the maximum observed
    /// noise value to clamp (here is real experiment it is about 0.01 m)
    //    double noise_threshold = 3 * stddev_;
    double noise_threshold = 0.01;
    if (x_noise > noise_threshold) {
      x_noise = noise_threshold;
    } else if (x_noise < -noise_threshold) {
      x_noise = -noise_threshold;
    }
    if (y_noise > noise_threshold) {
      y_noise = noise_threshold;
    } else if (y_noise < -noise_threshold) {
      y_noise = -noise_threshold;
    }

    position(0) += x_noise;
    position(1) += y_noise;
  }

  discrete_state->get_mutable_vector(p_idx_).get_mutable_value() << position;
  discrete_state->get_mutable_vector(utime_idx_).get_mutable_value()
      << int(true_ball_output->get_timestamp() * 1.0e6);
  double id = discrete_state->get_vector(id_idx_).get_value()(0);
  discrete_state->get_mutable_vector(id_idx_).get_mutable_value() << id + 1;

  return EventStatus::Succeeded();
}

void TrueBallToEstimatedBall::ConvertOutput(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_ball_position* output) const {
  Vector3d ball_position = context.get_discrete_state(p_idx_).value();
  double id = context.get_discrete_state(id_idx_).value()(0);
  double utime = context.get_discrete_state(utime_idx_).value()(0);

  for (int i = 0; i < 3; i++) {
    output->xyz[i] = ball_position(i);
    output->cam_statuses[i] = "N/A";
  }
  output->num_cameras_used = -1;
  output->id = (int)id;
  output->utime = utime;
}

}  // namespace systems
}  // namespace dairlib