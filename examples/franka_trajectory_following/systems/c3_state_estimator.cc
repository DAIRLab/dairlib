#include "c3_state_estimator.h"

#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_ball_position.hpp"

using drake::systems::Context;
using drake::systems::DiscreteUpdateEvent;
using drake::systems::State;
using drake::systems::EventStatus;
using drake::systems::BasicVector;
using drake::systems::Context;

namespace dairlib {
namespace systems {

C3StateEstimator::C3StateEstimator(const std::vector<double>& p_FIR_values,
    const std::vector<double>& v_FIR_values) :
    p_FIR_values_(p_FIR_values), v_FIR_values_(v_FIR_values), 
    p_filter_length_(p_FIR_values.size()), v_filter_length_(v_FIR_values.size()) {


  param_ = drake::yaml::LoadYamlFile<C3Parameters>(
      "examples/franka_trajectory_following/parameters.yaml");

  /// declare discrete states
  Vector3d initial_position;
  initial_position(0) = param_.x_c + param_.traj_radius * sin(param_.phase * 3.14159265 / 180);
  initial_position(1) = param_.y_c + param_.traj_radius * cos(param_.phase * 3.14159265 / 180);
  initial_position(2) = param_.ball_radius + param_.table_offset;

  p_idx_ = this->DeclareDiscreteState(initial_position);
  v_idx_ = this->DeclareDiscreteState(3);         // automatically initialized to all 0s
  w_idx_ = this->DeclareDiscreteState(3);         // automatically initialized to all 0s

  p_history_idx_ = this->DeclareAbstractState(
    drake::Value<std::deque<Vector3d>>(
      (size_t) p_filter_length_,
      initial_position));
      
  v_history_idx_ = this->DeclareAbstractState(
    drake::Value<std::deque<Vector3d>>(
      (size_t) v_filter_length_,
      VectorXd::Zero(3)));

  // if visualization on vison side is false, use 1/98.0 for the default value
  std::deque<double> default_dt_deque((size_t) dt_filter_length_, 1/60.0);
  dt_history_idx_ = this->DeclareAbstractState(
    drake::Value<std::deque<double>>(default_dt_deque));
  
  prev_time_idx_ = this->DeclareAbstractState(
    drake::Value<double>(0));
  
  this->DeclarePerStepUnrestrictedUpdateEvent(
    &C3StateEstimator::UpdateHistory);

  /// declare I/O ports
  franka_input_port_ = this->DeclareAbstractInputPort("franka_port",
                                 drake::Value<dairlib::lcmt_robot_output>{}).get_index();
  ball_input_port_ = this->DeclareAbstractInputPort("ball_port",
                                 drake::Value<dairlib::lcmt_ball_position>{}).get_index();
  this->DeclareVectorOutputPort(
      "x, u, t",
      OutputVector<double>(14, 13, 7),
      &C3StateEstimator::EstimateState);
}

EventStatus C3StateEstimator::UpdateHistory(const Context<double>& context,
    State<double>* state) const {
  
  /// Extract mutable abstract states
  // newest entries go to back of deque, oldest entries popped off the front
  auto& p_history = state->get_mutable_abstract_state<std::deque<Vector3d>>(
    p_history_idx_);
  auto& v_history = state->get_mutable_abstract_state<std::deque<Vector3d>>(
    v_history_idx_);
  auto& dt_history = state->get_mutable_abstract_state<std::deque<double>>(
    dt_history_idx_);
  auto& prev_time = state->get_mutable_abstract_state<double>(
    prev_time_idx_);

  /// check if received new input from camera
  const drake::AbstractValue* input = this->EvalAbstractInput(context, ball_input_port_);
  DRAKE_ASSERT(input != nullptr);
  const auto& ball_position = input->get_value<dairlib::lcmt_ball_position>();
  double timestamp = ball_position.utime * 1.0e-6;
  
  if (timestamp != prev_time){
    /// estimate dt
    dt_history.push_back(timestamp-prev_time);
    dt_history.pop_front();

    double dt = 0;
    for (int i = 0; i < dt_filter_length_; i++){
      dt += dt_history[i];
    }
    dt /= dt_filter_length_;

    /// estimate position

    Vector3d prev_position = state->get_discrete_state(p_idx_).value();
    p_history.push_back(Vector3d(
      ball_position.xyz[0], ball_position.xyz[1], ball_position.xyz[2]));
    p_history.pop_front();

    Vector3d estimated_position = VectorXd::Zero(3);
    for (int i = 0; i < p_filter_length_; i++){
      estimated_position += p_FIR_values_[i] * p_history[i];
    }
    state->get_mutable_discrete_state(p_idx_).get_mutable_value() << estimated_position;


    /// estimate velocity
    v_history.push_back(
      (estimated_position - prev_position) / dt);
    v_history.pop_front();

    Vector3d estimated_velocity = VectorXd::Zero(3);
    for (int i = 0; i < v_filter_length_; i++){
      estimated_velocity += v_FIR_values_[i] * v_history[i];
    }
    state->get_mutable_discrete_state(v_idx_).get_mutable_value() << estimated_velocity;

    ///  estimate angular velocity
    double ball_radius = param_.ball_radius;
    Vector3d r_ball(0, 0, ball_radius);
    state->get_mutable_discrete_state(w_idx_).get_mutable_value() << 
      r_ball.cross(estimated_velocity) / (ball_radius * ball_radius);   

    /// update prev_time
    prev_time = timestamp;
  }

  return EventStatus::Succeeded();
}

void C3StateEstimator::EstimateState(const drake::systems::Context<double>& context,
                  OutputVector<double>* output) const {

  /// parse inputs
  const drake::AbstractValue* input = this->EvalAbstractInput(context, franka_input_port_);
  DRAKE_ASSERT(input != nullptr);
  const auto& franka_output = input->get_value<dairlib::lcmt_robot_output>();

  /// read in estimates froms states
  Vector3d ball_position = context.get_discrete_state(p_idx_).value();
  Vector3d ball_velocity = context.get_discrete_state(v_idx_).value();
  Vector3d angular_velocity = context.get_discrete_state(w_idx_).value();

  /// generate output
  // NOTE: vector sizes are hard coded for C3 experiments  
  VectorXd positions = VectorXd::Zero(num_franka_positions_ + num_ball_positions_);
  for (int i = 0; i < num_franka_positions_; i++){
    positions(i) = franka_output.position[i];    
  }
  positions.tail(num_ball_positions_) << 1, 0, 0, 0, ball_position;

  VectorXd velocities = VectorXd::Zero(
    num_franka_velocities_ + num_ball_velocities_);
  for (int i = 0; i < num_franka_velocities_; i++){
    velocities(i) = franka_output.velocity[i];
  }
  velocities.tail(num_ball_velocities_) << angular_velocity, ball_velocity;

  VectorXd efforts = VectorXd::Zero(
      num_franka_efforts_ + num_ball_efforts_);
  for (int i = 0; i < num_franka_efforts_; i++){
    efforts(i) = franka_output.effort[i];
  }

  output->SetPositions(positions);
  output->SetVelocities(velocities);
  output->SetEfforts(efforts);
  output->set_timestamp(franka_output.utime * 1.0e-6);
}

}  // namespace systems
}  // namespace dairlib