#include "fingertips_delta_position_receiver.h"

#include <iostream>
#include <utility>

namespace dairlib::systems {
using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::Trajectory;

FingertipDeltaPositionReceiver::FingertipDeltaPositionReceiver(
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context,
    const Eigen::Vector3d& min_fingertips_delta_position,
    const Eigen::Vector3d& max_fingertips_delta_position,
    const std::string& fingertip_0_name, const std::string& fingertip_120_name,
    const std::string& fingertip_240_name,
    const unsigned int& delta_pos_update_frequency)
    : plant_(plant),
      context_(context),
      min_fingertips_delta_position_(min_fingertips_delta_position),
      max_fingertips_delta_position_(max_fingertips_delta_position),
      fingertip_0_name_(fingertip_0_name),
      fingertip_120_name_(fingertip_120_name),
      fingertip_240_name_(fingertip_240_name),
      delta_pos_update_frequency_(delta_pos_update_frequency) {
  this->set_name("fingertips_delta_position_receiver");
  state_port_ =
      this->DeclareVectorInputPort("x_trifinger",
                                   OutputVector<double>(plant.num_positions(),
                                                        plant.num_velocities(),
                                                        plant.num_actuators()))
          .get_index();

  fingertips_delta_position_port_ =
      this->DeclareAbstractInputPort(
              "fingertips_delta_position",
              drake::Value<dairlib::lcmt_fingertips_delta_position>{})
          .get_index();

  PiecewisePolynomial<double> empty_pp_traj(Eigen::MatrixXd::Zero(9, 2));
  Trajectory<double>& traj_inst = empty_pp_traj;
  fingertips_target_traj_port_ =
      this->DeclareAbstractOutputPort(
              "fingertips_target_traj", traj_inst,
              &FingertipDeltaPositionReceiver::CopyToOutputFingertipsTargetTraj)
          .get_index();

  fingertips_target_port_ =
      this->DeclareVectorOutputPort(
              "fingertips_target", TimestampedVector<double>(9),
              &FingertipDeltaPositionReceiver::CopyToOutputFingertipsTarget)
          .get_index();

  cur_fingertips_pos_port_ =
      this->DeclareVectorOutputPort("cur_fingertips_pos",
                                    TimestampedVector<double>(9),
                                    &FingertipDeltaPositionReceiver::
                                        CopyToOutputCurrentFingertipPositions)
          .get_index();

  // Declare update event.
  DeclareForcedDiscreteUpdateEvent(
      &FingertipDeltaPositionReceiver::DiscreteVariableUpdate);

  // Discrete state which stores the desired fingertips position.
  fingertips_target_idx_ = DeclareDiscreteState(Eigen::VectorXd::Zero(9));
  prev_target_timestamp_idx_ =
      DeclareDiscreteState(Eigen::VectorXd::Ones(1) * (-1));
  cur_fingertips_pos_idx_ = DeclareDiscreteState(Eigen::VectorXd::Zero(9));
}

drake::systems::EventStatus
FingertipDeltaPositionReceiver::DiscreteVariableUpdate(
    const drake::systems::Context<double>& context,
    drake::systems::DiscreteValues<double>* discrete_state) const {
  const OutputVector<double>* trifinger_state =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  const auto& fingertips_delta_pos_lcm_msg =
      this->EvalInputValue<dairlib::lcmt_fingertips_delta_position>(
          context, fingertips_delta_position_port_);

  // Evaluate the current positions of the fingertips.
  Eigen::VectorXd q_trifinger = trifinger_state->GetPositions();
  multibody::SetPositionsIfNew<double>(plant_, q_trifinger, context_);

  auto fingertip_0_pos =
      plant_
          .EvalBodyPoseInWorld(*context_,
                               plant_.GetBodyByName(fingertip_0_name_))
          .translation();
  auto fingertip_120_pos =
      plant_
          .EvalBodyPoseInWorld(*context_,
                               plant_.GetBodyByName(fingertip_120_name_))
          .translation();
  auto fingertip_240_pos =
      plant_
          .EvalBodyPoseInWorld(*context_,
                               plant_.GetBodyByName(fingertip_240_name_))
          .translation();
  Eigen::VectorXd cur_fingertips_pos(9);
  cur_fingertips_pos << fingertip_0_pos, fingertip_120_pos, fingertip_240_pos;
  discrete_state->get_mutable_vector(cur_fingertips_pos_idx_)
      .set_value(cur_fingertips_pos);

  Eigen::VectorXd fingertips_target_pos = cur_fingertips_pos;
  auto current_msg_timestamp = fingertips_delta_pos_lcm_msg->utime;
  auto previous_msg_timestamp = static_cast<int64_t>(
      context.get_discrete_state(prev_target_timestamp_idx_)[0]);

  // check if the obtained message from lcm input port is still old one,
  // if so, no update is performed on the discrete states.
  if (current_msg_timestamp == previous_msg_timestamp) {
    if (fingertips_delta_pos_lcm_msg->utime > 0) {
      return drake::systems::EventStatus::Succeeded();
    }
    // message not yet coming, update the current fingertip targets to
    // the current state and maintain these targets until new messages come.
    if (previous_msg_timestamp == -1) {
      discrete_state->get_mutable_vector(fingertips_target_idx_)
          .set_value(fingertips_target_pos);
      discrete_state->get_mutable_vector(prev_target_timestamp_idx_)
          .set_value(Eigen::VectorXd::Zero(1));
    }
  } else {
    fingertips_target_pos +=
        Eigen::VectorXd::Map(fingertips_delta_pos_lcm_msg->deltaPos, 9);
    discrete_state->get_mutable_vector(fingertips_target_idx_)
        .set_value(fingertips_target_pos);
    discrete_state->get_mutable_vector(prev_target_timestamp_idx_)
        .set_value(
            (Eigen::VectorXd::Ones(1) * fingertips_delta_pos_lcm_msg->utime));
  }
  return drake::systems::EventStatus::Succeeded();
}

void FingertipDeltaPositionReceiver::CopyToOutputFingertipsTargetTraj(
    const drake::systems::Context<double>& context,
    Trajectory<double>* target_traj) const {
  // retrieve fingertips target positions from the current discrete state.
  auto fingertips_target_pos =
      context.get_discrete_state(fingertips_target_idx_).get_value();
  auto cur_fingertips_pos =
      context.get_discrete_state(cur_fingertips_pos_idx_).get_value();

  double delta_pos_update_period = 1.0 / delta_pos_update_frequency_;

  auto cur_context_time = context.get_time();
  Eigen::VectorXd knots(2);
  knots << cur_context_time, cur_context_time + delta_pos_update_period;
  Eigen::MatrixXd samples(cur_fingertips_pos.size(), 2);
  samples.col(0) = cur_fingertips_pos;
  samples.col(1) = fingertips_target_pos;

  // generate a constant trajectory that can be fed to OSC controller.
  auto traj = PiecewisePolynomial<double>::FirstOrderHold(knots, samples);
  auto casted_target_traj =
      (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
          target_traj);
  *casted_target_traj = traj;
}

void FingertipDeltaPositionReceiver::CopyToOutputFingertipsTarget(
    const drake::systems::Context<double>& context,
    TimestampedVector<double>* fingertips_target) const {
  // retrieve fingertips target positions from the current discrete state.
  auto fingertips_target_pos =
      context.get_discrete_state(fingertips_target_idx_).get_value();
  fingertips_target->SetDataVector(fingertips_target_pos);
  fingertips_target->set_timestamp(context.get_time());
}

void FingertipDeltaPositionReceiver::CopyToOutputCurrentFingertipPositions(
    const drake::systems::Context<double>& context,
    systems::TimestampedVector<double>* cur_fingertips_pos) const {
  cur_fingertips_pos->SetDataVector(
      context.get_discrete_state(cur_fingertips_pos_idx_).get_value());
  cur_fingertips_pos->set_timestamp(context.get_time());
}

}  // namespace dairlib::systems